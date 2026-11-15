#include "game/cricket_app.h"

#include "core/log.h"
#include "debug/axis_gizmo.h"
#include "platform/imgui_platform.h"

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace game {

namespace {

// The limb test's targets in F's order.
constexpr motion::Goal kPoseOrder[] = {motion::Goal::FootR, motion::Goal::FootL, motion::Goal::HandR, motion::Goal::HandL,
                                       motion::Goal::Hips,  motion::Goal::Chest, motion::Goal::Head};
constexpr int kPoseCount = static_cast<int>(sizeof(kPoseOrder) / sizeof(kPoseOrder[0]));

core::LoopSettings loopSettings(const core::Config& c) {
    core::LoopSettings s;
    s.simHz = c.getFloat("sim.tickHz", 120.0f);
    s.maxStepsPerFrame = c.getInt("sim.maxStepsPerFrame", 8);
    return s;
}

render::LightingDesc readLighting(const core::Config& c) {
    render::LightingDesc l;
    l.sunDirection = c.getVec3("lighting.sunDirection", l.sunDirection);
    l.sunColor = c.getVec3("lighting.sunColor", l.sunColor);
    l.sunIlluminance = c.getFloat("lighting.sunIlluminance", l.sunIlluminance);
    l.skyColor = c.getVec3("lighting.skyColor", l.skyColor);
    l.horizonColor = c.getVec3("lighting.horizonColor", l.horizonColor);
    l.groundColor = c.getVec3("lighting.groundColor", l.groundColor);
    l.iblIntensity = c.getFloat("lighting.iblIntensity", l.iblIntensity);
    return l;
}

} // namespace

CricketApp::CricketApp(AppConfigs configs) : core::App(loopSettings(configs.engine)), cfg_(std::move(configs)) {}

bool CricketApp::init() {
    const core::Config& e = cfg_.engine;

    platform::WindowDesc wd;
    wd.title = e.getString("window.title", wd.title);
    wd.width = e.getInt("window.width", wd.width);
    wd.height = e.getInt("window.height", wd.height);
    wd.resizable = e.getBool("window.resizable", true);
#ifndef CRICKET_DEV_BUILD
    wd.dock = e.getString("window.dockRelease", "fullscreen");
#else
    wd.dock = e.getString("window.dockDebug", wd.dock);
#endif
    wd.maxScreenFraction = e.getFloat("window.maxScreenFraction", wd.maxScreenFraction);
    if (!window_.create(wd)) return false;

    vsync_ = e.getBool("window.vsync", true);
    if (vsync_) targetFrameSeconds_ = 1.0f / window_.refreshRate();

    // Dear ImGui context first: the renderer registers itself as ImGui's backend.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;  // no imgui.ini next to the exe; layout is code-defined
    ImGui::StyleColorsDark();

    render::RendererDesc rd;
    rd.nativeWindow = window_.nativeHandle();
    rd.width = window_.pixelWidth();
    rd.height = window_.pixelHeight();
    rd.backend = e.getString("render.backend", rd.backend);
    rd.vsync = vsync_;
    rd.shadowMapSize = e.getInt("render.shadowMapSize", rd.shadowMapSize);
    rd.shadowCascades = e.getInt("render.shadowCascades", rd.shadowCascades);
    rd.shadowFar = e.getFloat("render.shadowFar", rd.shadowFar);
    rd.debugLineCapacity = static_cast<size_t>(e.getInt("render.debugLineCapacity", 65536));
    rd.assetDir = e.getString("render.assetDir", rd.assetDir);
    renderer_ = render::createFilamentRenderer();
    if (!renderer_->init(rd)) return false;
    backendName_ = rd.backend;

    renderer_->setLighting(readLighting(e));

    if (!platform::imguiPlatformInit(window_)) return false;
    imguiReady_ = true;

    actions_.load(cfg_.input);
    camera_.configure(e, rd.width, rd.height);
    sim_.configure(e);

    if (!sim_.initPhysics(cfg_.physicsWorld, cfg_.scene)) return false;
    camera_.setGround(&sim_.physicsWorld());  // the eye stays above the one ground
    scene_ = buildTestScene(world_, *renderer_, cfg_.scene);

    rig::SkeletonDef skeletonDef;
    if (!rig::loadSkeletonDef(cfg_.humanSkeleton, skeletonDef) || !sim_.initFielder(skeletonDef, cfg_.humanJoints, cfg_.locomotion)) return false;
    const agents::Human& fielder = sim_.fielder();
    humanVisual_.build(world_, *renderer_, fielder.def(), fielder.rig(), cfg_.humanSkeleton);
    fielderModels_.assign(static_cast<size_t>(fielder.rig().numJoints()), math::Mat4(1.0f));
    limbTest_ = e.getBool("debug.limbTest", false);
    debugSettings_.shadows = e.getBool("render.shadows", false);
    debugSettings_.debugDraw = e.getBool("debug.debugDraw", true);
    debugSettings_.showOverlay = e.getBool("debug.showOverlay", true);
    debugSettings_.cameraSpeeds = camera_.speeds();
    debugSettings_.defaultCameraSpeeds = camera_.speeds();
    appliedSettings_ = debugSettings_;
    appliedSettings_.shadows = !debugSettings_.shadows;  // force the first apply
    appliedSettings_.showProbe = !debugSettings_.showProbe;

    if (cfg_.liveReload) configWatcher_.watch(cfg_.dir);
    return true;
}

void CricketApp::applyEngineConfig() {
    const core::Config& e = cfg_.engine;
    renderer_->setLighting(readLighting(e));
    camera::CameraSpeeds speeds = camera::readCameraSpeeds(e);
    debugSettings_.cameraSpeeds = speeds;  // applied by renderFrame's change check
    debugSettings_.defaultCameraSpeeds = speeds;
    debugSettings_.shadows = e.getBool("render.shadows", debugSettings_.shadows);
    debugSettings_.debugDraw = e.getBool("debug.debugDraw", debugSettings_.debugDraw);
}

void CricketApp::reloadConfig(const std::vector<std::string>& files) {
    for (const std::string& name : files) {
        core::Config fresh;
        if (!fresh.load(cfg_.dir + "/" + name)) {
            LOG_WARN("reload: {} has errors, keeping the previous values", name);
            continue;  // e.g. saved mid-edit with a JSON typo
        }
        if (name == "engine.json") {
            cfg_.engine = std::move(fresh);
            applyEngineConfig();
        } else if (name == "input.json") {
            cfg_.input = std::move(fresh);
            actions_.load(cfg_.input);
        } else if (name == "scene.json") {
            cfg_.scene = std::move(fresh);
            destroyTestScene(world_, *renderer_, scene_);
            scene_ = buildTestScene(world_, *renderer_, cfg_.scene);
            appliedSettings_.showProbe = !debugSettings_.showProbe;  // re-apply probe visibility
        } else if (name == "physics_world.json") {
            cfg_.physicsWorld = std::move(fresh);
            if (!sim_.initPhysics(cfg_.physicsWorld, cfg_.scene)) LOG_ERROR("reload: physics world could not be rebuilt");
        } else if (name == "human_skeleton.json") {
            cfg_.humanSkeleton = std::move(fresh);
            LOG_INFO("reload: human_skeleton.json changed; restart to rebuild the fielder");
            continue;
        } else if (name == "human_joints.json") {
            cfg_.humanJoints = std::move(fresh);
            agents::Human& f = sim_.fielder();
            f.init(f.def(), f.rig().height(), cfg_.humanJoints, cfg_.locomotion);  // limits + IK settings
        } else if (name == "locomotion.json") {
            cfg_.locomotion = std::move(fresh);
            sim_.fielder().loadLocomotion(cfg_.locomotion);
        } else {
            continue;  // not a file this app uses
        }
        LOG_INFO("reload: applied {}", name);
    }
}

void CricketApp::shutdown() {
    if (imguiReady_) platform::imguiPlatformShutdown();
    imguiReady_ = false;
    world_.registry().clear();
    if (renderer_) renderer_->shutdown();  // releases ImGui textures, so before DestroyContext
    renderer_.reset();
    if (ImGui::GetCurrentContext()) ImGui::DestroyContext();
    window_.destroy();
}

bool CricketApp::pumpEvents() {
    input_.beginFrame();
    window_.pumpEvents(input_, events_);

    core::Event ev;
    bool running = true;
    while (events_.pop(ev)) {
        switch (ev.type) {
            case core::EventType::Quit: running = false; break;
            case core::EventType::WindowResized:
                renderer_->resize(ev.width, ev.height);
                camera_.setViewport(ev.width, ev.height);
                break;
            default: break;
        }
    }

    if (imguiReady_) {
        const ImGuiIO& io = ImGui::GetIO();
        input_.uiWantsKeyboard = io.WantCaptureKeyboard;
        input_.uiWantsMouse = io.WantCaptureMouse;
    }

    if (actions_.pressed(input_, core::Action::Quit)) running = false;
    if (actions_.pressed(input_, core::Action::ToggleMouseLook)) mouseLook_ = !mouseLook_;
    window_.setRelativeMouse(mouseLook_);
    if (actions_.pressed(input_, core::Action::ToggleOverlay)) debugSettings_.showOverlay = !debugSettings_.showOverlay;
    if (actions_.pressed(input_, core::Action::ToggleDebugDraw)) debugSettings_.debugDraw = !debugSettings_.debugDraw;

    // Human movement: the fielder always turns to the camera's heading,
    // standing or walking, unless free view (V) holds the heading; W walks forward, S backs up, A/D sidestep
    // (character space, +X left).
    auto axis = [&](core::Action pos, core::Action neg) {
        return (actions_.down(input_, pos) ? 1.0f : 0.0f) - (actions_.down(input_, neg) ? 1.0f : 0.0f);
    };
    const camera::CameraPose view = camera_.pose();
    const math::Vec3 ahead = math::safeNormalize(math::Vec3(view.target.x - view.eye.x, 0.0f, view.target.z - view.eye.z),
                                                 -math::kForward);
    math::Vec2 move(axis(core::Action::StepLeft, core::Action::StepRight),
                    axis(core::Action::StepForward, core::Action::StepBack));
    // I: slow motion (the whole game) on/off.
    if (actions_.pressed(input_, core::Action::SlowMotion)) {
        slowMotion_ = !slowMotion_;
        setTimeScale(slowMotion_ ? cfg_.engine.getFloat("debug.slowMotionFactor", 0.2f) : 1.0);
        LOG_INFO("Slow motion {}", slowMotion_ ? "on" : "off");
    }
    agents::Human& fielder = sim_.fielder();
    if (actions_.pressed(input_, core::Action::FreeView)) freeView_ = !freeView_;
    if (!freeView_) heading_ = std::atan2(ahead.x, ahead.z);
    // WASD moves the hips (the user's design, 2026-11-15); the body's
    // balance steps follow. The heading only turns the player. The jump is
    // not used (reach and dive will come from the same push).
    fielder.setFacing(true, heading_);
    fielder.setStepInput(move);
    fielder.setPace(actions_.down(input_, core::Action::ModShift) ? 2 : actions_.down(input_, core::Action::Jog) ? 1 : 0);
    if (actions_.pressed(input_, core::Action::WalkFaster)) fielder.changeWalkSpeed(1);
    if (actions_.pressed(input_, core::Action::WalkSlower)) fielder.changeWalkSpeed(-1);

    // Live config: saved files are re-applied to the running game; F5 forces all.
    changedConfigs_.clear();
    if (actions_.pressed(input_, core::Action::ReloadConfig))
        changedConfigs_ = {"engine.json", "input.json", "scene.json", "human_skeleton.json", "locomotion.json",
                          "physics_world.json"};
    else if (cfg_.liveReload)
        configWatcher_.poll(changedConfigs_);
    if (!changedConfigs_.empty()) reloadConfig(changedConfigs_);
    return running;
}

void CricketApp::updatePose(float /*seconds: the target jumps per press, not per second*/) {
    agents::Human& fielder = sim_.fielder();
    if (actions_.pressed(input_, core::Action::ResetPose)) {
        // R: the targets go and the body is back at its standing pose.
        fielder.clearPose();
        fielder.resetPhysicsBody();
        poseActive_ = false;
    }
    if (actions_.pressed(input_, core::Action::PoseLimb)) {
        poseLimb_ = (poseLimb_ + 1) % kPoseCount;
        poseActive_ = false;
    }
    if (input_.uiWantsKeyboard) return;
    // Each press jumps the target one step (no slide).
    auto axis = [&](core::Action pos, core::Action neg) {
        return (actions_.pressed(input_, pos) ? 1.0f : 0.0f) - (actions_.pressed(input_, neg) ? 1.0f : 0.0f);
    };
    const float side = axis(core::Action::PoseRight, core::Action::PoseLeft);
    const float ahead = axis(core::Action::PoseAway, core::Action::PoseNear);
    if (side == 0.0f && ahead == 0.0f) return;

    // The target starts where its part is and as it is turned (the drawn
    // pose); each arrow press jumps it one step relative to the camera on the
    // ground plane, or up/down with Shift; with Ctrl (the chest and head
    // always) it turns instead.
    const motion::Goal goal = kPoseOrder[poseLimb_];
    const int joint = fielder.goalJoint(goal);
    if (joint < 0) return;
    if (!poseActive_) {
        const math::Transform bone = fielder.root() * math::toTransform(fielderModels_[static_cast<size_t>(joint)]);
        poseTarget_ = bone.position;
        poseTurn_ = bone.rotation;
        poseTurned_ = false;
        poseActive_ = true;
    }
    const camera::CameraPose view = camera_.pose();
    const math::Vec3 forward =
        math::safeNormalize(math::Vec3(view.target.x - view.eye.x, 0.0f, view.target.z - view.eye.z), -math::kForward);
    const math::Vec3 right = math::cross(forward, math::kUp);
    const bool shift = actions_.down(input_, core::Action::ModShift);
    const bool turnOnly = goal == motion::Goal::Head;
    if (turnOnly || actions_.down(input_, core::Action::PoseTurn)) {
        // Left/Right about up; Up/Down tip it away from / toward the camera
        // (about the camera's right); with Shift, Left/Right roll it about
        // the camera's view.
        const float step = math::radians(cfg_.engine.getFloat("debug.limbTestTurnDeg", 15.0f));
        math::Quat turn = shift ? math::fromAxisAngle(forward, side * step)
                                : math::fromAxisAngle(math::kUp, -side * step) * math::fromAxisAngle(right, -ahead * step);
        poseTurn_ = math::normalize(turn * poseTurn_);
        poseTurned_ = true;
    } else {
        const math::Vec3 move = shift ? math::kUp * ahead + right * side : forward * ahead + right * side;
        poseTarget_ += move * cfg_.engine.getFloat("debug.limbTestStep", 0.1f);
        // The target goes anywhere in the air, never under the ground (y = 0);
        // the part reaches as near as the body allows.
        poseTarget_.y = std::max(poseTarget_.y, 0.0f);
    }
    fielder.setPoseGoal(goal, poseTarget_, poseTurn_, poseTurned_);
}

void CricketApp::fixedUpdate(double dt) {
    sim_.tick(dt);
}

void CricketApp::renderFrame(double frameSeconds, float alpha) {
    auto frameStart = std::chrono::steady_clock::now();

    // --- Cameras (render-side, real frame time); the orbit follows the controlled player ---
    camera_.setFollowPoint(sim_.fielder().interpolatedRoot(alpha).position);
    camera_.setMouseLook(mouseLook_);
    camera_.update(input_, static_cast<float>(frameSeconds));
    camera::CameraPose pose = camera_.pose();
    render::CameraDesc cam;
    cam.eye = pose.eye;
    cam.target = pose.target;
    cam.up = pose.up;
    cam.fovYDegrees = cfg_.engine.getFloat("camera.fovY", 50.0f);
    cam.nearPlane = cfg_.engine.getFloat("camera.near", 0.1f);
    cam.farPlane = cfg_.engine.getFloat("camera.far", 2000.0f);
    renderer_->setCamera(cam);

    // --- Apply toggles that changed ---
    if (debugSettings_.shadows != appliedSettings_.shadows) renderer_->setShadowsEnabled(debugSettings_.shadows);
    if (debugSettings_.debugDraw != appliedSettings_.debugDraw) renderer_->setDebugDrawEnabled(debugSettings_.debugDraw);
    if (!(debugSettings_.cameraSpeeds == appliedSettings_.cameraSpeeds)) camera_.setSpeeds(debugSettings_.cameraSpeeds);
    if (debugSettings_.showProbe != appliedSettings_.showProbe) {
        sim_.setProbeEnabled(debugSettings_.showProbe);
        renderer_->setObjectVisible(world_.registry().get<Renderable>(scene_.probe).object, debugSettings_.showProbe);
    }
    appliedSettings_ = debugSettings_;

    // --- Interpolated sim state -> entities -> renderer scene ---
    math::Transform probe = sim_.probeTransform(alpha);
    for (auto e : world_.registry().view<FollowsProbe>()) world_.setTransform(e, probe);
    if (limbTest_) updatePose(static_cast<float>(frameSeconds));
    updateFielder(alpha);
    world_.syncToRenderer(*renderer_);

    if (debugSettings_.debugDraw) drawDebugGeometry(alpha);

    // --- Debug UI ---
    platform::imguiPlatformNewFrame();
    ImGui::NewFrame();
    debug::OverlayInfo info;
    info.stats = &stats();
    info.simHz = cfg_.engine.getFloat("sim.tickHz", 120.0f);
    info.renderAlpha = alpha;
    info.debugLines = renderer_->debugDraw().vertexCount() / 2;
    info.debugLinesDropped = renderer_->debugDraw().droppedLines();
    info.backend = backendName_.c_str();
    debug::drawOverlay(debugSettings_, info);
    debug::drawAxisGizmo(pose.eye, pose.target, pose.up);
    ImGui::Render();

    renderer_->render(ImGui::GetDrawData());

    // Frame limiter backs up the swap chain's vsync (and caps the rate if the
    // driver ignores it), keeping the GPU from rendering hundreds of frames.
    if (targetFrameSeconds_ > 0.0f) {
        double spent = std::chrono::duration<double>(std::chrono::steady_clock::now() - frameStart).count();
        platform::preciseSleep(targetFrameSeconds_ - spent - 0.0005);
    }
}

void CricketApp::updateFielder(float alpha) {
    const agents::Human& fielder = sim_.fielder();
    fielder.interpolatedPose(alpha, fielderPose_);
    if (!fielder.rig().localToModel(fielderPose_, fielderModels_)) return;
    fielder.physicalModels(sim_.physicsWorld(), alpha, fielderModels_);  // drawn from its physics body
    humanVisual_.update(world_, fielder.rig(), fielderModels_, fielder.interpolatedRoot(alpha));
}

void CricketApp::drawDebugGeometry(float alpha) {
    render::DebugDraw& dd = renderer_->debugDraw();
    if (debugSettings_.showGrid) dd.grid(math::Vec3(0, 0.02f, 0), 30.0f, 5.0f, render::rgba(255, 255, 255, 60));
    dd.axes(math::Transform::identity(), 2.0f);

    // Pitch outline and stump lines.
    float halfLen = 0.5f * scene_.pitchLength;
    dd.box(math::Vec3(0, 0.36f, halfLen), math::Vec3(0.115f, 0.36f, 0.02f), render::colors::kYellow);
    dd.box(math::Vec3(0, 0.36f, -halfLen), math::Vec3(0.115f, 0.36f, 0.02f), render::colors::kYellow);

    // Player's local axes at its root: X red (left), Y green (up), Z blue (forward).
    dd.axes(sim_.fielder().interpolatedRoot(alpha), 0.5f);

    // Limb test: the chosen limb's target (red once moved, yellow at the
    // limb before that).
    const agents::Human& fielder = sim_.fielder();
    if (limbTest_) {
        // The chosen target (yellow at its part until moved, red after) and,
        // once turned, its axes.
        const int joint = fielder.goalJoint(kPoseOrder[poseLimb_]);
        if (joint >= 0) {
            const math::Transform bone =
                fielder.interpolatedRoot(alpha) * math::toTransform(fielderModels_[static_cast<size_t>(joint)]);
            dd.sphere(poseActive_ ? poseTarget_ : bone.position, 0.05f, poseActive_ ? render::colors::kRed : render::colors::kYellow);
            if (poseActive_ && poseTurned_) dd.axes(math::Transform{poseTarget_, poseTurn_}, 0.2f);
        }
    }
    // The fielder's physics body (HumanBody): each part's Jolt shape (white)
    // and each joint at its pivot: hinge (yellow, a line along its axis),
    // swing-twist (cyan, its twist X red, Y green, Z blue axes); a joint with
    // a muscle (motor) has a red ring.
    drawHumanBody(sim_.fielder().humanBody(), alpha);

    // The fielder's capsule (white while it holds the body, red once let go)
    // and the line the release reads: from the centre of the feet (yellow
    // dot) to the hips (green while held, red let go).
    auto drawCapsule = [&](const agents::Human& h) {
        math::Vec3 feet, feetCentre, hips;
        float height = 0.0f, radius = 0.0f;
        if (!h.capsuleDebug(feet, height, radius, feetCentre, hips)) return;
        const render::Color c = h.supported() ? render::colors::kWhite : render::colors::kRed;
        const math::Vec3 low = feet + math::kUp * radius, high = feet + math::kUp * std::max(height - radius, radius);
        dd.circle(low, math::kUp, radius, c, 16);
        dd.circle(high, math::kUp, radius, c, 16);
        for (const math::Vec3 s : {math::kRight, -math::kRight, math::kForward, -math::kForward})
            dd.line(low + s * radius, high + s * radius, c);
        dd.sphere(low, radius, c, 8);
        dd.sphere(high, radius, c, 8);
        dd.sphere(feetCentre, 0.03f, render::colors::kYellow, 8);
        dd.line(feetCentre, hips, h.supported() ? render::colors::kGreen : render::colors::kRed);
        // The decider's targets and answer (DESIGN "Debug drawing"): a
        // pinned foot's anchor as a ring sized by its support weight
        // (white), a driven target magenta, the plan's pelvis (the hold's
        // target) and its offset line from the capsule (cyan; red when
        // clamped at its bound), a reach short of its target as a red line
        // from the end to the target.
        const motion::TargetSet& t = h.bodyTargets();
        const motion::BodyQueue& q = h.bodyQueue();
        if (!q.plan) return;
        const math::Transform& root = h.root();
        for (int i = 0; i < 2; ++i) {
            const math::Vec3 at = root.transformPoint(t.foot[i].position);
            if (t.foot[i].mode == motion::TargetMode::Pinned)
                dd.circle(at, math::kUp, 0.02f + 0.08f * q.supportWeight[i], render::colors::kWhite, 12);
            else if (t.foot[i].mode == motion::TargetMode::Driven)
                dd.sphere(at, 0.025f, render::colors::kMagenta, 8);
        }
        for (int i = 0; i < 2; ++i)
            if (t.hand[i].mode == motion::TargetMode::Driven)
                dd.sphere(root.transformPoint(t.hand[i].position), 0.025f, render::colors::kMagenta, 8);
        const math::Vec3 pelvis = root.transformPoint(math::Vec3((*q.plan)[0][3]));
        dd.sphere(pelvis, 0.03f, render::colors::kCyan, 8);
        dd.line(feet, pelvis, render::colors::kCyan);
        for (int e = 0; e < 4; ++e) {
            const motion::ReachReport& r = q.reach[e];
            if (r.reached) continue;
            const math::Vec3 target = root.transformPoint(e < 2 ? t.hand[e].position : t.foot[e - 2].position);
            dd.line(target - r.direction * r.shortfall, target, render::colors::kRed);
        }
    };
    drawCapsule(sim_.fielder());

    if (debugSettings_.showProbe) {
        math::Transform probe = sim_.probeTransform(alpha);
        dd.axes(probe, 0.6f);
        dd.sphere(probe.position, 0.4f, render::colors::kCyan);
    }
}

void CricketApp::drawHumanBody(const agents::HumanBody& body, float alpha) {
    if (!body.ready()) return;
    render::DebugDraw& dd = renderer_->debugDraw();
    const physics::PhysicsWorld& world = sim_.physicsWorld();
    const render::Color shapeColor = render::rgba(255, 140, 0);  // orange (the kit is white)
    for (const agents::HumanBody::Part& p : body.parts()) {
        const math::Transform t = world.interpolated(p.body, alpha);
        if (p.shapeType == physics::ShapeType::Capsule) {
            const math::Vec3 up = t.transformDirection(math::kUp), side = t.transformDirection(math::kRight),
                             front = t.transformDirection(math::kForward);
            const math::Vec3 a = t.position - up * p.halfHeight, b = t.position + up * p.halfHeight;
            dd.circle(a, up, p.radius, shapeColor, 12);
            dd.circle(b, up, p.radius, shapeColor, 12);
            for (const math::Vec3 s : {side, -side, front, -front}) dd.line(a + s * p.radius, b + s * p.radius, shapeColor);
            dd.sphere(a, p.radius, shapeColor, 8);
            dd.sphere(b, p.radius, shapeColor, 8);
        } else if (p.shapeType == physics::ShapeType::Box) {
            const math::Vec3& h = p.halfExtents;
            math::Vec3 c[8];
            for (int i = 0; i < 8; ++i)
                c[i] = t.transformPoint(math::Vec3(i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z));
            for (int i = 0; i < 8; ++i)
                for (int bit : {1, 2, 4})
                    if (!(i & bit)) dd.line(c[i], c[i | bit], shapeColor);
        } else {
            dd.sphere(t.position, p.radius, shapeColor, 8);
        }
        if (p.parent < 0) continue;
        const math::Transform bone = t * math::inverse(p.shape);  // the joint sits at the bone's origin
        if (p.hinge) {
            const math::Vec3 axis = bone.transformDirection(p.frameX);
            dd.sphere(bone.position, 0.02f, render::colors::kYellow, 8);
            dd.line(bone.position - axis * 0.08f, bone.position + axis * 0.08f, render::colors::kYellow);
        } else {
            dd.sphere(bone.position, 0.02f, render::colors::kCyan, 8);
            dd.line(bone.position, bone.position + bone.transformDirection(p.frameX) * 0.08f, render::colors::kRed);
            dd.line(bone.position, bone.position + bone.transformDirection(p.frameY) * 0.08f, render::colors::kGreen);
            dd.line(bone.position, bone.position + bone.transformDirection(p.frameZ) * 0.08f, render::colors::kBlue);
        }
        if (p.muscle) dd.circle(bone.position, bone.transformDirection(p.frameX), 0.035f, render::colors::kRed, 12);
    }
}

} // namespace game
