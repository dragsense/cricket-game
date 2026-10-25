#include "game/cricket_app.h"

#include "core/log.h"
#include "debug/axis_gizmo.h"
#include "platform/imgui_platform.h"

#include <imgui.h>

#include <chrono>
#include <cmath>

namespace game {

namespace {

constexpr float kStoppedSpeed = 0.05f;  // m/s; below this the fielder counts as stopped

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
#ifdef NDEBUG
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
    feeder_.configure(e);

    scene_ = buildTestScene(world_, *renderer_, cfg_.scene);
    ballPos_ = cfg_.scene.getVec3("ball.restPosition", math::Vec3(0.15f, 0.0f, 5.0f));
    ballPos_.y = cfg_.scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi;

    rig::SkeletonDef skeletonDef;
    if (!rig::loadSkeletonDef(cfg_.fielderSkeleton, skeletonDef) || !sim_.initFielder(skeletonDef, cfg_.fielderJoints, cfg_.locomotion)) return false;
    const agents::Fielder& fielder = sim_.fielder();
    fielderVisual_.build(world_, *renderer_, fielder.def(), fielder.rig(), cfg_.fielderSkeleton);
    fielderModels_.assign(static_cast<size_t>(fielder.rig().numJoints()), math::Mat4(1.0f));

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
            feeder_.configure(cfg_.engine);
        } else if (name == "input.json") {
            cfg_.input = std::move(fresh);
            actions_.load(cfg_.input);
        } else if (name == "scene.json") {
            cfg_.scene = std::move(fresh);
            destroyTestScene(world_, *renderer_, scene_);
            scene_ = buildTestScene(world_, *renderer_, cfg_.scene);
            appliedSettings_.showProbe = !debugSettings_.showProbe;  // re-apply probe visibility
        } else if (name == "fielder_skeleton.json") {
            cfg_.fielderSkeleton = std::move(fresh);
            LOG_INFO("reload: fielder_skeleton.json changed; restart to rebuild the fielder");
            continue;
        } else if (name == "fielder_joints.json") {
            cfg_.fielderJoints = std::move(fresh);
            agents::Fielder& f = sim_.fielder();
            f.init(f.def(), f.rig().height(), cfg_.fielderJoints, cfg_.locomotion);  // limits + IK settings
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

    // Fielder movement: W turns toward the camera's heading and walks forward;
    // A/D sidestep and S backs up without turning (character space, +X left).
    // The heading is taken from the camera only while stopped, so the camera
    // can orbit a moving fielder without steering them.
    auto axis = [&](core::Action pos, core::Action neg) {
        return (actions_.down(input_, pos) ? 1.0f : 0.0f) - (actions_.down(input_, neg) ? 1.0f : 0.0f);
    };
    const camera::CameraPose view = camera_.pose();
    const math::Vec3 ahead = math::safeNormalize(math::Vec3(view.target.x - view.eye.x, 0.0f, view.target.z - view.eye.z),
                                                 -math::kForward);
    const math::Vec2 move(axis(core::Action::StepLeft, core::Action::StepRight),
                          axis(core::Action::StepForward, core::Action::StepBack));
    if (sim_.fielder().speed() < kStoppedSpeed) heading_ = std::atan2(ahead.x, ahead.z);
    sim_.fielder().setMoveInput(move, actions_.down(input_, core::Action::StepForward), heading_);

    // Pace: walk by default, hold Shift to run, Ctrl to jog (Shift wins).
    sim_.fielder().setGait(actions_.down(input_, core::Action::Run)   ? motion::Gait::Run
                           : actions_.down(input_, core::Action::Jog) ? motion::Gait::Jog
                                                                      : motion::Gait::Walk);

    if (actions_.pressed(input_, core::Action::ClearReach)) sim_.fielder().clearReach();
    if (actions_.pressed(input_, core::Action::Pick)) sim_.fielder().startPick();
    {
        agents::Fielder& f = sim_.fielder();
        int mode = -1;
        if (actions_.pressed(input_, core::Action::CycleHand)) mode = (static_cast<int>(f.handMode()) + 1) % 4;
        if (actions_.pressed(input_, core::Action::HandLeft)) mode = static_cast<int>(agents::HandMode::Left);
        if (actions_.pressed(input_, core::Action::HandRight)) mode = static_cast<int>(agents::HandMode::Right);
        if (actions_.pressed(input_, core::Action::HandBoth)) mode = static_cast<int>(agents::HandMode::Both);
        if (mode >= 0) {
            f.setHandMode(static_cast<agents::HandMode>(mode));
            constexpr const char* kNames[] = {"auto", "left", "right", "both"};
            LOG_INFO("picking hand: {}", kNames[mode]);
        }
    }
    if (actions_.pressed(input_, core::Action::ThrowBall)) {
        sim_.fielder().clearReach();
        feeder_.throwAt(sim_.fielder().root(), cfg_.scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi);
    }

    // Live config: saved files are re-applied to the running game; F5 forces all.
    changedConfigs_.clear();
    if (actions_.pressed(input_, core::Action::ReloadConfig))
        changedConfigs_ = {"engine.json", "input.json", "scene.json", "fielder_skeleton.json", "locomotion.json"};
    else if (cfg_.liveReload)
        configWatcher_.poll(changedConfigs_);
    if (!changedConfigs_.empty()) reloadConfig(changedConfigs_);
    return running;
}

void CricketApp::fixedUpdate(double dt) {
    // A thrown ball is the fielder's target every tick until it is caught
    // (then it rides in the hand) or has gone past.
    if (feeder_.active() && sim_.fielder().holding()) feeder_.stop();
    if (feeder_.active()) {
        feeder_.tick(static_cast<float>(dt));
        sim_.fielder().setReachTarget(feeder_.position(), feeder_.velocity());
    }
    sim_.tick(dt);
}

void CricketApp::renderFrame(double frameSeconds, float alpha) {
    auto frameStart = std::chrono::steady_clock::now();

    updateReachBall(static_cast<float>(frameSeconds));

    // --- Cameras (render-side, real frame time); the orbit follows the fielder ---
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

void CricketApp::updateReachBall(float dt) {
    const float radius = cfg_.scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi;
    bool moved = false;

    // E / Q: raise or lower the ball (never below the ground).
    const float lift = (actions_.down(input_, core::Action::BallUp) ? 1.0f : 0.0f) -
                       (actions_.down(input_, core::Action::BallDown) ? 1.0f : 0.0f);
    if (lift != 0.0f && !sim_.fielder().holding() && !feeder_.active()) {  // held or thrown: not ours
        ballPos_.y = std::max(radius, ballPos_.y + lift * cfg_.input.getFloat("ballLiftSpeed", 1.0f) * dt);
        moved = true;
    }

    // Left-drag: ray through the cursor from the camera (vertical FOV, window
    // aspect), hit with the level plane through the ball.
    const float w = static_cast<float>(window_.width()), h = static_cast<float>(window_.height());
    if (input_.down(core::MouseButton::Left) && !input_.uiWantsMouse && !mouseLook_ && w > 0.0f && h > 0.0f) {
        const camera::CameraPose view = camera_.pose();
        const math::Vec3 forward = math::normalize(view.target - view.eye);
        const math::Vec3 right = math::normalize(math::cross(forward, view.up));
        const math::Vec3 up = math::cross(right, forward);
        const float tanHalf = std::tan(0.5f * math::radians(cfg_.engine.getFloat("camera.fovY", 50.0f)));
        const float x = (2.0f * input_.mouseX / w - 1.0f) * tanHalf * (w / h);
        const float y = (1.0f - 2.0f * input_.mouseY / h) * tanHalf;
        const math::Vec3 ray = math::normalize(forward + right * x + up * y);
        float t = std::fabs(ray.y) > 1e-4f ? (ballPos_.y - view.eye.y) / ray.y : -1.0f;
        feeder_.stop();  // manual placement takes over from a throw
        // A click on a held ball takes it from the hand and puts it down there.
        if (sim_.fielder().holding()) {
            ballPos_.y = radius;
            t = std::fabs(ray.y) > 1e-4f ? (ballPos_.y - view.eye.y) / ray.y : -1.0f;
        }
        if (t > 0.0f) {
            const math::Vec3 hit = view.eye + ray * t;
            ballPos_.x = hit.x;
            ballPos_.z = hit.z;
            moved = true;
        }
    }

    // Placing the ball only sets it; Space makes the fielder pick it.
    if (!moved) return;
    math::Transform ball;
    ball.position = ballPos_;
    world_.setTransform(scene_.ball, ball);
    sim_.fielder().setReachTarget(ballPos_);
}

void CricketApp::updateFielder(float alpha) {
    const agents::Fielder& fielder = sim_.fielder();
    fielder.interpolatedPose(alpha, fielderPose_);
    if (!fielder.rig().localToModel(fielderPose_, fielderModels_)) return;
    fielderVisual_.update(world_, fielder.rig(), fielderModels_, fielder.interpolatedRoot(alpha));
    if (feeder_.active()) {
        ballPos_ = feeder_.position(alpha);
        math::Transform ball;
        ball.position = ballPos_;
        world_.setTransform(scene_.ball, ball);
    } else if (fielder.holding()) {
        // The ball rides in the hand; a later drag starts from here.
        ballPos_ = fielder.ballInHand(fielderModels_, fielder.interpolatedRoot(alpha));
        math::Transform ball;
        ball.position = ballPos_;
        world_.setTransform(scene_.ball, ball);
    }
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

    if (debugSettings_.showProbe) {
        math::Transform probe = sim_.probeTransform(alpha);
        dd.axes(probe, 0.6f);
        dd.sphere(probe.position, 0.4f, render::colors::kCyan);
    }
}

} // namespace game
