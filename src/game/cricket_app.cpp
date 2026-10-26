#include "game/cricket_app.h"

#include "core/log.h"
#include "debug/axis_gizmo.h"
#include "platform/imgui_platform.h"

#include <imgui.h>

#include <chrono>
#include <random>
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

motion::ThrowSettings readThrowSettings(const core::Config& c) {
    motion::ThrowSettings s;
    s.radiusFactor = c.getFloat("arc.radiusFactor", s.radiusFactor);
    s.windupTime = c.getFloat("swing.windupTime", s.windupTime);
    s.swingTime = c.getFloat("swing.swingTime", s.swingTime);
    s.cockReach = c.getFloat("swing.cockReach", s.cockReach);
    s.throwSpeed = c.getFloat("release.speed", s.throwSpeed);
    return s;
}

motion::BowlSettings readBowlSettings(const core::Config& c) {
    motion::BowlSettings s;
    s.elbowBend = math::radians(c.getFloat("arm.elbowBendDeg", 10.0f));
    s.startAngle = math::radians(c.getFloat("circle.startDeg", 135.0f));
    s.setupTime = c.getFloat("circle.setupTime", s.setupTime);
    s.downBackTime = c.getFloat("circle.downBackTime", s.downBackTime);
    s.overTime = c.getFloat("circle.overTime", s.overTime);
    s.followTime = c.getFloat("circle.followTime", s.followTime);
    s.followAcross = c.getFloat("circle.followAcross", s.followAcross);
    s.speed = c.getFloat("release.speed", s.speed);
    return s;
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

    if (!sim_.initPhysics(cfg_.physicsWorld, cfg_.scene)) return false;
    sim_.initBall(cfg_.ball, cfg_.scene);
    scene_ = buildTestScene(world_, *renderer_, cfg_.scene, sim_.wickets().layout());
    ballPos_ = cfg_.scene.getVec3("ball.restPosition", math::Vec3(0.15f, 0.0f, 5.0f));
    ballPos_.y = cfg_.scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi;

    rig::SkeletonDef skeletonDef;
    if (!rig::loadSkeletonDef(cfg_.fielderSkeleton, skeletonDef) || !sim_.initFielder(skeletonDef, cfg_.fielderJoints, cfg_.locomotion)) return false;
    sim_.fielder().setThrowSettings(readThrowSettings(cfg_.throwing));
    const agents::Fielder& fielder = sim_.fielder();
    fielderVisual_.build(world_, *renderer_, fielder.def(), fielder.rig(), cfg_.fielderSkeleton);
    fielderModels_.assign(static_cast<size_t>(fielder.rig().numJoints()), math::Mat4(1.0f));
    if (!sim_.initBowler(skeletonDef, cfg_.fielderJoints, cfg_.locomotion,
                         cfg_.bowling.getVec3("bowler.position", math::Vec3(0.4f, 0.0f, -9.4f)),
                         math::radians(cfg_.bowling.getFloat("bowler.yawDeg", 0.0f))))
        return false;
    sim_.bowler().setBowlSettings(readBowlSettings(cfg_.bowling));
    const agents::Fielder& bowler = sim_.bowler();
    bowlerVisual_.build(world_, *renderer_, bowler.def(), bowler.rig(), cfg_.fielderSkeleton);
    bowlerModels_.assign(static_cast<size_t>(bowler.rig().numJoints()), math::Mat4(1.0f));
    limbTest_ = e.getBool("debug.limbTest", false);
    if (limbTest_) bowlerVisual_.setVisible(*renderer_, world_, false);
    // One ball: the bowler starts with it; the fielder's hand is empty (T
    // sends the ball to the fielder to pick).
    sim_.fielder().setReachTarget(ballPos_);
    bowlRelease_ = math::radians(cfg_.bowling.getFloat("aim.releaseDeg", 15.0f));

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
            sim_.initBall(cfg_.ball, cfg_.scene);
            destroyTestScene(world_, *renderer_, scene_);
            sim_.rebuildWickets(cfg_.physicsWorld, cfg_.scene);
            scene_ = buildTestScene(world_, *renderer_, cfg_.scene, sim_.wickets().layout());
            appliedSettings_.showProbe = !debugSettings_.showProbe;  // re-apply probe visibility
        } else if (name == "physics_world.json") {
            cfg_.physicsWorld = std::move(fresh);
            if (!sim_.initPhysics(cfg_.physicsWorld, cfg_.scene)) LOG_ERROR("reload: physics world could not be rebuilt");
            sim_.initBall(cfg_.ball, cfg_.scene);
        } else if (name == "ball.json") {
            cfg_.ball = std::move(fresh);
            sim_.initBall(cfg_.ball, cfg_.scene);
        } else if (name == "ball_launch.json") {
            cfg_.ballLaunch = std::move(fresh);
        } else if (name == "throw.json") {
            cfg_.throwing = std::move(fresh);
            sim_.fielder().setThrowSettings(readThrowSettings(cfg_.throwing));
        } else if (name == "bowling.json") {
            cfg_.bowling = std::move(fresh);  // the bowler's place applies on restart
            sim_.bowler().setBowlSettings(readBowlSettings(cfg_.bowling));
        } else if (name == "fielder_skeleton.json") {
            cfg_.fielderSkeleton = std::move(fresh);
            LOG_INFO("reload: fielder_skeleton.json changed; restart to rebuild the fielder");
            continue;
        } else if (name == "fielder_joints.json") {
            cfg_.fielderJoints = std::move(fresh);
            agents::Fielder& f = sim_.fielder();
            f.init(f.def(), f.rig().height(), cfg_.fielderJoints, cfg_.locomotion);  // limits + IK settings
            agents::Fielder& b = sim_.bowler();
            b.init(b.def(), b.rig().height(), cfg_.fielderJoints, cfg_.locomotion);
        } else if (name == "locomotion.json") {
            cfg_.locomotion = std::move(fresh);
            sim_.fielder().loadLocomotion(cfg_.locomotion);
        } else {
            continue;  // not a file this app uses
        }
        LOG_INFO("reload: applied {}", name);
    }
}

void CricketApp::launchBall(int index) {
    const nlohmann::json& presets = cfg_.ballLaunch.root().value("presets", nlohmann::json::array());
    if (!presets.is_array() || index >= static_cast<int>(presets.size())) {
        LOG_WARN("ball launch: no preset {} in ball_launch.json", index + 1);
        return;
    }
    const nlohmann::json& p = presets[static_cast<size_t>(index)];
    const auto pos = p.value("position", std::vector<float>{0.0f, 1.0f, 0.0f});
    const float speed = p.value("speedKmh", 100.0f) / 3.6f;
    const float elevation = math::radians(p.value("elevationDeg", 0.0f));
    const float azimuth = math::radians(p.value("azimuthDeg", 0.0f));
    const math::Vec3 dir(std::sin(azimuth) * std::cos(elevation), std::sin(elevation), std::cos(azimuth) * std::cos(elevation));
    const math::Vec3 from = pos.size() == 3 ? math::Vec3(pos[0], pos[1], pos[2]) : math::Vec3(0, 1, 0);
    sim_.fielder().clearReach();
    sim_.fielder().setReachTarget(from);  // lets go of a held ball
    sim_.launchBall(from, dir * speed);
    LOG_INFO("ball launch: {} at {:.0f} km/h", p.value("name", std::string("preset")), speed * 3.6f);
}

void CricketApp::releaseToFielder() {
    const core::Config& c = cfg_.ballLaunch;
    const math::Vec3 from = c.getVec3("toFielder.position", math::Vec3(0.0f, 0.8f, 9.0f));
    auto range = [&](const char* key, float lo, float hi) {
        const nlohmann::json* r = nullptr;
        if (auto t = c.root().find("toFielder"); t != c.root().end())
            if (auto it = t->find(key); it != t->end() && it->is_array() && it->size() == 2) r = &*it;
        if (r) lo = (*r)[0].get<float>(), hi = (*r)[1].get<float>();
        return std::uniform_real_distribution<float>(lo, std::max(lo, hi))(rng_);
    };
    const float elevation = math::radians(range("elevationDeg", 10.0f, 55.0f));
    const float landShare = range("landFraction", 0.2f, 1.6f);

    // Along the ground line to the fielder: the speed (no drag) puts the first
    // landing at landShare of the way, so the ball bounces before them (< 1),
    // reaches them in the air (~1) or goes over them (> 1).
    const math::Vec3 target = sim_.fielder().root().position;
    math::Vec3 flat(target.x - from.x, 0.0f, target.z - from.z);
    const float dist = math::length(flat);
    flat = math::safeNormalize(flat, -math::kForward);
    const float g = std::fabs(sim_.physicsWorld().world().gravity.y);
    const float speed = std::sqrt(g * landShare * dist / std::max(0.1f, std::sin(2.0f * elevation)));
    sim_.fielder().clearReach();
    sim_.fielder().setReachTarget(from);  // lets go of a held ball
    sim_.launchBall(from, (flat * std::cos(elevation) + math::kUp * std::sin(elevation)) * speed);
    LOG_INFO("ball released at the fielder: {:.0f} km/h, {:.0f} deg, {:.1f} m away", speed * 3.6f,
             math::degrees(elevation), dist);
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
    math::Vec2 move(axis(core::Action::StepLeft, core::Action::StepRight),
                    axis(core::Action::StepForward, core::Action::StepBack));
    if (limbTest_) move = math::Vec2(0.0f);
    // B switches the controlled player (fielder or bowler); the other stands.
    if (!limbTest_ && actions_.pressed(input_, core::Action::BowlAim)) {
        bowlAim_ = !bowlAim_;
        bowlTilting_ = false;
    }
    agents::Fielder& controlled = bowlAim_ ? sim_.bowler() : sim_.fielder();
    (bowlAim_ ? sim_.fielder() : sim_.bowler()).setMoveInput(math::Vec2(0.0f), false, 0.0f);
    if (controlled.speed() < kStoppedSpeed) heading_ = std::atan2(ahead.x, ahead.z);
    controlled.setMoveInput(move, !limbTest_ && actions_.down(input_, core::Action::StepForward), heading_);

    // Pace: walk by default, hold Shift to run, Ctrl to jog (Shift wins).
    controlled.setGait(actions_.down(input_, core::Action::Run)   ? motion::Gait::Run
                       : actions_.down(input_, core::Action::Jog) ? motion::Gait::Jog
                                                                  : motion::Gait::Walk);

    if (actions_.pressed(input_, core::Action::ClearReach)) sim_.fielder().clearReach();
    if (actions_.pressed(input_, core::Action::Pick)) sim_.fielder().startPick();
    for (int i = 0; i < 5; ++i)
        if (actions_.pressed(input_, static_cast<core::Action>(static_cast<int>(core::Action::LaunchBall1) + i))) launchBall(i);
    if (actions_.pressed(input_, core::Action::ThrowBall)) releaseToFielder();
    // Throw aim with the ball held: a left click fixes the slot point on the
    // slot arc; then the wheel turns the swing arc about it (the camera does
    // not zoom meanwhile) and a second click throws, releasing at the point
    // picked on the swing arc. R goes back to choosing the slot.
    // With the bowler controlled, the mouse aims the bowling circle; a click
    // on it hands the wheel to the circle's lean (until R or B), so the
    // camera can zoom in first. The fielder's throw aim waits meanwhile.
    if (bowlAim_ && actions_.pressed(input_, core::Action::ClearReach)) bowlTilting_ = false;
    // A second click bowls: the arm goes round and lets the ball go at the
    // release point.
    if (bowlAim_ && !input_.uiWantsMouse && input_.pressed(core::MouseButton::Left)) {
        if (bowlTilting_) sim_.bowler().startBowl(bowlTilt_, bowlRelease_);
        bowlTilting_ = true;
    }
    if (bowlTilting_ && !input_.uiWantsMouse) {
        bowlTilt_ = math::clamp(bowlTilt_ + input_.wheel * math::radians(cfg_.bowling.getFloat("aim.tiltStepDeg", 5.0f)),
                                math::radians(cfg_.bowling.getFloat("aim.minTiltDeg", -15.0f)),
                                math::radians(cfg_.bowling.getFloat("aim.maxTiltDeg", 90.0f)));
        input_.wheel = 0.0f;
    }
    agents::Fielder& fielder = sim_.fielder();
    if (!fielder.holding() || actions_.pressed(input_, core::Action::ClearReach)) aimSwing_ = false;
    if (aimSwing_ && !bowlAim_ && !input_.uiWantsMouse) {
        tilt_ += input_.wheel * math::radians(cfg_.throwing.getFloat("aim.tiltStepDeg", 5.0f));
        input_.wheel = 0.0f;
    }
    if (input_.pressed(core::MouseButton::Left) && !input_.uiWantsMouse && !limbTest_ && !bowlAim_ && fielder.holding() &&
        !fielder.throwing()) {
        if (!aimSwing_) {
            aimSwing_ = true;
            tilt_ = 0.0f;
        } else {
            fielder.startThrow(slotAngle_, tilt_, releaseAngle_);
            aimSwing_ = false;
        }
    }

    // Live config: saved files are re-applied to the running game; F5 forces all.
    changedConfigs_.clear();
    if (actions_.pressed(input_, core::Action::ReloadConfig))
        changedConfigs_ = {"engine.json", "input.json", "scene.json", "fielder_skeleton.json", "locomotion.json",
                          "physics_world.json", "ball.json", "ball_launch.json", "throw.json",
                          "bowling.json"};
    else if (cfg_.liveReload)
        configWatcher_.poll(changedConfigs_);
    if (!changedConfigs_.empty()) reloadConfig(changedConfigs_);
    return running;
}

void CricketApp::updatePose(float seconds) {
    agents::Fielder& fielder = sim_.fielder();
    if (actions_.pressed(input_, core::Action::ClearReach)) {
        fielder.clearPose();
        poseActive_ = false;
    }
    if (actions_.pressed(input_, core::Action::PoseLimb)) {
        poseLimb_ = (poseLimb_ + 1) % 4;
        poseActive_ = false;
    }
    if (input_.uiWantsKeyboard) return;
    auto axis = [&](core::Action pos, core::Action neg) {
        return (actions_.down(input_, pos) ? 1.0f : 0.0f) - (actions_.down(input_, neg) ? 1.0f : 0.0f);
    };
    const float side = axis(core::Action::PoseRight, core::Action::PoseLeft);
    const float ahead = axis(core::Action::PoseAway, core::Action::PoseNear);
    if (side == 0.0f && ahead == 0.0f) return;

    // The target starts where the limb is; the arrows move it relative to the
    // camera on the ground plane, or up/down with Shift.
    const bool hand = poseLimb_ >= 2;
    const bool left = poseLimb_ % 2 == 1;
    if (!poseActive_) {
        poseTarget_ = hand ? fielder.wrist(left, fielderModels_, fielder.root())
                           : (left ? fielder.targets().footL : fielder.targets().footR);
        poseActive_ = true;
    }
    const camera::CameraPose view = camera_.pose();
    const math::Vec3 forward =
        math::safeNormalize(math::Vec3(view.target.x - view.eye.x, 0.0f, view.target.z - view.eye.z), -math::kForward);
    const math::Vec3 right = math::cross(forward, math::kUp);
    const math::Vec3 move = actions_.down(input_, core::Action::Run) ? math::kUp * ahead + right * side
                                                                    : forward * ahead + right * side;
    poseTarget_ += move * (cfg_.engine.getFloat("debug.limbTestSpeed", 0.5f) * seconds);
    poseTarget_.y = std::max(poseTarget_.y, 0.0f);
    if (hand) fielder.setPoseHand(left, poseTarget_);
    else fielder.setPoseFoot(left ? motion::Foot::Left : motion::Foot::Right, poseTarget_);
}

void CricketApp::fixedUpdate(double dt) {
    // A moving ball is the fielder's target every tick until it is picked
    // (then it rides in the hand).
    if (sim_.ballInFlight() && sim_.fielder().holding()) sim_.stopBall();
    if (sim_.ballInFlight()) sim_.fielder().setReachTarget(sim_.ball().position, sim_.ball().velocity);
    sim_.tick(dt);
    math::Vec3 position, velocity;
    if (sim_.fielder().takeRelease(position, velocity)) sim_.launchBall(position, velocity);
    if (sim_.bowler().takeRelease(position, velocity)) {
        sim_.launchBall(position, velocity);
        ballBackTimer_ = cfg_.bowling.getFloat("release.returnDelay", 3.0f);
    }
    // The bowled ball goes back to the bowler after a delay, unless the
    // fielder has it.
    if (ballBackTimer_ >= 0.0f && (ballBackTimer_ -= static_cast<float>(dt)) < 0.0f && !sim_.fielder().holding()) {
        sim_.stopBall();
        sim_.bowler().giveBall();
    }
}

void CricketApp::updateWickets(float alpha) {
    const match::Wickets& wickets = sim_.wickets();
    if (!wickets.built()) return;
    const physics::PhysicsWorld& physics = sim_.physicsWorld();
    for (int i = 0; i < match::kWicketPieces; ++i) {
        entt::entity e = scene_.wicketPieces[static_cast<size_t>(i)];
        if (e == entt::null) continue;
        world_.setTransform(e, wicketPieceVisual(wickets.layout(), i, physics.interpolated(wickets.body(i), alpha)));
    }
}

void CricketApp::renderFrame(double frameSeconds, float alpha) {
    auto frameStart = std::chrono::steady_clock::now();

    // --- Cameras (render-side, real frame time); the orbit follows the controlled player ---
    camera_.setFollowPoint((bowlAim_ ? sim_.bowler() : sim_.fielder()).interpolatedRoot(alpha).position);
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
    updateThrowArc(alpha);
    updateBowlArc(alpha);
    updateWickets(alpha);
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
    const agents::Fielder& fielder = sim_.fielder();
    fielder.interpolatedPose(alpha, fielderPose_);
    if (!fielder.rig().localToModel(fielderPose_, fielderModels_)) return;
    fielderVisual_.update(world_, fielder.rig(), fielderModels_, fielder.interpolatedRoot(alpha));
    if (sim_.ballInFlight()) {
        const math::Transform ball = sim_.ballTransform(alpha);  // rolls and spins
        ballPos_ = ball.position;
        world_.setTransform(scene_.ball, ball);
    } else if (fielder.holding()) {
        // The ball rides in the hand; a later drag starts from here.
        ballPos_ = fielder.ballInHand(fielderModels_, fielder.interpolatedRoot(alpha));
        math::Transform ball;
        ball.position = ballPos_;
        world_.setTransform(scene_.ball, ball);
    }
    const agents::Fielder& bowler = sim_.bowler();
    bowler.interpolatedPose(alpha, bowlerPose_);
    if (!bowler.rig().localToModel(bowlerPose_, bowlerModels_)) return;
    bowlerVisual_.update(world_, bowler.rig(), bowlerModels_, bowler.interpolatedRoot(alpha));
    if (!limbTest_ && !sim_.ballInFlight() && bowler.holding() && !fielder.holding()) {
        ballPos_ = bowler.ballInHand(bowlerModels_, bowler.interpolatedRoot(alpha));
        math::Transform ball;
        ball.position = ballPos_;
        world_.setTransform(scene_.ball, ball);
    }
}

void CricketApp::updateThrowArc(float alpha) {
    const agents::Fielder& fielder = sim_.fielder();
    if (limbTest_ || (!fielder.holding() && !fielder.throwing())) return;
    const math::Transform root = fielder.interpolatedRoot(alpha);
    slotArc_ = motion::makeThrowArc(fielder.holdingShoulder(fielderModels_, root), root.rotation, fielder.holdingLeft(),
                                    cfg_.throwing.getFloat("arc.radiusFactor", 0.95f) * fielder.armLength());
    swingArc_ = motion::makeSwingArc(slotArc_, slotAngle_, tilt_);
    if (input_.uiWantsMouse || bowlAim_ || fielder.throwing()) return;  // the points stay put during a throw

    // Mouse ray onto the arc being aimed; both are half circles (-90..+90 deg).
    math::Vec3 eye, dir;
    mouseRay(eye, dir);
    float angle = 0.0f;
    if (!(aimSwing_ ? swingArc_ : slotArc_).rayAngle(eye, dir, angle)) return;
    angle = math::clamp(angle, -math::kHalfPi, math::kHalfPi);
    if (aimSwing_) releaseAngle_ = angle;
    else slotAngle_ = angle;
}

void CricketApp::updateBowlArc(float alpha) {
    const agents::Fielder& bowler = sim_.bowler();
    const math::Transform root = bowler.interpolatedRoot(alpha);
    bowlArc_ = motion::makeBowlingArc(bowler.holdingShoulder(bowlerModels_, root), root.rotation, bowler.holdingLeft(),
                                      bowler.bowlRadius(), bowlTilt_);
    if (!bowlAim_ || input_.uiWantsMouse || bowler.bowling()) return;  // the point stays put while bowling
    math::Vec3 eye, dir;
    mouseRay(eye, dir);
    float angle = 0.0f;
    if (bowlArc_.rayAngle(eye, dir, angle)) bowlRelease_ = angle;
}

void CricketApp::mouseRay(math::Vec3& origin, math::Vec3& direction) const {
    const camera::CameraPose pose = camera_.pose();
    const float w = static_cast<float>(std::max(1, window_.width()));
    const float h = static_cast<float>(std::max(1, window_.height()));
    const float x = mouseLook_ ? 0.0f : 2.0f * input_.mouseX / w - 1.0f;
    const float y = mouseLook_ ? 0.0f : 1.0f - 2.0f * input_.mouseY / h;
    const float tanHalf = std::tan(0.5f * math::radians(cfg_.engine.getFloat("camera.fovY", 50.0f)));
    const math::Vec3 ahead = math::safeNormalize(pose.target - pose.eye, -math::kForward);
    const math::Vec3 right = math::safeNormalize(math::cross(ahead, pose.up), math::kRight);
    const math::Vec3 up = math::cross(right, ahead);
    origin = pose.eye;
    direction = math::normalize(ahead + right * (x * tanHalf * w / h) + up * (y * tanHalf));
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
    // limb before that); the throw and bowling arcs are hidden.
    const agents::Fielder& fielder = sim_.fielder();
    if (limbTest_) {
        const bool hand = poseLimb_ >= 2;
        const bool left = poseLimb_ % 2 == 1;
        const math::Vec3 at = poseActive_ ? poseTarget_
                              : hand        ? fielder.wrist(left, fielderModels_, fielder.interpolatedRoot(alpha))
                              : left        ? fielder.targets().footL
                                            : fielder.targets().footR;
        dd.sphere(at, 0.05f, poseActive_ ? render::colors::kRed : render::colors::kYellow);
    } else {
        // Throw arcs (half circles) while the ball is held or thrown: the slot
        // arc (cyan) with the slot point (magenta); once the slot is fixed, the
        // swing arc (yellow) with its start (white) and the release point (red).
        if (fielder.holding() || fielder.throwing()) {
            constexpr int kSegments = 24;
            auto halfCircle = [&](const motion::ThrowArc& arc, render::Color color) {
                for (int i = 0; i < kSegments; ++i) {
                    const float a = -math::kHalfPi + math::kPi * static_cast<float>(i) / kSegments;
                    dd.line(arc.point(a), arc.point(a + math::kPi / kSegments), color);
                }
            };
            halfCircle(slotArc_, render::colors::kCyan);
            dd.sphere(slotArc_.point(slotAngle_), 0.04f, render::colors::kMagenta);
            if (aimSwing_ || fielder.throwing()) {
                halfCircle(swingArc_, render::colors::kYellow);
                dd.sphere(swingArc_.point(-math::kHalfPi), 0.03f, render::colors::kWhite);
                dd.sphere(swingArc_.point(releaseAngle_), 0.04f, render::colors::kRed);
            }
        }

        // Bowling circle (full): yellow while the mouse aims the bowler, grey
        // otherwise; the top (white) and the release point (red).
        constexpr int kCircleSegments = 48;
        const render::Color circleColor = bowlAim_ ? render::colors::kYellow : render::colors::kGrey;
        for (int i = 0; i < kCircleSegments; ++i) {
            const float a = math::kTwoPi * static_cast<float>(i) / kCircleSegments;
            dd.line(bowlArc_.point(a), bowlArc_.point(a + math::kTwoPi / kCircleSegments), circleColor);
        }
        dd.sphere(bowlArc_.point(0.0f), 0.03f, render::colors::kWhite);
        dd.sphere(bowlArc_.point(bowlRelease_), 0.04f, render::colors::kRed);
    }

    if (debugSettings_.showProbe) {
        math::Transform probe = sim_.probeTransform(alpha);
        dd.axes(probe, 0.6f);
        dd.sphere(probe.position, 0.4f, render::colors::kCyan);
    }
}

} // namespace game
