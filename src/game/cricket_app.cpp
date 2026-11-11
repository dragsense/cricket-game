#include "game/cricket_app.h"

#include "core/log.h"
#include "debug/axis_gizmo.h"
#include "platform/imgui_platform.h"

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <cmath>

namespace game {

namespace {

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
    s.slotRadiusFactor = c.getFloat("slot.radiusFactor", s.slotRadiusFactor);
    s.swingRadiusFactor = c.getFloat("chest.radiusFactor", s.swingRadiusFactor);
    s.maxBack = math::radians(c.getFloat("chest.maxBackDeg", math::degrees(s.maxBack)));
    s.turnShare = c.getFloat("chest.turnShare", s.turnShare);
    s.pelvisShare = c.getFloat("chest.pelvisShare", s.pelvisShare);
    s.untwistRate = c.getFloat("chest.untwistRate", s.untwistRate);
    s.push = c.getFloat("timing.push", s.push);
    s.blendShare = c.getFloat("timing.blendShare", s.blendShare);
    s.readyTime = c.getFloat("timing.readyTime", s.readyTime);
    s.windTime = c.getFloat("timing.windTime", s.windTime);
    s.swingTime = c.getFloat("timing.swingTime", s.swingTime);
    s.followAngle = math::radians(c.getFloat("timing.followDeg", math::degrees(s.followAngle)));
    s.followTime = c.getFloat("timing.followTime", s.followTime);
    s.strideForce = c.getFloat("stride.minForce", s.strideForce);
    s.minStride = c.getFloat("stride.minLength", s.minStride);
    s.maxStride = c.getFloat("stride.maxLength", s.maxStride);
    s.pauseAtRelease = c.getBool("test.pauseAtRelease", s.pauseAtRelease);
    s.minSpeed = c.getFloat("release.minSpeed", s.minSpeed);
    s.maxSpeed = c.getFloat("release.maxSpeed", s.maxSpeed);
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
    s.coilTime = c.getFloat("action.time", s.coilTime);
    s.pullTime = c.getFloat("action.pullTime", s.pullTime);
    s.frontDelay = c.getFloat("action.frontDelay", s.frontDelay);
    s.frontGap = c.getVec3("action.frontGap", s.frontGap);
    s.elbowUpTurn = math::radians(c.getFloat("action.elbowUpTurnDeg", math::degrees(s.elbowUpTurn)));    s.turnDone = math::radians(c.getFloat("swing.turnDoneDeg", math::degrees(s.turnDone)));
    s.backTime = c.getFloat("back.time", s.backTime);
    s.coilAhead = c.getFloat("coil.aheadStart", s.coilAhead);
    s.chestBase = math::radians(c.getFloat("coil.chestDeg", math::degrees(s.chestBase)));
    s.bodyBase = math::radians(c.getFloat("swing.bodyDeg", math::degrees(s.bodyBase)));
    s.maxCoil = math::radians(c.getFloat("coil.maxDeg", math::degrees(s.maxCoil)));
    s.leanSign = c.getFloat("swing.leanSign", s.leanSign);
    s.strideLength = c.getFloat("swing.strideLength", s.strideLength);
    s.strideAt = c.getFloat("swing.strideAt", s.strideAt);
    s.swingTime = c.getFloat("swing.time", s.swingTime);
    s.releaseLean = c.getFloat("swing.lean", s.releaseLean);
    s.frontClear = c.getFloat("frontArm.clear", s.frontClear);
    s.minSpeed = c.getFloat("swing.minSpeed", s.minSpeed);
    s.maxSpeed = c.getFloat("swing.maxSpeed", s.maxSpeed);
    s.followAngle = math::radians(c.getFloat("follow.angleDeg", math::degrees(s.followAngle)));
    s.swingFollowTime = c.getFloat("follow.time", s.swingFollowTime);
    s.followReach = c.getFloat("follow.reach", s.followReach);
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
    ballPos_ = cfg_.scene.getVec3("ball.restPosition", math::Vec3(0.15f, 0.0f, 5.0f));
    ballPos_.y = cfg_.scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi;

    rig::SkeletonDef skeletonDef;
    if (!rig::loadSkeletonDef(cfg_.humanSkeleton, skeletonDef) || !sim_.initFielder(skeletonDef, cfg_.humanJoints, cfg_.locomotion)) return false;
    sim_.fielder().setThrowSettings(readThrowSettings(cfg_.throwing));
    const agents::Human& fielder = sim_.fielder();
    humanVisual_.build(world_, *renderer_, fielder.def(), fielder.rig(), cfg_.humanSkeleton);
    fielderModels_.assign(static_cast<size_t>(fielder.rig().numJoints()), math::Mat4(1.0f));
    if (!sim_.initBowler(skeletonDef, cfg_.humanJoints, cfg_.locomotion,
                         cfg_.bowling.getVec3("bowler.position", math::Vec3(0.4f, 0.0f, -9.4f)),
                         math::radians(cfg_.bowling.getFloat("bowler.yawDeg", 0.0f))))
        return false;
    applyBowlSettings();
    const agents::Human& bowler = sim_.bowler();
    bowlerVisual_.build(world_, *renderer_, bowler.def(), bowler.rig(), cfg_.humanSkeleton);
    bowlerModels_.assign(static_cast<size_t>(bowler.rig().numJoints()), math::Mat4(1.0f));
    limbTest_ = e.getBool("debug.limbTest", false);
    if (limbTest_) bowlerVisual_.setVisible(*renderer_, world_, false);
    // One ball: the bowler starts with it; the fielder's hand is empty (T
    // sends the ball to the fielder to pick).
    sim_.fielder().setReachTarget(ballPos_);
    bowlRelease_ = math::radians(cfg_.bowling.getFloat("aim.releaseDeg", 15.0f));
    actionGap_ = cfg_.bowling.getFloat("action.gap", 0.1f);


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
        } else if (name == "ball_launch.json") {
            cfg_.ballLaunch = std::move(fresh);
        } else if (name == "throw.json") {
            cfg_.throwing = std::move(fresh);
            sim_.fielder().setThrowSettings(readThrowSettings(cfg_.throwing));
        } else if (name == "bowling.json") {
            cfg_.bowling = std::move(fresh);  // the bowler's place applies on restart
            applyBowlSettings();
        } else if (name == "human_skeleton.json") {
            cfg_.humanSkeleton = std::move(fresh);
            LOG_INFO("reload: human_skeleton.json changed; restart to rebuild the fielder");
            continue;
        } else if (name == "human_joints.json") {
            cfg_.humanJoints = std::move(fresh);
            agents::Human& f = sim_.fielder();
            f.init(f.def(), f.rig().height(), cfg_.humanJoints, cfg_.locomotion);  // limits + IK settings
            agents::Human& b = sim_.bowler();
            b.init(b.def(), b.rig().height(), cfg_.humanJoints, cfg_.locomotion);
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
    // B switches the controlled player (fielder or bowler); the other stands.
    if (!limbTest_ && actions_.pressed(input_, core::Action::BowlAim)) {
        bowlAim_ = !bowlAim_;
        sim_.bowler().cancelCoil();
        bowlStep_ = 0;
    }
    // I: slow motion (the whole game) on/off. U: a random action.
    if (actions_.pressed(input_, core::Action::SlowMotion)) {
        slowMotion_ = !slowMotion_;
        setTimeScale(slowMotion_ ? cfg_.bowling.getFloat("slowMotion.factor", 0.2f) : 1.0);
        LOG_INFO("Bowling slow motion {}", slowMotion_ ? "on" : "off");
    }
    if (!limbTest_ && actions_.pressed(input_, core::Action::RandomBowl)) randomBowl();
    agents::Human& controlled = bowlAim_ ? sim_.bowler() : sim_.fielder();
    (bowlAim_ ? sim_.fielder() : sim_.bowler()).setFacing(false, 0.0f);
    if (actions_.pressed(input_, core::Action::FreeView)) freeView_ = !freeView_;
    if (!freeView_) heading_ = std::atan2(ahead.x, ahead.z);
    // One way to move (the user's call, 2026-11-09): WASD takes steps (held:
    // walking); the body follows the feet. The heading only turns the player.
    controlled.setFacing(true, heading_);
    controlled.setStepInput(move);
    if (actions_.pressed(input_, core::Action::WalkFaster)) controlled.changeWalkSpeed(1);
    if (actions_.pressed(input_, core::Action::WalkSlower)) controlled.changeWalkSpeed(-1);

    // Limb test: no picking, balls or throwing (no ball reaches the hand).
    if (!limbTest_ && actions_.pressed(input_, core::Action::ClearReach)) sim_.fielder().clearReach();
    if (!limbTest_ && actions_.pressed(input_, core::Action::Pick))
        sim_.fielder().startPick(actions_.down(input_, core::Action::PickStep));
    for (int i = 0; i < 5 && !limbTest_; ++i)
        if (actions_.pressed(input_, static_cast<core::Action>(static_cast<int>(core::Action::LaunchBall1) + i))) launchBall(i);
    if (!limbTest_ && actions_.pressed(input_, core::Action::ThrowBall)) releaseToFielder();
    if (!limbTest_ && actions_.pressed(input_, core::Action::GiveBall) && !sim_.fielder().holding() &&
        !sim_.fielder().throwing()) {
        sim_.stopBall();
        sim_.fielder().giveBall();
        ballBackTimer_ = -1.0f;
    }
    // Throw aim with the ball held: a left click fixes the slot point on the
    // slot arc; then the wheel turns the swing arc about it (the camera does
    // not zoom meanwhile) and a second click throws, releasing at the point
    // picked on the swing arc. R goes back to choosing the slot.
    // With the bowler controlled, the mouse aims the bowling circle; a click
    // on it hands the wheel to the circle's lean (until R or B), so the
    // camera can zoom in first. The fielder's throw aim waits meanwhile.
    // Until the action click, the wheel turns the action circle about the
    // vertical, Ctrl + wheel sizes it (at most the arm length) and Shift +
    // wheel sets the gap between
    // the action point and the hand's way past it; after it, the wheel leans
    // the release circle. The camera does not zoom meanwhile.
    if (bowlAim_ && bowlStep_ == 0 && sim_.bowler().holding() && !input_.uiWantsMouse && !autoBowl_) {
        if (actions_.down(input_, core::Action::ModShift))
            actionGap_ = math::clamp(actionGap_ + input_.wheel * cfg_.bowling.getFloat("action.gapStep", 0.02f), 0.0f,
                                     cfg_.bowling.getFloat("action.maxGap", 0.3f));
        else if (actions_.down(input_, core::Action::ModCtrl))
            actionSize_ = math::clamp(actionSize_ + input_.wheel * cfg_.bowling.getFloat("action.sizeStep", 0.05f),
                                      cfg_.bowling.getFloat("action.minSize", 0.3f), 1.0f);
        else
            actionTurn_ = math::clamp(actionTurn_ + input_.wheel * math::radians(cfg_.bowling.getFloat("aim.tiltStepDeg", 5.0f)),
                                      -math::kPi, math::kPi);
        input_.wheel = 0.0f;
    }
    if (bowlAim_ && bowlStep_ == 2 && sim_.bowler().holding() && !input_.uiWantsMouse && !autoBowl_) {
        const float step = input_.wheel * math::radians(cfg_.bowling.getFloat("aim.tiltStepDeg", 5.0f));
        releaseTilt_ = math::clamp(releaseTilt_ + step, 0.0f,
                                       math::radians(cfg_.bowling.getFloat("aim.maxTiltDeg", 90.0f)));
        input_.wheel = 0.0f;
    }
    // Throw: once the hand is at the slot point, the wheel leans the chest
    // circle, before the force click and after it (the wound-up chest turns
    // with it); here, before the camera would zoom with it.
    if (throwStep_ >= 1 && sim_.fielder().throwing() && !bowlAim_ && !input_.uiWantsMouse) {
        const float maxTilt = math::radians(cfg_.throwing.getFloat("aim.maxTiltDeg", 30.0f));
        tilt_ = math::clamp(tilt_ + input_.wheel * math::radians(cfg_.throwing.getFloat("aim.tiltStepDeg", 5.0f)), -maxTilt,
                            maxTilt);
        input_.wheel = 0.0f;
        sim_.fielder().tiltThrow(tilt_);
    }

    // Live config: saved files are re-applied to the running game; F5 forces all.
    changedConfigs_.clear();
    if (actions_.pressed(input_, core::Action::ReloadConfig))
        changedConfigs_ = {"engine.json", "input.json", "scene.json", "human_skeleton.json", "locomotion.json",
                          "physics_world.json", "ball_launch.json", "throw.json",
                          "bowling.json"};
    else if (cfg_.liveReload)
        configWatcher_.poll(changedConfigs_);
    if (!changedConfigs_.empty()) reloadConfig(changedConfigs_);
    return running;
}

void CricketApp::updatePose(float /*seconds: the target jumps per press, not per second*/) {
    agents::Human& fielder = sim_.fielder();
    if (actions_.pressed(input_, core::Action::ClearReach)) {
        // R: the targets go and the body is back at its standing pose.
        fielder.clearPose();
        fielder.resetPhysicsBody();
        poseActive_ = false;
    }
    if (actions_.pressed(input_, core::Action::PoseLimb)) {
        poseLimb_ = (poseLimb_ + 1) % 4;  // right foot, left foot, right hand, left hand
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

    // The target starts where the limb is; each arrow press jumps it one step
    // relative to the camera on the ground plane, or up/down with Shift.
    const bool hand = poseLimb_ >= 2;
    const bool left = poseLimb_ % 2 == 1;
    if (!poseActive_) {
        poseTarget_ = hand ? fielder.wrist(left, fielderModels_, fielder.root())
                           : fielder.ankle(left, fielderModels_, fielder.root());
        poseActive_ = true;
    }
    const camera::CameraPose view = camera_.pose();
    const math::Vec3 forward =
        math::safeNormalize(math::Vec3(view.target.x - view.eye.x, 0.0f, view.target.z - view.eye.z), -math::kForward);
    const math::Vec3 right = math::cross(forward, math::kUp);
    const math::Vec3 move = actions_.down(input_, core::Action::ModShift) ? math::kUp * ahead + right * side
                                                                    : forward * ahead + right * side;
    poseTarget_ += move * cfg_.engine.getFloat("debug.limbTestStep", 0.1f);
    // The target goes anywhere in the air, never under the ground (y = 0);
    // the limb reaches as near as the body allows.
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
    updateThrowAim(alpha);
    updateBowlArc(alpha);
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
    const agents::Human& bowler = sim_.bowler();
    bowler.interpolatedPose(alpha, bowlerPose_);
    if (!bowler.rig().localToModel(bowlerPose_, bowlerModels_)) return;
    bowler.physicalModels(sim_.physicsWorld(), alpha, bowlerModels_);
    bowlerVisual_.update(world_, bowler.rig(), bowlerModels_, bowler.interpolatedRoot(alpha));
    if (!limbTest_ && !sim_.ballInFlight() && bowler.holding() && !fielder.holding()) {
        ballPos_ = bowler.ballInHand(bowlerModels_, bowler.interpolatedRoot(alpha));
        math::Transform ball;
        ball.position = ballPos_;
        world_.setTransform(scene_.ball, ball);
    }
}

void CricketApp::updateThrowAim(float alpha) {
    // With the ball held, three clicks, each acting at once: on the height
    // circle (cyan) the hand goes to the slot point; a click on the throw
    // circle (yellow) turns the chest back to the force point (behind: how
    // far it winds back), the hand riding with it; the wheel leans the circle
    // before and after; a click on it is the release point, and the chest
    // throws. R lets go of the throw.
    agents::Human& fielder = sim_.fielder();
    if (bowlAim_ || (!fielder.holding() && !fielder.throwing())) {
        throwStep_ = 0;
        return;
    }
    if (!fielder.throwing()) throwStep_ = 0;
    const math::Transform root = fielder.interpolatedRoot(alpha);
    auto toWorld = [&](motion::ThrowArc arc) {
        arc.centre = root.transformPoint(arc.centre);
        arc.side = root.transformDirection(arc.side);
        arc.up = root.transformDirection(arc.up);
        arc.normal = root.transformDirection(arc.normal);
        return arc;
    };
    slotArc_ = toWorld(fielder.slotArc());
    chestArc_ = toWorld(fielder.chestArc(slotAngle_, tilt_));
    // Test (throw.json test.pauseAtRelease): a click lets the waiting ball go.
    if (fielder.throwPaused() && input_.pressed(core::MouseButton::Left) && !input_.uiWantsMouse) fielder.letGo();
    if (fielder.throwSwinging() || input_.uiWantsMouse) return;  // the points stay put during the swing

    if (actions_.pressed(input_, core::Action::ClearReach) && fielder.throwing()) {
        fielder.cancelThrow();
        throwStep_ = 0;
        return;
    }
    math::Vec3 eye, dir;
    mouseRay(eye, dir);
    float angle = 0.0f;
    if ((throwStep_ == 0 ? slotArc_ : chestArc_).rayAngle(eye, dir, angle)) {
        if (throwStep_ == 0) slotAngle_ = angle;
        // Anywhere on the full circle: the force point counts back from the
        // slot point (0 .. -360 deg), the release point forward from it.
        else if (throwStep_ == 1) backAngle_ = angle > 0.0f ? angle - math::kTwoPi : angle;
        else releaseAngle_ = angle > backAngle_ ? angle : angle + math::kTwoPi;
    }
    if (!input_.pressed(core::MouseButton::Left)) return;
    if (throwStep_ == 0) {
        fielder.beginThrow(slotAngle_);
        tilt_ = 0.0f;
    } else if (throwStep_ == 1) {
        fielder.windThrow(tilt_, backAngle_);
    } else {
        fielder.releaseThrow(releaseAngle_);
    }
    throwStep_ = (throwStep_ + 1) % 3;
}

void CricketApp::applyBowlSettings() {
    motion::BowlSettings s = readBowlSettings(cfg_.bowling);
    sim_.bowler().setBowlSettings(s);
}

void CricketApp::randomBowl() {
    // The bowler takes control (the camera follows him) and bowls standing
    // where he is: a random action point, gap, release circle lean and
    // release point within config/bowling.json "random". The hand goes to the
    // action point at once, then the way down (the front foot strides) and
    // the release follow on their own (updateBowlArc). No run-up (removed
    // 2026-11-09, the user's call).
    agents::Human& bowler = sim_.bowler();
    if (!bowler.holding() || bowler.bowling()) return;
    auto range = [&](const std::string& key, float lo, float hi) {
        const float a = cfg_.bowling.getFloat(key + ".min", lo), b = cfg_.bowling.getFloat(key + ".max", hi);
        return std::uniform_real_distribution<float>(a, std::max(a, b))(rng_);
    };
    bowlAim_ = true;
    actionTurn_ = math::radians(range("random.turnDeg", -30.0f, 60.0f));
    actionAngle_ = math::radians(range("random.actionDeg", 90.0f, 200.0f));
    actionSize_ = range("random.size", 0.8f, 1.0f);
    actionGap_ = range("random.gap", 0.0f, 0.2f);
    releaseTilt_ = math::radians(range("random.tiltDeg", 0.0f, 40.0f));
    bowlRelease_ = math::radians(range("random.releaseDeg", -3.0f, 3.0f));
    bowler.actionBowl(bowler.actionCircle(actionTurn_, actionSize_ * bowler.armLength()).point(actionAngle_), actionGap_,
                      actionTurn_);
    bowlStep_ = 1;
    autoRelease_ = true;  // down and release on their own once the hand holds
    autoBowl_ = true;
}

void CricketApp::updateBowlArc(float alpha) {
    // With the bowler controlled and the ball held: the action circle
    // (yellow, full, round the bowling shoulder, seen from the side; the
    // wheel turns it, Ctrl + wheel sizes it) with the action point
    // (white) the mouse picks on it and the hand's way there (cyan); a click
    // sends the hand there; once it holds, a second click sends it on down
    // to the bottom point (cyan). Once it is down, the release circle (red,
    // pivoting on the bottom point straight below the shoulder, leaned out
    // to the bowling-arm side by the wheel) and a third click on it sets the
    // release point and bowls the hand round it. R lets go.
    agents::Human& bowler = sim_.bowler();
    const math::Transform root = bowler.interpolatedRoot(alpha);
    auto toWorld = [&](motion::ThrowArc arc) {
        arc.centre = root.transformPoint(arc.centre);
        arc.side = root.transformDirection(arc.side);
        arc.up = root.transformDirection(arc.up);
        arc.normal = root.transformDirection(arc.normal);
        return arc;
    };
    // The release circle keeps the facing from before the bowling turn: it
    // does not turn with the body (its centre still rides the shoulder).
    releaseArc_ = toWorld(bowler.bowlArc(releaseTilt_));
    if (bowler.bowling()) {
        const math::Quat base = math::fromAxisAngle(math::kUp, bowler.bowlBaseYaw());
        const motion::ThrowArc local = bowler.bowlArc(releaseTilt_);
        releaseArc_.side = base * local.side;
        releaseArc_.up = base * local.up;
        releaseArc_.normal = base * local.normal;
    }
    const bool releaseReady = bowler.armDown() && bowler.bodyTurned();  // the red circle and its click
    actionArc_ = toWorld(bowler.actionCircle(actionTurn_, actionSize_ * bowler.armLength()));
    if (!bowler.bowling()) {
        bowlStep_ = 0;
        autoRelease_ = false;
        autoBowl_ = false;
    }
    if (autoRelease_ && bowler.actionHeld()) {  // U: the way down and the release follow on their own
        bowler.downBowl();
        bowlStep_ = 2;
    }
    if (autoRelease_ && releaseReady) {
        bowler.releaseBowl(bowlRelease_, releaseTilt_);
        bowlStep_ = 3;
        autoRelease_ = false;
    }
    if (!bowlAim_ || input_.uiWantsMouse || (bowler.bowling() && !bowler.coiled() && !bowler.armDown())) return;  // points stay put while bowling
    if (actions_.pressed(input_, core::Action::ClearReach)) {
        bowler.cancelCoil();
        bowlStep_ = 0;
        return;
    }
    // Click-by-click bowling is off for now (U only): a click only goes on
    // from a check stop (above), it never steps the delivery itself.
    if (bowlStep_ >= 0) return;
    math::Vec3 eye, dir;
    mouseRay(eye, dir);
    float angle = 0.0f;
    // U: the random points are not the mouse's; the clicks still step on.
    if (!autoBowl_ && bowlStep_ == 0 && actionArc_.rayAngle(eye, dir, angle)) {
        // Front half only (0 down, 90 ahead, 180 up): behind snaps to the nearer end.
        angle = std::remainder(angle, math::kTwoPi);
        actionAngle_ = angle >= 0.0f ? angle : (angle > -math::kHalfPi ? 0.0f : math::kPi);
    }
    const math::Vec3 actionPoint =
        bowler.actionCircle(actionTurn_, actionSize_ * bowler.armLength()).point(actionAngle_);  // character space
    if (!autoBowl_ && bowlStep_ == 2 && releaseArc_.rayAngle(eye, dir, angle)) bowlRelease_ = angle;
    if (!input_.pressed(core::MouseButton::Left)) return;
    if (bowlStep_ == 0 && bowler.holding()) {
        bowler.actionBowl(actionPoint, actionGap_, actionTurn_);
        bowlStep_ = 1;
    } else if (bowlStep_ == 1 && bowler.actionHeld()) {
        bowler.downBowl();
        bowlStep_ = 2;
    } else if (bowlStep_ == 2 && releaseReady) {
        bowler.releaseBowl(bowlRelease_, releaseTilt_);
        bowlStep_ = 3;
    }
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
    // limb before that); the bowling arc is hidden.
    const agents::Human& fielder = sim_.fielder();
    if (limbTest_) {
        const bool hand = poseLimb_ >= 2;
        const bool left = poseLimb_ % 2 == 1;
        const math::Vec3 at = poseActive_ ? poseTarget_
                              : hand        ? fielder.wrist(left, fielderModels_, fielder.interpolatedRoot(alpha))
                              : left        ? fielder.targets().footL
                                            : fielder.targets().footR;
        dd.sphere(at, 0.05f, poseActive_ ? render::colors::kRed : render::colors::kYellow);
    }
    {
        // Throw aim while the ball is held or thrown: the height circle (cyan)
        // around the chest with the slot point (magenta); then the throw
        // circle (yellow, around the shoulder) with the force point (white)
        // and the release point (red).
        if (fielder.holding() || fielder.throwing()) {
            constexpr int kSegments = 32;
            auto arcLine = [&](const motion::ThrowArc& arc, float from, float to, render::Color color) {
                for (int i = 0; i < kSegments; ++i) {
                    const float a = from + (to - from) * static_cast<float>(i) / kSegments;
                    dd.line(arc.point(a), arc.point(a + (to - from) / kSegments), color);
                }
            };
            arcLine(slotArc_, -math::kPi, math::kPi, render::colors::kCyan);
            dd.sphere(slotArc_.point(slotAngle_), 0.04f, render::colors::kMagenta);
            if (throwStep_ > 0 || fielder.throwing()) {
                arcLine(chestArc_, -math::kPi, math::kPi, render::colors::kYellow);
                dd.sphere(chestArc_.point(backAngle_), 0.04f, render::colors::kWhite);
                if (throwStep_ > 1 || fielder.throwing()) dd.sphere(chestArc_.point(releaseAngle_), 0.04f, render::colors::kRed);
            }
        }

        // Bowling: the action circle (yellow, while the mouse aims the bowler
        // before the action click) with the action point (white) and the
        // hand's ways there and on down to the bottom point (cyan); after the
        // action click, the release circle (red) with the release point
        // (magenta).
        constexpr int kCircleSegments = 48;
        auto circle = [&](const motion::ThrowArc& arc, render::Color color) {
            for (int i = 0; i < kCircleSegments; ++i) {
                const float a = math::kTwoPi * static_cast<float>(i) / kCircleSegments;
                dd.line(arc.point(a), arc.point(a + math::kTwoPi / kCircleSegments), color);
            }
        };
        if (!limbTest_) {
            const agents::Human& bowler = sim_.bowler();
            const math::Transform root = bowler.interpolatedRoot(alpha);
            const math::Vec3 actionPoint = bowler.actionCircle(actionTurn_, actionSize_ * bowler.armLength()).point(actionAngle_);
            if (bowlAim_ && bowlStep_ == 0) {
                for (int i = 0; i < kCircleSegments / 2; ++i) {  // the front half only
                    const float a = math::kTwoPi * static_cast<float>(i) / kCircleSegments;
                    dd.line(actionArc_.point(a), actionArc_.point(a + math::kTwoPi / kCircleSegments), render::colors::kYellow);
                }
                for (int i = 0; i < kCircleSegments; ++i) {
                    const float u0 = static_cast<float>(i) / kCircleSegments, u1 = static_cast<float>(i + 1) / kCircleSegments;
                    const math::Vec3 p0 = bowler.actionPathPoint(actionPoint, actionGap_, u0);
                    const math::Vec3 p1 = bowler.actionPathPoint(actionPoint, actionGap_, u1);
                    dd.line(root.transformPoint(p0), root.transformPoint(p1), render::colors::kCyan);
                    // The front hand's way, to join the bowling hand.
                    dd.line(root.transformPoint(bowler.frontPathPoint(actionPoint, actionGap_, u0)),
                            root.transformPoint(bowler.frontPathPoint(actionPoint, actionGap_, u1)), render::colors::kBlue);
                }
            }
            if (bowlAim_) dd.sphere(root.transformPoint(actionPoint), 0.04f, render::colors::kWhite);
            if (bowlAim_ && bowlStep_ <= 1) {
                // The hands' ways from the action point down to below their shoulders.
                const math::Vec3 bottom = bowler.bottomPoint();
                for (int i = 0; i < kCircleSegments; ++i) {
                    const float u0 = static_cast<float>(i) / kCircleSegments, u1 = static_cast<float>(i + 1) / kCircleSegments;
                    dd.line(root.transformPoint(bowler.arcBetween(actionPoint, bottom, u0)),
                            root.transformPoint(bowler.arcBetween(actionPoint, bottom, u1)), render::colors::kCyan);
                    dd.line(root.transformPoint(bowler.frontDownPoint(actionPoint, actionGap_, u0)),
                            root.transformPoint(bowler.frontDownPoint(actionPoint, actionGap_, u1)), render::colors::kBlue);
                }
            }
            if (bowlAim_ && (bowlStep_ >= 3 || (bowler.armDown() && bowler.bodyTurned()))) {  // hand down, body turned
                circle(releaseArc_, render::colors::kRed);
                dd.sphere(releaseArc_.point(bowlRelease_), 0.05f, render::colors::kMagenta);
            }
        }
    }

    // The humans' physics bodies (HumanBody): each part's Jolt shape (white)
    // and each joint at its pivot: hinge (yellow, a line along its axis),
    // swing-twist (cyan, its twist X red, Y green, Z blue axes); a joint with
    // a muscle (motor) has a red ring.
    drawHumanBody(sim_.fielder().humanBody(), alpha);
    if (!limbTest_) drawHumanBody(sim_.bowler().humanBody(), alpha);

    // Each player's capsule (white while it holds the body, red once let go)
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
    if (!limbTest_) drawCapsule(sim_.bowler());

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
