#pragma once
// Wires platform, renderer, debug UI, cameras and the simulation into the
// core fixed-timestep loop.

#include "camera/debug_camera.h"
#include "core/app.h"
#include "core/config.h"
#include "core/events.h"
#include "core/file_watcher.h"
#include "core/input.h"
#include "debug/debug_overlay.h"
#include "game/fielder_visual.h"
#include "game/test_scene.h"
#include "game/world.h"
#include "match/sim_world.h"
#include "motion/throw_arc.h"
#include "platform/sdl_window.h"
#include "render/renderer.h"

#include <memory>
#include <random>

namespace game {

struct AppConfigs {
    core::Config engine;  // config/engine.json
    core::Config input;   // config/input.json
    core::Config scene;   // config/scene.json
    core::Config fielderSkeleton;  // config/fielder_skeleton.json
    core::Config fielderJoints;    // config/fielder_joints.json
    core::Config locomotion;       // config/locomotion.json
    core::Config physicsWorld;     // config/physics_world.json
    core::Config ball;             // config/ball.json
    core::Config ballLaunch;       // config/ball_launch.json
    core::Config throwing;         // config/throw.json
    core::Config bowling;          // config/bowling.json
    std::string dir = "config";    // where the files above were loaded from
    bool liveReload = false;       // dev builds: watch `dir` and re-apply changes
};

class CricketApp final : public core::App {
public:
    explicit CricketApp(AppConfigs configs);

protected:
    bool init() override;
    void shutdown() override;
    bool pumpEvents() override;
    void fixedUpdate(double dt) override;
    void renderFrame(double frameSeconds, float alpha) override;

private:
    void drawDebugGeometry(float alpha);
    void updateFielder(float alpha);
    // Stump and bail visuals follow their interpolated Jolt bodies.
    void updateWickets(float alpha);
    // Re-reads the named config files (e.g. "scene.json") and re-applies them
    // to the running game. A file that fails to parse keeps its previous values.
    void reloadConfig(const std::vector<std::string>& files);
    void applyEngineConfig();
    // Launches the ball with preset `index` of config/ball_launch.json.
    void launchBall(int index);
    // T: releases the ball from the batting position toward the fielder
    // (config/ball_launch.json "toFielder"), to be picked.
    void releaseToFielder();
    // While the ball is held: the throw arc around the holding shoulder and the
    // release point under the mouse (drawn), from the interpolated pose.
    void updateThrowArc(float alpha);
    // The bowler's circle around the bowling shoulder; with B on, the release
    // point under the mouse (drawn), from the interpolated pose.
    void updateBowlArc(float alpha);
    // Limb test: F chooses the limb, a click grabs or lets go of its target,
    // which follows the mouse meanwhile (Shift: up/down); R resets the pose.
    void updatePose();
    // World ray from the camera through the mouse (screen centre in mouse-look).
    void mouseRay(math::Vec3& origin, math::Vec3& direction) const;

    AppConfigs cfg_;
    platform::Window window_;
    std::unique_ptr<render::Renderer> renderer_;
    core::InputState input_;
    core::ActionMap actions_;
    core::EventQueue events_;
    camera::DebugCamera camera_;
    core::FileWatcher configWatcher_;
    std::vector<std::string> changedConfigs_;  // reused each frame
    match::SimWorld sim_;
    World world_;
    TestScene scene_;
    FielderVisual fielderVisual_;
    rig::Pose fielderPose_;                  // interpolated, render side
    std::vector<math::Mat4> fielderModels_;  // model-space joint matrices
    FielderVisual bowlerVisual_;
    rig::Pose bowlerPose_;
    std::vector<math::Mat4> bowlerModels_;
    debug::DebugSettings debugSettings_;
    math::Vec3 ballPos_{0.0f};  // the ball's centre, world space
    // Throw aim (radians): slot point on slotArc_ (0 sidearm, +pi/2 overarm),
    // the swing arc's tilt about it and the release point on swingArc_.
    motion::ThrowArc slotArc_, swingArc_;
    float slotAngle_ = 0.0f, tilt_ = 0.0f, releaseAngle_ = 0.0f;
    bool aimSwing_ = false;  // the slot is fixed; aiming the release point
    // Bowling aim (radians): the circle's lean from vertical and the release
    // point on it (0 top, + in front); bowlAim_: the bowler is controlled.
    motion::ThrowArc bowlArc_;
    float bowlTilt_ = 0.0f, bowlRelease_ = 0.0f;
    bool bowlAim_ = false;
    bool bowlTilting_ = false;  // clicked on the circle: the wheel leans it
    // Limb test (engine.json debug.limbTest): the bowler, arcs and walking are
    // hidden; limb 0 right foot, 1 left foot, 2 right hand, 3 left hand.
    bool limbTest_ = false;
    int poseLimb_ = 0;
    bool poseDragging_ = false;    // the target follows the mouse
    math::Vec3 poseTarget_{0.0f};  // world
    float ballBackTimer_ = -1.0f;  // s until the bowled ball returns to the bowler; < 0 off
    std::mt19937 rng_{std::random_device{}()};  // T: release speed and landing spot
    float heading_ = 0.0f;    // fielder's W heading; follows the camera only while stopped
    bool mouseLook_ = false;  // off: right-drag rotates; Tab toggles mouse-look (cursor hidden)
    debug::DebugSettings appliedSettings_;  // last values pushed to the renderer
    std::string backendName_;
    bool imguiReady_ = false;
    bool vsync_ = true;
    float targetFrameSeconds_ = 0.0f;
};

} // namespace game
