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
#include "game/human_visual.h"
#include "game/test_scene.h"
#include "game/world.h"
#include "match/sim_world.h"
#include "platform/sdl_window.h"
#include "render/renderer.h"

#include <memory>

namespace game {

struct AppConfigs {
    core::Config engine;  // config/engine.json
    core::Config input;   // config/input.json
    core::Config scene;   // config/scene.json
    core::Config humanSkeleton;  // config/human_skeleton.json
    core::Config humanJoints;    // config/human_joints.json
    core::Config locomotion;       // config/locomotion.json
    core::Config throwing;         // config/throw.json
    core::Config physicsWorld;     // config/physics_world.json
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
    void drawHumanBody(const agents::HumanBody& body, float alpha);  // its physics shapes and joints
    void updateFielder(float alpha);
    // Re-reads the named config files (e.g. "scene.json") and re-applies them
    // to the running game. A file that fails to parse keeps its previous values.
    void reloadConfig(const std::vector<std::string>& files);
    void applyEngineConfig();
    // Limb test: F chooses the limb, the arrows move its target (Shift:
    // up/down), C sends the body there; R resets the pose.
    void updatePose(float seconds);

    // The throw (T): its circles follow the fielder, the mouse picks a point on
    // the one to click (cyan: hand height, yellow: back), each click acts at once.
    void updateThrow(float alpha);
    void drawThrow();
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
    HumanVisual humanVisual_;
    rig::Pose fielderPose_;                  // interpolated, render side
    std::vector<math::Mat4> fielderModels_;  // model-space joint matrices
    debug::DebugSettings debugSettings_;
    // Limb test (engine.json debug.limbTest): the chosen target, in F's
    // order (kPoseOrder: right foot, left foot, right hand, left hand, hips,
    // chest, head), where its goal is and how it is turned (world).
    bool limbTest_ = false;
    int poseLimb_ = 0;
    bool poseActive_ = false;      // poseTarget_ / poseTurn_ belong to the chosen target
    bool poseTurned_ = false;      // the chosen target was turned
    bool freeView_ = false;        // V: the camera moves freely, the player keeps their heading
    bool poseSent_ = false;        // the body goes to the chosen target as it is now (C)
    math::Vec3 poseTarget_{0.0f};  // world
    math::Quat poseTurn_ = math::identityQuat();  // world
    bool throwMode_ = false;       // T
    motion::ThrowArc slotArc_, swingArc_;  // world
    float throwAim_ = 0.0f, throwTilt_ = 0.0f;  // rad, the mouse's point on the circle to click, the yellow one's lean
    bool slowMotion_ = false;      // I: the whole game in slow motion
    bool mouseLook_ = false;  // off: right-drag rotates; Tab toggles mouse-look (cursor hidden)
    debug::DebugSettings appliedSettings_;  // last values pushed to the renderer
    std::string backendName_;
    bool imguiReady_ = false;
    bool vsync_ = true;
    float targetFrameSeconds_ = 0.0f;
};

} // namespace game
