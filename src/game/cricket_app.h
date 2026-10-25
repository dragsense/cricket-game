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
#include "platform/sdl_window.h"
#include "render/renderer.h"

#include <memory>

namespace game {

struct AppConfigs {
    core::Config engine;  // config/engine.json
    core::Config input;   // config/input.json
    core::Config scene;   // config/scene.json
    core::Config fielderSkeleton;  // config/fielder_skeleton.json
    core::Config fielderJoints;    // config/fielder_joints.json
    core::Config locomotion;       // config/locomotion.json
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
    // Left click on the ground: moves the ball there and makes it the reach target.
    void placeReachTarget();
    // Re-reads the named config files (e.g. "scene.json") and re-applies them
    // to the running game. A file that fails to parse keeps its previous values.
    void reloadConfig(const std::vector<std::string>& files);
    void applyEngineConfig();

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
    debug::DebugSettings debugSettings_;
    float heading_ = 0.0f;    // fielder's W heading; follows the camera only while stopped
    bool mouseLook_ = false;  // off: right-drag rotates; Tab toggles mouse-look (cursor hidden)
    debug::DebugSettings appliedSettings_;  // last values pushed to the renderer
    std::string backendName_;
    bool imguiReady_ = false;
    bool vsync_ = true;
    float targetFrameSeconds_ = 0.0f;
};

} // namespace game
