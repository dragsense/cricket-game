#pragma once
// Wires platform, renderer, debug UI, cameras and the simulation into the
// core fixed-timestep loop.

#include "camera/debug_camera.h"
#include "core/app.h"
#include "core/config.h"
#include "core/events.h"
#include "core/input.h"
#include "debug/debug_overlay.h"
#include "debug/fielder_panel.h"
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

    AppConfigs cfg_;
    platform::Window window_;
    std::unique_ptr<render::Renderer> renderer_;
    core::InputState input_;
    core::ActionMap actions_;
    core::EventQueue events_;
    camera::DebugCamera camera_;
    match::SimWorld sim_;
    World world_;
    TestScene scene_;
    FielderVisual fielderVisual_;
    debug::FielderPanelState fielderPanel_;
    bool fielderBodyShown_ = true;
    rig::Pose fielderPose_;                  // interpolated, render side
    std::vector<math::Mat4> fielderModels_;  // model-space joint matrices
    debug::DebugSettings debugSettings_;
    debug::DebugSettings appliedSettings_;  // last values pushed to the renderer
    std::string backendName_;
    bool imguiReady_ = false;
    bool vsync_ = true;
    float targetFrameSeconds_ = 0.0f;
};

} // namespace game
