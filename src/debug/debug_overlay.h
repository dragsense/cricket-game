#pragma once
// Dear ImGui debug overlay: frame stats and runtime toggles.

#include "camera/debug_camera.h"
#include "core/app.h"

namespace debug {

struct DebugSettings {
    bool showOverlay = true;
    bool shadows = false;
    bool debugDraw = true;
    bool showGrid = false;
    bool showProbe = false;
    bool ikDemo = false;           // fielder IK targets sweep through every joint's range
    bool showDemoWindow = false;
    int cameraMode = 0;  // 0 = orbit, 1 = free-fly
    camera::CameraSpeeds cameraSpeeds;
    camera::CameraSpeeds defaultCameraSpeeds;  // from config, for the Reset button
};

struct OverlayInfo {
    const core::LoopStats* stats = nullptr;
    double simHz = 0;
    float renderAlpha = 0;
    size_t debugLines = 0;
    size_t debugLinesDropped = 0;
    const char* backend = "";
};

// Builds this frame's ImGui windows. Call between ImGui::NewFrame and ImGui::Render.
void drawOverlay(DebugSettings& settings, const OverlayInfo& info);

} // namespace debug
