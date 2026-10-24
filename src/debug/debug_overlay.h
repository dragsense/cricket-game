#pragma once
// Dear ImGui debug overlay: frame stats and runtime toggles.

#include "core/app.h"

namespace debug {

struct DebugSettings {
    bool showOverlay = true;
    bool shadows = true;
    bool debugDraw = true;
    bool showGrid = true;
    bool showProbe = true;
    bool showDemoWindow = false;
    int cameraMode = 0;  // 0 = orbit, 1 = free-fly
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
