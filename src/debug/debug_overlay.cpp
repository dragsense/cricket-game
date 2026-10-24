#include "debug/debug_overlay.h"

#include <imgui.h>

namespace debug {

void drawOverlay(DebugSettings& s, const OverlayInfo& info) {
    if (s.showDemoWindow) ImGui::ShowDemoWindow(&s.showDemoWindow);
    if (!s.showOverlay) return;

    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.8f);
    if (!ImGui::Begin("Debug", &s.showOverlay, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    const core::LoopStats& st = *info.stats;
    ImGui::Text("FPS         %6.1f", st.fps);
    ImGui::Text("Frame time  %6.2f ms", st.frameMs);
    ImGui::Text("Sim ticks/s %6.1f  (target %.0f)", st.simTicksPerSecond, info.simHz);
    ImGui::Text("Sim time    %8.2f s  (%lld ticks)", st.simTime, st.simTicks);
    ImGui::Text("Alpha       %6.3f", info.renderAlpha);
    ImGui::Text("Backend     %s", info.backend);
    ImGui::Text("Debug lines %zu%s", info.debugLines, info.debugLinesDropped ? " (DROPPING)" : "");

    ImGui::Separator();
    ImGui::Checkbox("Shadows", &s.shadows);
    ImGui::Checkbox("Debug draw", &s.debugDraw);
    ImGui::BeginDisabled(!s.debugDraw);
    ImGui::Checkbox("Grid", &s.showGrid);
    ImGui::EndDisabled();
    ImGui::Checkbox("Interpolation probe", &s.showProbe);
    ImGui::Checkbox("ImGui demo", &s.showDemoWindow);

    ImGui::Separator();
    ImGui::TextUnformatted("Camera (C to switch)");
    ImGui::RadioButton("Orbit", &s.cameraMode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Free-fly", &s.cameraMode, 1);
    ImGui::TextDisabled(s.cameraMode == 0 ? "RMB drag: rotate  wheel: zoom  WASD: pan"
                                           : "RMB hold: look  WASD/QE: move  Shift: fast");
    ImGui::TextDisabled("F1 overlay  F2 debug draw  Esc quit");
    ImGui::End();
}

} // namespace debug
