#include "debug/axis_gizmo.h"

#include <imgui.h>

#include <algorithm>
#include <array>

namespace debug {

namespace {

constexpr float kRadius = 42.0f;  // pixels, axis length
constexpr float kMargin = 20.0f;
constexpr float kTipRadius = 9.0f;

struct GizmoAxis {
    math::Vec3 dir;
    ImU32 color;
    const char* label;  // nullptr for the negative (unlabelled) ends
};

} // namespace

void drawAxisGizmo(math::Vec3 eye, math::Vec3 target, math::Vec3 up) {
    math::Vec3 forward = math::safeNormalize(target - eye, math::Vec3(0, 0, -1));
    math::Vec3 right = math::safeNormalize(math::cross(forward, up), math::kRight);
    math::Vec3 camUp = math::cross(right, forward);

    const ImU32 red = IM_COL32(230, 60, 60, 255), green = IM_COL32(60, 210, 80, 255), blue = IM_COL32(70, 120, 240, 255);
    const ImU32 grey = IM_COL32(170, 170, 170, 200);
    std::array<GizmoAxis, 6> axes{{{{1, 0, 0}, red, "X"},
                                   {{0, 1, 0}, green, "Y"},
                                   {{0, 0, 1}, blue, "Z"},
                                   {{-1, 0, 0}, grey, nullptr},
                                   {{0, -1, 0}, grey, nullptr},
                                   {{0, 0, -1}, grey, nullptr}}};
    // Draw the farthest ends first so nearer ones overlap them.
    std::sort(axes.begin(), axes.end(),
              [&](const GizmoAxis& a, const GizmoAxis& b) { return math::dot(a.dir, forward) > math::dot(b.dir, forward); });

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImVec2 centre(vp->Pos.x + vp->Size.x - kMargin - kRadius - kTipRadius, vp->Pos.y + kMargin + kRadius + kTipRadius);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    dl->AddCircleFilled(centre, kRadius + kTipRadius + 4.0f, IM_COL32(20, 20, 24, 120));

    for (const GizmoAxis& a : axes) {
        ImVec2 tip(centre.x + kRadius * math::dot(a.dir, right), centre.y - kRadius * math::dot(a.dir, camUp));
        if (a.label) {
            dl->AddLine(centre, tip, a.color, 3.0f);
            dl->AddCircleFilled(tip, kTipRadius, a.color);
            ImVec2 ts = ImGui::CalcTextSize(a.label);
            dl->AddText(ImVec2(tip.x - 0.5f * ts.x, tip.y - 0.5f * ts.y), IM_COL32(255, 255, 255, 255), a.label);
        } else {
            dl->AddCircleFilled(tip, kTipRadius * 0.6f, a.color);
        }
    }
}

} // namespace debug
