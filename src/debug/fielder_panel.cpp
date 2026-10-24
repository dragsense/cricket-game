#include "debug/fielder_panel.h"

#include <imgui.h>

namespace debug {

bool drawFielderPanel(FielderPanelState& s, const rig::SkeletonDef& def, const rig::Rig& rig, const rig::Pose& pose) {
    if (!s.open) return false;
    ImGui::SetNextWindowPos(ImVec2(12, 330), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380, 520), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Fielder", &s.open)) {
        ImGui::End();
        return false;
    }

    bool heightChanged = ImGui::SliderFloat("Height (m)", &s.height, def.minHeight, def.maxHeight, "%.2f");
    ImGui::Text("Leg length %.3f m   joints %d", rig.legLength(), rig.numJoints());

    ImGui::Checkbox("Body", &s.showBody);
    ImGui::SameLine();
    ImGui::Checkbox("Bones", &s.draw.bones);
    ImGui::SameLine();
    ImGui::Checkbox("Joints", &s.draw.joints);
    ImGui::SameLine();
    ImGui::Checkbox("Axes", &s.draw.axes);
    ImGui::SameLine();
    ImGui::Checkbox("CoM", &s.draw.centreOfMass);

    ImGui::SeparatorText("Local rotation (deg, X Y Z)");
    if (ImGui::BeginTable("bones", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Bone");
        ImGui::TableSetupColumn("X");
        ImGui::TableSetupColumn("Y");
        ImGui::TableSetupColumn("Z");
        ImGui::TableHeadersRow();
        for (int j = 0; j < rig.numJoints() && j < static_cast<int>(pose.locals.size()); ++j) {
            math::Vec3 e = math::degrees(math::eulerAngles(pose.locals[static_cast<size_t>(j)].rotation));
            int depth = 0;
            for (int p = rig.parent(j); p >= 0; p = rig.parent(p)) ++depth;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%*s%s", depth, "", rig.jointName(j).c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%6.1f", e.x);
            ImGui::TableNextColumn();
            ImGui::Text("%6.1f", e.y);
            ImGui::TableNextColumn();
            ImGui::Text("%6.1f", e.z);
        }
        ImGui::EndTable();
    }
    ImGui::End();
    return heightChanged;
}

} // namespace debug
