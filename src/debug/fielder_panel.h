#pragma once
// ImGui "Fielder" panel: height slider, skeleton draw toggles and a bone list
// with local rotation readouts.

#include "debug/skeleton_debug.h"
#include "rig/rig.h"

namespace debug {

struct FielderPanelState {
    bool open = true;
    bool showBody = true;
    SkeletonDrawFlags draw;
    float height = 1.8f;
};

// Returns true when the height slider changed this frame.
bool drawFielderPanel(FielderPanelState& state, const rig::SkeletonDef& def, const rig::Rig& rig, const rig::Pose& pose);

} // namespace debug
