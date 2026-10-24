#pragma once
// Scene orientation gizmo in the top-right screen corner (Unity style):
// world X red, Y green, Z blue, rotating with the camera.

#include "math/math.h"

namespace debug {

// Call between ImGui::NewFrame and ImGui::Render.
void drawAxisGizmo(math::Vec3 eye, math::Vec3 target, math::Vec3 up);

} // namespace debug
