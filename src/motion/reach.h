#pragma once
// Reach math shared by every stage of hand-driven reaching (clavicle, spine,
// pelvis): the smallest swing about a pivot that brings a chain's end within
// `reach` of a target (law of cosines). See docs/BIOMECHANICS.md section 9.
// Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct Swing {
    math::Vec3 axis{1, 0, 0};  // unit axis through the pivot
    float angle = 0.0f;        // radians, >= 0; 0 when already within reach
};

// Rotation about `pivot` that moves `end` so its distance to `target` becomes
// `reach` (or, if impossible, points it straight at the target). r = |end -
// pivot|, d = |target - pivot|; the needed angle between them satisfies
// reach^2 = r^2 + d^2 - 2 r d cos(angle).
Swing swingToward(math::Vec3 pivot, math::Vec3 end, math::Vec3 target, float reach);

// `target` moved toward `from` so it is at most `reach` away.
math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach);

} // namespace motion
