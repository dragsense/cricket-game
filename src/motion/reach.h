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

// Clamps `q` per axis of its rotation vector (axis * angle, radians): x, y
// and z each limited to [min, max]. Used for the pelvis (tilt, turn, side tilt).
math::Quat clampRotationVector(math::Quat q, math::Vec3 min, math::Vec3 max);

// Body translation that brings `shoulder` within `reach` of `target` while
// both hips stay within `maxLeg` of their planted feet. Each iteration pushes
// the body toward the target by the remaining error, then projects each hip
// back onto its leg's reach sphere, so a blocked hip slides along it (forward
// and down: the knees bend). There is no minimum (squat) limit.
math::Vec3 bodyReachOffset(math::Vec3 shoulder, math::Vec3 target, float reach, math::Vec3 hipL, math::Vec3 hipR,
                           math::Vec3 footL, math::Vec3 footR, float maxLeg, int iterations);

// `target` moved toward `from` so it is at most `reach` away.
math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach);

} // namespace motion
