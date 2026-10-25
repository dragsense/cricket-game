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

// Base of support (BIOMECHANICS 3) as a capsule `radius` wide around the
// segment between the two foot centres `a` and `b` (ground plane x, z).
// Returns how far `com` is outside it, as a vector from the capsule's
// nearest point to `com`; zero inside.
math::Vec2 supportExcess(math::Vec2 com, math::Vec2 a, math::Vec2 b, float radius);

// Fitts' law movement time (s) to move `distance` onto a target `width`
// wide: a + b * log2(2D / W), never below a (MacKenzie 1992; BIOMECHANICS 9.6).
float movementTime(float distance, float width, float a, float b);

// A ball moving in a straight line: time (s) until it is closest to `point`
// on the ground plane (x, z), i.e. crosses the line through `point` square to
// its path. Negative once it has passed; 0 if it is not moving.
float timeToClosest(math::Vec3 ball, math::Vec3 velocity, math::Vec3 point);

// `target` moved toward `from` so it is at most `reach` away.
math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach);

} // namespace motion
