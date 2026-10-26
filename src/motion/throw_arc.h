#pragma once
// Throw arc (M4): a circle around the throwing shoulder in the plane square to
// the facing direction, like a clock face seen from the front. The release
// point is an angle on it: -90 deg straight down (underarm), 0 out to the side
// (sidearm, T-pose), +90 overhead (overarm). See docs/BIOMECHANICS.md section 10.
// Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct ThrowArc {
    math::Vec3 centre{0.0f};   // throwing shoulder, world
    math::Vec3 side{1, 0, 0};  // unit, out from the body on the throwing arm's side
    math::Vec3 up{0, 1, 0};    // unit
    math::Vec3 forward{0, 0, 1};  // unit facing direction, the circle's normal
    float radius = 0.6f;

    // Point on the circle at `angle` (radians).
    math::Vec3 point(float angle) const;
    // Angle of the circle point nearest `p` (p projected onto the plane).
    float angleOf(math::Vec3 p) const;
    // Angle where the ray hits the circle's plane, snapped to the circle.
    // False if the ray is parallel to the plane or points away from it.
    bool rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const;
};

// Arc for a fielder facing `rootRotation` (character +Z forward, +X left).
ThrowArc makeThrowArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius);

} // namespace motion
