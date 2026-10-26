#pragma once
// Throw arcs (M4). Both are half circles around the throwing shoulder, as
// angles on a circle: point(a) = centre + radius (cos a side + sin a up).
// - Slot arc: square to the facing direction, like a clock face seen from
//   the front; -90 deg straight down (underarm), 0 out to the side (sidearm),
//   +90 overhead (overarm). The user picks the slot point on it.
// - Swing arc: through the slot point (angle 0) and the facing direction
//   (+90), turned by `tilt` about the shoulder-to-slot line. The hand starts
//   at -90 (behind), swings forward and releases at a chosen angle, moving
//   along the circle's tangent.
// See docs/BIOMECHANICS.md section 10. Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct ThrowArc {
    math::Vec3 centre{0.0f};      // throwing shoulder
    math::Vec3 side{1, 0, 0};     // unit, angle 0
    math::Vec3 up{0, 1, 0};       // unit, angle +90 deg
    math::Vec3 normal{0, 0, 1};   // unit, the circle's plane normal
    float radius = 0.6f;

    // Point at `angle` (radians); `reach` scales the radius (< 1 bends the elbow).
    math::Vec3 point(float angle, float reach = 1.0f) const;
    // Unit direction of travel at `angle` toward larger angles.
    math::Vec3 tangent(float angle) const;
    // Angle of the circle point nearest `p` (p projected onto the plane).
    float angleOf(math::Vec3 p) const;
    // Angle where the ray hits the circle's plane, snapped to the circle.
    // False if the ray is parallel to the plane or points away from it.
    bool rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const;
};

// Slot arc for a fielder facing `rootRotation` (character +Z forward, +X left).
ThrowArc makeThrowArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius);
// Swing arc through the slot arc's point at `slotAngle`, tilted by `tilt` radians.
ThrowArc makeSwingArc(const ThrowArc& slotArc, float slotAngle, float tilt);

// config/throw.json (BIOMECHANICS 10).
struct ThrowSettings {
    float radiusFactor = 0.95f;  // x arm length
    float windupTime = 0.3f;     // s, hand eases to the swing arc's start
    float swingTime = 0.12f;     // s, start to the release point, speeding up
    // Shoulder-to-hand distance at the start, x radius: the elbow bends
    // (cocking) and extends by the release point.
    float cockReach = 0.6f;
    float throwSpeed = 25.0f;    // m/s at release
};

} // namespace motion
