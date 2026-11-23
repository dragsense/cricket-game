#pragma once
// The throw's circles (the user's controls; M5): point(a) = centre + radius
// (cos a side + sin a up).
// - Height circle: round the chest, square to the facing direction, like a
//   clock face seen from the front; -90 deg straight down (underarm), 0 out to
//   the throwing side (sidearm), +90 overhead (overarm).
// - Throw circle: through the hand drawn back (its pivot), its centre on the
//   line from the standing throwing shoulder to the hand; toward the slot
//   point's way (0) and the facing direction (+90; -90 behind), turned by
//   `tilt` about that line (the hand stays on it).
// Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct ThrowArc {
    math::Vec3 centre{0.0f};
    math::Vec3 side{1.0f, 0.0f, 0.0f};    // unit, angle 0
    math::Vec3 up{0.0f, 1.0f, 0.0f};      // unit, angle +90 deg
    math::Vec3 normal{0.0f, 0.0f, 1.0f};  // unit, the circle's plane normal
    float radius = 0.6f;

    math::Vec3 point(float angle) const;
    // Angle of the circle point nearest `p` (p projected onto the plane).
    float angleOf(math::Vec3 p) const;
    // Angle where the ray hits the circle's plane, snapped to the circle.
    // False if the ray is parallel to the plane or points away from it.
    bool rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const;
    // The same circle in another space (`t`: rigid).
    ThrowArc in(const math::Transform& t) const;
};

// The height circle of a body facing +Z (+X its left).
ThrowArc makeSlotArc(math::Vec3 chest, bool leftHand, float radius);
// The throw circle through `hand`, its centre toward `shoulder`, toward the
// height circle's point at `slotAngle`, tilted by `tilt` radians about the line
// from `shoulder` to `hand`.
ThrowArc makeSwingArc(const ThrowArc& slotArc, float slotAngle, math::Vec3 shoulder, math::Vec3 hand, float tilt, float radius);

} // namespace motion
