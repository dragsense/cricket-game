#include "motion/throw_arc.h"

#include <cmath>

namespace motion {

math::Vec3 ThrowArc::point(float angle) const { return centre + radius * (std::cos(angle) * side + std::sin(angle) * up); }

float ThrowArc::angleOf(math::Vec3 p) const {
    const math::Vec3 d = p - centre;
    return std::atan2(math::dot(d, up), math::dot(d, side));
}

bool ThrowArc::rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const {
    const float denom = math::dot(direction, normal);
    if (std::abs(denom) < math::kEpsilon) return false;
    const float t = math::dot(centre - origin, normal) / denom;
    if (t < 0.0f) return false;
    angle = angleOf(origin + direction * t);
    return true;
}

ThrowArc ThrowArc::in(const math::Transform& t) const {
    ThrowArc a = *this;
    a.centre = t.transformPoint(centre);
    a.side = t.transformDirection(side);
    a.up = t.transformDirection(up);
    a.normal = t.transformDirection(normal);
    return a;
}

ThrowArc makeSlotArc(math::Vec3 chest, bool leftHand, float radius) {
    ThrowArc a;
    a.centre = chest;
    a.normal = math::kForward;
    a.up = math::kUp;
    a.side = math::cross(a.up, a.normal) * (leftHand ? 1.0f : -1.0f);
    a.radius = radius;
    return a;
}

ThrowArc makeSwingArc(const ThrowArc& slotArc, float slotAngle, math::Vec3 shoulder, math::Vec3 hand, float tilt, float radius) {
    // side = shoulder to the slot point, up = the facing direction square to it;
    // that plane turned as little as holds the line from the shoulder to the
    // hand, and the circle along it so the hand is on it (its pivot).
    ThrowArc a;
    a.radius = radius;
    const math::Vec3 side = math::safeNormalize(slotArc.point(slotAngle) - shoulder, slotArc.side);
    const math::Vec3 up = math::safeNormalize(slotArc.normal - side * math::dot(slotArc.normal, side), math::kUp);
    const math::Vec3 toHand = math::safeNormalize(hand - shoulder, side);
    math::Vec3 normal = math::cross(side, up);
    normal = math::safeNormalize(normal - toHand * math::dot(normal, toHand), normal);
    a.side = math::safeNormalize(side - normal * math::dot(side, normal), toHand);
    a.up = math::cross(normal, a.side);
    a.centre = hand - toHand * radius;
    const math::Quat lean = math::fromAxisAngle(toHand, tilt);
    a.side = lean * a.side;
    a.up = lean * a.up;
    a.normal = math::cross(a.side, a.up);
    return a;
}

} // namespace motion
