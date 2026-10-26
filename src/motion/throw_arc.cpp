#include "motion/throw_arc.h"

#include <cmath>

namespace motion {

math::Vec3 ThrowArc::point(float angle, float reach) const {
    return centre + radius * reach * (std::cos(angle) * side + std::sin(angle) * up);
}

math::Vec3 ThrowArc::tangent(float angle) const {
    return -std::sin(angle) * side + std::cos(angle) * up;
}

float ThrowArc::angleOf(math::Vec3 p) const {
    const math::Vec3 d = p - centre;
    return std::atan2(math::dot(d, up), math::dot(d, side));
}

bool ThrowArc::rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const {
    const float denom = math::dot(direction, normal);
    if (std::fabs(denom) < math::kEpsilon) return false;
    const float t = math::dot(centre - origin, normal) / denom;
    if (t < 0.0f) return false;
    angle = angleOf(origin + direction * t);
    return true;
}

ThrowArc makeThrowArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius) {
    ThrowArc arc;
    arc.centre = shoulder;
    arc.normal = math::safeNormalize(rootRotation * math::kForward, math::kForward);
    arc.up = math::kUp;
    arc.side = math::safeNormalize(math::cross(arc.up, arc.normal) * (leftArm ? 1.0f : -1.0f), math::kRight);
    arc.radius = radius;
    return arc;
}

ThrowArc makeSwingArc(const ThrowArc& slotArc, float slotAngle, float tilt) {
    // side = shoulder to the slot point; up = the facing direction turned
    // about it by `tilt` (it stays square to side, so the circle keeps
    // passing through the slot point).
    ThrowArc arc;
    arc.centre = slotArc.centre;
    arc.radius = slotArc.radius;
    arc.side = std::cos(slotAngle) * slotArc.side + std::sin(slotAngle) * slotArc.up;
    const math::Vec3 across = math::cross(arc.side, slotArc.normal);
    arc.up = math::normalize(std::cos(tilt) * slotArc.normal + std::sin(tilt) * across);
    arc.normal = math::cross(arc.side, arc.up);
    return arc;
}

} // namespace motion
