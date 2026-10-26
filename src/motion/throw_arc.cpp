#include "motion/throw_arc.h"

#include <cmath>

namespace motion {

math::Vec3 ThrowArc::point(float angle) const {
    return centre + radius * (std::cos(angle) * side + std::sin(angle) * up);
}

float ThrowArc::angleOf(math::Vec3 p) const {
    const math::Vec3 d = p - centre;
    return std::atan2(math::dot(d, up), math::dot(d, side));
}

bool ThrowArc::rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const {
    const float denom = math::dot(direction, forward);
    if (std::fabs(denom) < math::kEpsilon) return false;
    const float t = math::dot(centre - origin, forward) / denom;
    if (t < 0.0f) return false;
    angle = angleOf(origin + direction * t);
    return true;
}

ThrowArc makeThrowArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius) {
    ThrowArc arc;
    arc.centre = shoulder;
    arc.forward = math::safeNormalize(rootRotation * math::kForward, math::kForward);
    arc.up = math::kUp;
    arc.side = math::safeNormalize(math::cross(arc.up, arc.forward) * (leftArm ? 1.0f : -1.0f), math::kRight);
    arc.radius = radius;
    return arc;
}

} // namespace motion
