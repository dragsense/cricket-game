#include "motion/reach.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr float kReachEpsilon = 1e-4f;  // m; closer than this counts as reached
}

Swing swingToward(math::Vec3 pivot, math::Vec3 end, math::Vec3 target, float reach) {
    Swing s;
    if (math::distance(end, target) - reach <= kReachEpsilon) return s;
    const math::Vec3 toEnd = end - pivot, toTarget = target - pivot;
    const float r = math::length(toEnd), d = math::length(toTarget);
    if (r < math::kEpsilon || d < math::kEpsilon) return s;
    const math::Vec3 a = toEnd / r, b = toTarget / d;
    const math::Vec3 axis = math::cross(a, b);
    const float sinAB = math::length(axis);
    if (sinAB < math::kEpsilon) return s;  // already aligned (or exactly opposite): nothing to swing toward
    const float between = std::atan2(sinAB, math::dot(a, b));
    const float cosNeeded = math::clamp((r * r + d * d - reach * reach) / (2.0f * r * d), -1.0f, 1.0f);
    s.angle = std::max(0.0f, between - std::acos(cosNeeded));
    s.axis = axis / sinAB;
    return s;
}

math::Quat clampRotationVector(math::Quat q, math::Vec3 min, math::Vec3 max) {
    if (q.w < 0.0f) q = -q;  // shortest arc
    const math::Vec3 v(q.x, q.y, q.z);
    const float s = math::length(v);
    if (s < math::kEpsilon) return math::identityQuat();
    const math::Vec3 rv = v * (2.0f * std::atan2(s, q.w) / s);
    const math::Vec3 c = math::clamp(rv, min, max);
    const float angle = math::length(c);
    return angle < math::kEpsilon ? math::identityQuat() : math::fromAxisAngle(c / angle, angle);
}

math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach) {
    const math::Vec3 d = target - from;
    const float len = math::length(d);
    return len <= reach ? target : from + d * (reach / len);
}

} // namespace motion
