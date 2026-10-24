#include "motion/body_reposition.h"

#include <cmath>

namespace motion {

namespace {

// Moves `pelvis` so this leg's hip is within [lo, hi] of its foot.
math::Vec3 clampLeg(math::Vec3 pelvis, const LegSupport& leg, float lo, float hi) {
    math::Vec3 d = pelvis + leg.hipOffset - leg.foot;
    float len = math::length(d);
    float clamped = math::clamp(len, lo, hi);
    if (std::fabs(clamped - len) < 1e-5f) return pelvis;
    return pelvis + math::safeNormalize(d, math::kUp) * (clamped - len);
}

} // namespace

math::Vec3 repositionBody(math::Vec3 pelvis, const LegSupport& left, const LegSupport& right, float legLength,
                          float stanceWeight, const RepositionSettings& s, float dt) {
    const float w = math::clamp(stanceWeight, 0.0f, 1.0f);

    // Comfortable pelvis: standing tall above each foot, blended by stance.
    auto stand = [&](const LegSupport& leg) { return leg.foot + math::kUp * (s.standReach * legLength) - leg.hipOffset; };
    math::Vec3 goal = math::lerp(stand(left), stand(right), w);
    pelvis = math::lerp(pelvis, goal, 1.0f - std::exp(-s.followRate * dt));

    const float lo = s.minReach * legLength, hi = s.maxReach * legLength;
    const LegSupport& stance = w >= 0.5f ? right : left;
    const LegSupport& other = w >= 0.5f ? left : right;
    for (int i = 0; i < s.iterations; ++i) {
        pelvis = clampLeg(pelvis, other, lo, hi);
        pelvis = clampLeg(pelvis, stance, lo, hi);
    }
    return pelvis;
}

} // namespace motion
