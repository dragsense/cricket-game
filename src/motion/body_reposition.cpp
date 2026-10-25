#include "motion/body_reposition.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

// Moves `pelvis` so this leg's hip is within [lo, hi] of its foot. The top
// eases in over `band` (smooth, no kink as the leg nears straight).
math::Vec3 clampLeg(math::Vec3 pelvis, const LegSupport& leg, float lo, float hi, float band) {
    math::Vec3 d = pelvis + leg.hipOffset - leg.foot;
    float len = math::length(d);
    float clamped = std::max(len, lo);
    const float knee = hi - band;
    if (band > 0.0f && clamped > knee) clamped = hi - band * std::exp(-(clamped - knee) / band);
    else clamped = std::min(clamped, hi);
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
    goal.x = math::lerp(math::lerp(stand(left).x, stand(right).x, 0.5f), goal.x, s.sideFollow);
    goal.z = math::lerp(s.restZ, goal.z, s.forwardFollow);
    pelvis = math::lerp(pelvis, goal, 1.0f - std::exp(-s.followRate * dt));

    const float lo = s.minReach * legLength, hi = s.maxReach * legLength, band = s.softBand * legLength;
    const LegSupport& stance = w >= 0.5f ? right : left;
    const LegSupport& other = w >= 0.5f ? left : right;
    for (int i = 0; i < s.iterations; ++i) {
        // The unloaded leg only limits the reach: a tucked (lifted) foot must
        // not push the body away; its knee just bends more.
        pelvis = clampLeg(pelvis, other, 0.0f, hi, band);
        pelvis = clampLeg(pelvis, stance, lo, hi, band);
    }
    return pelvis;
}

} // namespace motion
