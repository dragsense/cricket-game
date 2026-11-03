#include "motion/body_reposition_backup.h"

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

// One semi-implicit step of a damped spring toward `goal` (frequency w, ratio z).
template <typename T>
void spring(T& x, T& v, T goal, float w, float z, float dt) {
    v += (w * w * (goal - x) - 2.0f * z * w * v) * dt;
    x += v * dt;
}

} // namespace

math::Quat BodyState::rotation() const { return math::fromAxisAngle(math::kForward, drop); }

float bentReach(float thigh, float shin, float kneeFlex) {
    return std::sqrt(std::max(thigh * thigh + shin * shin + 2.0f * thigh * shin * std::cos(kneeFlex), 0.0f));
}


void repositionBody(BodyState& body, const LegSupport& left, const LegSupport& right, float legLength,
                    float stanceWeight, const RepositionSettings& s, float dt) {
    const float w = math::clamp(stanceWeight, 0.0f, 1.0f);
    const float single = std::fabs(2.0f * w - 1.0f);  // 0 both feet loaded, 1 one foot

    // Where: between the feet by the weight each carries.
    auto stand = [&](const LegSupport& leg) { return leg.foot + math::kUp * (s.standReach * legLength) - leg.hipOffset; };
    const math::Vec3 overL = stand(left), overR = stand(right);
    math::Vec3 goal = math::lerp(overL, overR, w);
    goal.x = math::lerp(0.5f * (overL.x + overR.x), goal.x, s.sideFollow);
    goal.z = math::lerp(s.restZ, goal.z, s.forwardFollow);

    // How high: as high as every loaded leg reaches straight (standReach);
    // the lowest one decides. A lifted foot sets nothing.
    const float reach = s.standReach * legLength;
    auto hipHeight = [&](const LegSupport& leg) {
        const math::Vec3 hip = goal + leg.hipOffset;
        const math::Vec2 out(hip.x - leg.foot.x, hip.z - leg.foot.z);
        return leg.foot.y + std::sqrt(std::max(reach * reach - math::dot(out, out), 0.0f)) - leg.hipOffset.y;
    };
    const float yL = hipHeight(left), yR = hipHeight(right);
    const float both = std::min(yL, yR), one = w >= 0.5f ? yR : yL;
    goal.y = math::lerp(goal.y, math::lerp(both, one, single), s.standShare);
    body.goal = goal;

    // On one foot the lifted side drops (BIOMECHANICS 4.3).
    spring(body.drop, body.dropRate, -s.standShare * s.stanceDrop * (2.0f * w - 1.0f), s.followRate, s.damping, dt);
    const math::Quat tilt = body.rotation();
    const LegSupport l{tilt * left.hipOffset, left.foot}, r{tilt * right.hipOffset, right.foot};
    spring(body.position, body.velocity, goal, s.followRate, s.damping, dt);

    // Rule 1: the legs reach. The unloaded leg only limits the reach (a lifted
    // foot must not push the body away; its knee bends more); stance leg last.
    const math::Vec3 sprung = body.position;
    body.position = clampToLegs(sprung, w >= 0.5f ? l : r, w >= 0.5f ? r : l, legLength, s);
    // The clamp stops the spring: drop the velocity pushing into it, so the
    // body does not jump once the limit lets go.
    const math::Vec3 push = body.position - sprung;
    const float pushLength = math::length(push);
    if (pushLength > 1e-6f) {
        const math::Vec3 n = push / pushLength;
        const float into = math::dot(body.velocity, n);
        if (into < 0.0f) body.velocity -= n * into;
    }
}

math::Vec3 clampToLegs(math::Vec3 pelvis, const LegSupport& first, const LegSupport& second, float legLength,
                       const RepositionSettings& s) {
    const float lo = s.minReach * legLength, hi = s.maxReach * legLength, band = s.softBand * legLength;
    for (int i = 0; i < s.iterations; ++i) {
        pelvis = clampLeg(pelvis, first, 0.0f, hi, band);
        pelvis = clampLeg(pelvis, second, lo, hi, band);
    }
    return pelvis;
}

} // namespace motion
