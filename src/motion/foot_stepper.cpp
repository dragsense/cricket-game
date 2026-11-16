#include "motion/foot_stepper.h"

#include <cmath>

namespace motion {

namespace {

float smoothstep(float x) {
    x = math::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

} // namespace

float swingArc(float t, float peakAt) {
    const float p = math::clamp(peakAt, 0.1f, 0.9f);
    return t < p ? smoothstep(t / p) : smoothstep((1.0f - t) / (1.0f - p));
}

void FootStepper::reset(math::Vec3 left, math::Vec3 right) {
    feet_[0] = left;
    feet_[1] = right;
}

void FootStepper::rebase(const math::Transform& from, const math::Transform& to) {
    const math::Transform toNew = math::inverse(to) * from;
    for (math::Vec3& f : feet_) f = toNew.transformPoint(f);
}

} // namespace motion
