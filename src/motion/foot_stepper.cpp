#include "motion/foot_stepper.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kMinStep = 0.01f;  // metres; shorter moves are not worth a step

} // namespace

void FootStepper::reset(math::Vec3 left, math::Vec3 right) {
    feet_[0] = left;
    feet_[1] = right;
    stanceWidth_ = std::fabs(left.x - right.x);
    swing_ = -1;
    t_ = 0.0f;
}

bool FootStepper::startStep(math::Vec2 dir, const StepSettings& s) {
    if (math::length2(dir) < math::kEpsilon) return false;
    const math::Vec3 step(dir.x * s.sideStepLength, 0.0f, dir.y > 0.0f ? dir.y * s.stepLength : dir.y * s.backStepLength);

    // Each foot's next spot, relative to the other (planted) foot. A foot only
    // adds the step when it leads (sideways: the foot on that side); a
    // trailing foot just closes back to stance width.
    math::Vec3 target[2];
    float move[2];
    bool leads[2];
    for (int i = 0; i < 2; ++i) {
        const math::Vec3 side(i == 0 ? stanceWidth_ : -stanceWidth_, 0.0f, 0.0f);
        leads[i] = math::dot(side, step) > 0.0f;
        target[i] = feet_[1 - i] + side + (math::dot(side, step) >= 0.0f ? step : math::Vec3(0.0f));
        target[i].y = feet_[1 - i].y;
        move[i] = math::distance(target[i], feet_[i]);
    }
    if (move[0] < kMinStep && move[1] < kMinStep) return false;

    // The foot with further to go steps; on a tie the leading foot, else alternate.
    int f;
    if (std::fabs(move[0] - move[1]) > kMinStep) f = move[0] > move[1] ? 0 : 1;
    else if (leads[0] != leads[1]) f = leads[0] ? 0 : 1;
    else f = 1 - last_;

    swing_ = f;
    from_ = feet_[f];
    to_ = target[f];
    t_ = 0.0f;
    return true;
}

void FootStepper::tick(float dt, math::Vec2 dir, const StepSettings& s) {
    if (swing_ < 0 && !startStep(dir, s)) return;

    t_ = std::min(1.0f, t_ + dt / std::max(s.stepDuration, 0.05f));
    const float ease = t_ * t_ * (3.0f - 2.0f * t_);  // smoothstep: lift off and land softly
    math::Vec3 p = math::lerp(from_, to_, ease);
    p.y += s.stepHeight * std::sin(math::kPi * t_);
    feet_[swing_] = p;

    if (t_ >= 1.0f) {  // planted
        feet_[swing_] = to_;
        last_ = swing_;
        swing_ = -1;
    }
}

} // namespace motion
