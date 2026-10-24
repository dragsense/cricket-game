#include "motion/foot_stepper.h"

#include "motion/walk_controller.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kMinStep = 0.01f;  // metres; shorter moves are not worth a step
constexpr float kShuffleWidthFraction = 0.6f;  // closing foot stops this share of stance width from the other
constexpr float kStartTie = 0.05f; // metres; feet this close in distance-to-go count as level
constexpr float kPitchSettle = 0.75f;  // swing share by which the foot reaches its landing pitch
constexpr float kToeRelax = 0.3f;      // swing share by which bent toes straighten after push-off
constexpr float kFootFlatRate = 12.0f; // 1/s, a landed foot lowers to flat

float smoothstep(float x) {
    x = math::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

} // namespace

void FootStepper::reset(math::Vec3 left, math::Vec3 right) {
    feet_[0] = left;
    feet_[1] = right;
    stance_[0] = left;
    stance_[1] = right;
    stanceWidth_ = std::fabs(left.x - right.x);
    pitch_[0] = pitch_[1] = toe_[0] = toe_[1] = 0.0f;
    swing_ = -1;
    t_ = 0.0f;
    wait_ = 0.0f;
    idle_ = true;
}

void FootStepper::rebase(const math::Transform& from, const math::Transform& to) {
    const math::Transform toNew = math::inverse(to) * from;
    for (math::Vec3& f : feet_) f = toNew.transformPoint(f);
    from_ = toNew.transformPoint(from_);
    to_ = toNew.transformPoint(to_);
}

void FootStepper::tickGait(float dt, math::Vec3 velocity, const GaitTiming& g, float stepHeight, float settleDistance,
                           const GaitStyle& style) {
    const bool moving = math::length2(velocity) > math::kEpsilon;
    // A foot lands ahead of its stance spot by half its stance time of travel,
    // so it passes under the hip mid-stance. Re-aimed every tick: the body
    // keeps moving (and turning) during the swing.
    auto target = [&](int f, float swingLeft) {
        math::Vec3 p = stance_[f] + velocity * (swingLeft + 0.5f * g.stance);
        p.y = stance_[f].y;
        p.x = stance_[f].x * style.widthScale + velocity.x * (swingLeft + 0.5f * g.stance);
        // Sideways walking is a side shuffle: the lead foot steps out and the
        // trail foot closes beside it; the feet never cross (+X is left).
        const float other = feet_[1 - f].x, gap = kShuffleWidthFraction * stanceWidth_ * style.widthScale;
        p.x = f == 0 ? std::max(p.x, other + gap) : std::min(p.x, other - gap);
        return p;
    };

    if (swing_ < 0) {
        wait_ += dt;
        if (wait_ < g.period - g.swing) return;  // double support
        float move[2];
        for (int i = 0; i < 2; ++i) move[i] = math::distance(target(i, g.swing), feet_[i]);
        int f = 1 - last_;
        if (moving && idle_) {
            // Starting from standing: no swap from the last walk. Sideways, the
            // foot on that side always leads (A: left, D: right). Otherwise the
            // foot with further to go; if level, the one that started last walk.
            if (std::fabs(velocity.x) > std::fabs(velocity.z)) f = velocity.x > 0.0f ? 0 : 1;  // +X is left
            else if (std::fabs(move[0] - move[1]) > kStartTie) f = move[0] > move[1] ? 0 : 1;
            else f = start_;
            start_ = f;
            idle_ = false;
        }
        if (!moving) {  // settle: the foot furthest from its stance spot, if it is far off
            idle_ = true;
            for (int i = 0; i < 2; ++i)  // a foot left lifted by a run's flight always comes down
                if (feet_[i].y > stance_[i].y + kMinStep) move[i] += settleDistance;
            if (move[0] < settleDistance && move[1] < settleDistance) return;
            f = move[0] > move[1] ? 0 : 1;
        }
        swing_ = f;
        from_ = feet_[f];
        t_ = 0.0f;
        wait_ = 0.0f;
        dur_ = g.swing;
    }
    to_ = target(swing_, (1.0f - t_) * dur_);
    const int swing = swing_, stance = 1 - swing_;
    advanceSwing(dt, dur_, stepHeight, style.peakAt);

    // Swing foot: pointed after toe-off, toes relax, then levels for landing.
    const float t = t_;
    pitch_[swing] = math::lerp(style.toeOffPitch, style.landPitch, smoothstep(t / kPitchSettle));
    toe_[swing] *= 1.0f - smoothstep(t / kToeRelax);

    // Stance foot: late in the other foot's swing the heel rises and the foot
    // rolls over its ball (toes stay down), then the toes push off into the
    // flight. The next swing starts from there.
    if (style.heelRise > 0.0f && moving) {
        const float roll = smoothstep((t - (1.0f - style.heelRise)) / style.heelRise);
        const float flightShare = dur_ > 0.0f ? std::min(g.flight / dur_, 1.0f) : 0.0f;
        const float lift = flightShare > 0.0f ? smoothstep((t - (1.0f - flightShare)) / flightShare) : 0.0f;
        // Before the roll, a heel-strike landing (toes up) lowers to flat.
        const float pitch = roll > 0.0f ? style.toeOffPitch * roll
                                        : pitch_[stance] * (1.0f - std::min(1.0f, kFootFlatRate * dt));
        pitch_[stance] = pitch;
        toe_[stance] = -std::max(pitch, 0.0f);
        feet_[stance].y = stance_[stance].y + style.ballDistance * std::sin(std::max(pitch, 0.0f)) + style.flightLift * lift;
    }
}

void FootStepper::advanceSwing(float dt, float duration, float height, float peakAt) {
    t_ = std::min(1.0f, t_ + dt / std::max(duration, 0.05f));
    math::Vec3 p = math::lerp(from_, to_, smoothstep(t_));  // lift off and land softly
    // Arc apex at `peakAt` (0.5: centred sine; earlier: heel recovery).
    const float shape = std::log(0.5f) / std::log(math::clamp(peakAt, 0.1f, 0.9f));
    p.y += height * std::sin(math::kPi * std::pow(t_, shape));
    feet_[swing_] = p;

    if (t_ >= 1.0f) {  // planted
        feet_[swing_] = to_;
        last_ = swing_;
        swing_ = -1;
    }
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
    advanceSwing(dt, s.stepDuration, s.stepHeight);
}

} // namespace motion
