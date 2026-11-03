#include "motion/foot_stepper.h"

#include "motion/walk_controller.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kMinStep = 0.01f;  // metres; shorter moves are not worth a step
constexpr float kStartTie = 0.05f; // metres; feet this close in distance-to-go count as level
constexpr float kPitchSettle = 0.75f;  // swing share by which the foot reaches its landing pitch
constexpr float kToeRelax = 0.3f;      // swing share by which bent toes straighten after push-off
constexpr float kFootFlatRate = 12.0f; // 1/s, a landed foot lowers to flat
constexpr float kAimLock = 0.7f;      // swing share after which the landing spot stops being re-aimed

float smoothstep(float x) {
    x = math::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// Swing height share, 0 -> 1 at `peakAt` -> 0. Eases in and out on both
// sides, so the foot never jumps up at lift-off however early the apex is.
float swingArc(float t, float peakAt) {
    const float p = math::clamp(peakAt, 0.1f, 0.9f);
    return t < p ? smoothstep(t / p) : smoothstep((1.0f - t) / (1.0f - p));
}

} // namespace

void FootStepper::reset(math::Vec3 left, math::Vec3 right) {
    feet_[0] = left;
    feet_[1] = right;
    stance_[0] = left;
    stance_[1] = right;
    stanceWidth_ = std::fabs(left.x - right.x);
    pitch_[0] = pitch_[1] = toe_[0] = toe_[1] = 0.0f;
    swing_ = early_ = -1;
    t_ = 0.0f;
    wait_ = 0.0f;
    idle_ = true;
}

void FootStepper::rebase(const math::Transform& from, const math::Transform& to) {
    const math::Transform toNew = math::inverse(to) * from;
    for (math::Vec3& f : feet_) f = toNew.transformPoint(f);
    from_ = toNew.transformPoint(from_);
    to_ = toNew.transformPoint(to_);
    earlyFrom_ = toNew.transformPoint(earlyFrom_);
    earlyTo_ = toNew.transformPoint(earlyTo_);
}

void FootStepper::tickGait(float dt, math::Vec3 velocity, const GaitTiming& g, float stepHeight, float settleDistance,
                           const GaitStyle& style) {
    const bool moving = math::length2(velocity) > math::kEpsilon;
    // A foot lands ahead of its stance spot by landAhead of its stance time
    // of travel (walking half: it passes under the hip mid-stance; running
    // less: most of the contact is behind the hips). Re-aimed every tick: the body
    // keeps moving (and turning) during the swing.
    auto target = [&](int f, float swingLeft) {
        math::Vec3 stance = stance_[f];
        stance.x *= style.widthScale;
        if (stanceTurn_ != 0.0f) stance = math::fromAxisAngle(math::kUp, stanceTurn_) * stance;
        math::Vec3 p = stance + velocity * (swingLeft + style.landAhead * g.stance);
        p.y = stance_[f].y;
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
        liftAt_ = 0.0f;
        from_ = feet_[f];
        t_ = 0.0f;
        wait_ = 0.0f;
        stretch_ = nextStretch_;
        nextStretch_ = 1.0f;
        // A swing lasts the step plus the flight, so it overlaps the other
        // foot's by the flight and each foot keeps its stance time on the ground.
        dur_ = (g.swing + g.flight) * stretch_;
    }
    // Aimed at the predicted landing spot while the swing is young; from
    // kAimLock on the spot is fixed (held in the world), so the foot comes
    // down onto it instead of chasing a moving spot into the ground.
    if (t_ < kAimLock) to_ = target(swing_, (1.0f - t_) * dur_);
    const int swing = swing_, stance = 1 - swing_;
    advanceSwing(dt, dur_, stepHeight * stretch_, style.peakAt);

    // Swing foot: pointed after toe-off, toes relax, then levels for landing.
    const float t = t_;
    pitch_[swing] = math::lerp(style.toeOffPitch, style.landPitch, smoothstep(t / kPitchSettle));
    toe_[swing] *= 1.0f - smoothstep(t / kToeRelax);

    // Stance foot: a heel-strike landing (toes up) lowers to flat.
    pitch_[stance] *= 1.0f - std::min(1.0f, kFootFlatRate * dt);
    toe_[stance] *= 1.0f - std::min(1.0f, kFootFlatRate * dt);

    // Running: the standing foot leaves the ground before the swinging one
    // lands (flight). With only the flight time left in this swing it starts
    // its own next swing, along the same arc; when this foot lands, that swing
    // carries on as the current one. Walking (no flight) never does.
    if (moving && g.flight > 0.0f && swing_ >= 0 && early_ < 0 && (1.0f - t_) * dur_ <= g.flight) {
        early_ = stance;
        earlyFrom_ = feet_[stance];
        earlyTime_ = 0.0f;
    }
    if (early_ >= 0) {
        earlyTime_ += dt;
        const float swingTime = std::max(g.swing + g.flight, 0.05f);
        const float u = math::clamp(earlyTime_ / swingTime, 0.0f, 1.0f);
        if (u < kAimLock) earlyTo_ = target(early_, std::max(swingTime - earlyTime_, 0.0f));
        math::Vec3 p = math::lerp(earlyFrom_, earlyTo_, smoothstep(u));
        p.y += stepHeight * swingArc(u, style.peakAt);
        feet_[early_] = p;
        pitch_[early_] = math::lerp(style.toeOffPitch, style.landPitch, smoothstep(u / kPitchSettle));
        if (swing_ < 0) {  // the other foot has landed: this swing is now the current one
            swing_ = early_;
            from_ = earlyFrom_;
            to_ = earlyTo_;
            dur_ = swingTime;
            t_ = u;
            wait_ = 0.0f;
            stretch_ = 1.0f;
            early_ = -1;
        }
    }
}

void FootStepper::advanceSwing(float dt, float duration, float height, float peakAt, float liftAt) {
    t_ = std::min(1.0f, t_ + dt / std::max(duration, 0.05f));
    // The foot stays down until `liftAt` (the weight moves off it), then swings.
    const float lift = math::clamp(liftAt, 0.0f, 0.9f);
    // Eases off and lands softly (zero speed at both ends).
    const float t = math::clamp((t_ - lift) / (1.0f - lift), 0.0f, 1.0f);
    math::Vec3 p = math::lerp(from_, to_, smoothstep(t));
    // Arc apex at `peakAt` (0.5: centred; earlier: heel recovery).
    p.y += height * swingArc(t, peakAt);
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

bool FootStepper::stepTo(Foot f, math::Vec3 spot) {
    if (swing_ >= 0) return false;
    swing_ = static_cast<int>(f);
    liftAt_ = 0.0f;
    from_ = feet_[swing_];
    to_ = math::Vec3(spot.x, from_.y, spot.z);
    t_ = 0.0f;
    return true;
}

void FootStepper::place(Foot f, math::Vec3 spot) {
    const int i = static_cast<int>(f);
    feet_[i] = spot;
    pitch_[i] = toe_[i] = 0.0f;
}

void FootStepper::tick(float dt, math::Vec2 dir, const StepSettings& s) {
    if (swing_ < 0) {
        if (!startStep(dir, s)) return;
        liftAt_ = s.liftAt;
    }
    advanceSwing(dt, s.stepDuration, s.stepHeight, s.peakAt, liftAt_);
}

} // namespace motion
