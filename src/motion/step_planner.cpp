#include "motion/step_planner.h"

#include "motion/foot_stepper.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr float kReaim = 0.95f;  // swing share after which the spot stays
constexpr float kMinDuration = 0.05f;
constexpr float kMinReplan = 1.0f / 120.0f;  // s: a re-planned rest of the swing lasts at least a tick

// A quintic from x0 at speed v0 (per unit of tau: the speed times the
// duration) to x1 at rest (no speed, no acceleration), at tau in [0, 1]:
// x0 + v0 tau + (10 D - 6 v0) tau^3 + (-15 D + 8 v0) tau^4 + (6 D - 3 v0) tau^5,
// D = x1 - x0 (the minimum-jerk path with a start speed).
template <typename T>
T quintic(T x0, T v0, T x1, float tau) {
    const T d = x1 - x0;
    const float t2 = tau * tau, t3 = t2 * tau;
    return x0 + v0 * tau + (d * 10.0f - v0 * 6.0f) * t3 + (d * -15.0f + v0 * 8.0f) * (t3 * tau) +
           (d * 6.0f - v0 * 3.0f) * (t3 * t2);
}
} // namespace

void StepPlanner::start(int foot, math::Vec3 from, math::Vec3 spot, math::Vec3 normal) {
    phase_ = Phase::Start;
    foot_ = foot;
    time_ = share_ = lastAlong_ = lastU_ = 0.0f;
    landed_ = false;
    from_ = at_ = from;
    vel_ = math::Vec3(0.0f);
    aim_ = spot;
    normal_ = math::safeNormalize(normal, math::kUp);
    plan_ = Replan{};
}

float StepPlanner::along(float time, const StepPlannerSettings& s) const {
    const float duration = std::max(s.duration, kMinDuration);
    const float lift = std::clamp(s.liftAt, 0.0f, 0.9f);
    return math::minimumJerk(std::clamp((time / duration - lift) / (1.0f - lift), 0.0f, 1.0f));
}

void StepPlanner::retarget(math::Vec3 spot) {
    if (phase_ == Phase::None || math::distance(spot, aim_) <= 1e-4f) return;
    if (plan_.on) {
        // Re-planned: the rest from where the target is, in the time left.
        if (phase_ != Phase::Land) replan(spot, plan_.duration - plan_.time);
        return;
    }
    // The rest of the swing from where the target is now (w of the way):
    // the path's start is re-expressed so the same share lands on the new
    // spot, so the target does not jump.
    const float w = lastAlong_;
    if (w >= kReaim) return;
    const math::Vec3 at = math::lerp(from_, aim_, w);
    from_ = (at - spot * w) / (1.0f - w);
    aim_ = spot;
}

void StepPlanner::cancel() {
    if (phase_ == Phase::None) return;
    if (phase_ == Phase::Start && !plan_.on) {
        // Not lifted: it stays where it is.
        aim_ = from_;
        phase_ = Phase::Land;
        return;
    }
    // Down under where the target is, on the rest of the clock.
    const math::Vec3 under(at_.x, from_.y, at_.z);
    retarget(under);
}

void StepPlanner::setTimeLeft(float left, const StepPlannerSettings& s) {
    (void)s;
    if (phase_ == Phase::None || phase_ == Phase::Land) return;
    replan(aim_, left);
}

float StepPlanner::timeLeft(const StepPlannerSettings& s) const {
    if (phase_ == Phase::None || phase_ == Phase::Land) return 0.0f;
    if (plan_.on) return std::max(0.0f, plan_.duration - plan_.time);
    return std::max(0.0f, std::max(s.duration, kMinDuration) - time_);
}

void StepPlanner::replan(math::Vec3 spot, float left) {
    // From the target now, at its speed, to the spot in `left` s: level, a
    // quintic that arrives at rest; up, the step's arc by the swing's
    // progress (tickReplanned).
    plan_.on = true;
    plan_.from = at_;
    plan_.velocity = vel_;
    plan_.time = 0.0f;
    plan_.duration = std::max(left, kMinReplan);
    plan_.share = share_;
    plan_.along = lastAlong_;
    plan_.u = lastU_;
    aim_ = spot;
    phase_ = Phase::Up;
}

void StepPlanner::rebase(const math::Transform& toNew) {
    for (math::Vec3* p : {&from_, &aim_, &at_, &plan_.from}) *p = toNew.transformPoint(*p);
    vel_ = toNew.rotation * vel_;
    plan_.velocity = toNew.rotation * plan_.velocity;
    normal_ = toNew.rotation * normal_;
}

void StepPlanner::tickReplanned(float dt, TargetSet& t) {
    Target& f = t.foot[foot_];
    const math::Vec3 before = at_;
    plan_.time = std::min(plan_.duration, plan_.time + dt);
    const float tau = plan_.time / plan_.duration;
    const math::Vec3 n = normal_;
    // The start split into its height over the spot's ground and the rest.
    const float h0 = math::dot(plan_.from - aim_, n), vh0 = math::dot(plan_.velocity, n);
    const math::Vec3 p0 = plan_.from - n * h0, v0 = plan_.velocity - n * vh0;
    (void)vh0;
    const math::Vec3 level = quintic(p0, v0 * plan_.duration, aim_, tau);
    share_ = plan_.share + (1.0f - plan_.share) * tau;
    lastAlong_ = plan_.along + (1.0f - plan_.along) * tau;
    lastU_ = plan_.u + (1.0f - plan_.u) * tau;
    // Up: the step's own arc by the swing's progress (the same through every
    // re-plan, so they never add up), what the target was off it at the
    // re-plan fading out.
    const float off = h0 - height_ * swingArc(plan_.u, peakAt_);
    const float h = std::max(0.0f, height_ * swingArc(lastU_, peakAt_) + off * (1.0f - math::minimumJerk(tau)));
    at_ = level + n * h;
    vel_ = dt > 0.0f ? (at_ - before) / dt : math::Vec3(0.0f);
    f.mode = TargetMode::Driven;
    f.position = at_;
    f.velocity = vel_;
    f.swing = lastU_;
    f.weightPosition = f.weightRotation = 1.0f;
    if (plan_.time >= plan_.duration) phase_ = Phase::Land;  // next tick: pinned
}

bool StepPlanner::tick(float dt, const StepPlannerSettings& s, TargetSet& t) {
    landed_ = false;
    if (phase_ == Phase::None || foot_ < 0) return false;
    Target& f = t.foot[foot_];
    f.contactNormal = normal_;
    height_ = s.height;
    peakAt_ = s.peakAt;
    if (phase_ == Phase::Land) {
        // Landed: pinned on the spot; the planner's job is done.
        f.mode = TargetMode::Pinned;
        f.position = at_ = aim_;
        f.velocity = vel_ = math::Vec3(0.0f);
        f.swing = -1.0f;
        share_ = 1.0f;
        landed_ = true;
        phase_ = Phase::None;
        plan_.on = false;
        return false;
    }
    if (plan_.on) {
        tickReplanned(dt, t);
        return true;
    }
    const float duration = std::max(s.duration, kMinDuration);
    const float lift = std::clamp(s.liftAt, 0.0f, 0.9f);
    const math::Vec3 before = at_;
    time_ = std::min(duration, time_ + dt);
    share_ = time_ / duration;
    const float u = std::clamp((share_ - lift) / (1.0f - lift), 0.0f, 1.0f);
    lastU_ = u;
    lastAlong_ = math::minimumJerk(u);
    phase_ = share_ < lift ? Phase::Start : Phase::Up;
    math::Vec3 goal = math::lerp(from_, aim_, lastAlong_);
    goal += normal_ * (s.height * swingArc(u, s.peakAt));
    // Push-off (the weight shift): the heel rises, the foot rolls over its
    // ball up to the toe-off pitch, so the trailing leg stays long and the
    // hips need not drop (a determinant of gait, Saunders 1953); the swing
    // carries that lift away.
    {
        const float r = std::clamp(share_ / std::max(lift, 1e-3f), 0.0f, 1.0f);
        const float pitch = s.toeOffPitch * r * r * (3.0f - 2.0f * r);
        const float fade = 1.0f - lastAlong_;
        goal += normal_ * (fade * s.ballReach * std::sin(pitch));
        goal += math::Vec3(0.0f, 0.0f, fade * s.ballReach * (1.0f - std::cos(pitch)));
    }
    at_ = goal;
    vel_ = dt > 0.0f ? (at_ - before) / dt : math::Vec3(0.0f);
    f.mode = TargetMode::Driven;
    f.position = at_;
    f.velocity = vel_;
    f.swing = u;  // timed from the step's start (0 until it lifts): the hips go between the feet
    f.weightPosition = f.weightRotation = 1.0f;
    if (time_ >= duration) phase_ = Phase::Land;  // next tick: pinned
    return true;
}

} // namespace motion
