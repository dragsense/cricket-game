#include "motion/step_planner.h"

#include "motion/foot_stepper.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr float kReaim = 0.95f;  // swing share after which the spot stays
constexpr float kMinDuration = 0.05f;

float minimumJerk(float u) { return u * u * u * (10.0f + u * (6.0f * u - 15.0f)); }  // Flash & Hogan 1985
} // namespace

void StepPlanner::start(int foot, math::Vec3 from, math::Vec3 spot, math::Vec3 normal) {
    phase_ = Phase::Start;
    foot_ = foot;
    time_ = share_ = 0.0f;
    landed_ = false;
    from_ = at_ = from;
    aim_ = spot;
    normal_ = math::safeNormalize(normal, math::kUp);
}

float StepPlanner::along(float time, const StepPlannerSettings& s) const {
    const float duration = std::max(s.duration, kMinDuration);
    const float lift = std::clamp(s.liftAt, 0.0f, 0.9f);
    return minimumJerk(std::clamp((time / duration - lift) / (1.0f - lift), 0.0f, 1.0f));
}

void StepPlanner::retarget(math::Vec3 spot) {
    if (phase_ == Phase::None || math::distance(spot, aim_) <= 1e-4f) return;
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
    if (phase_ == Phase::Start) {
        // Not lifted: it stays where it is.
        aim_ = from_;
        phase_ = Phase::Land;
        return;
    }
    // Down under where the target is, on the rest of the clock.
    const math::Vec3 under(at_.x, from_.y, at_.z);
    retarget(under);
}

void StepPlanner::rebase(const math::Transform& toNew) {
    for (math::Vec3* p : {&from_, &aim_, &at_}) *p = toNew.transformPoint(*p);
    normal_ = toNew.rotation * normal_;
}

bool StepPlanner::tick(float dt, const StepPlannerSettings& s, TargetSet& t) {
    landed_ = false;
    if (phase_ == Phase::None || foot_ < 0) return false;
    Target& f = t.foot[foot_];
    f.contactNormal = normal_;
    if (phase_ == Phase::Land) {
        // Landed: pinned on the spot; the planner's job is done.
        f.mode = TargetMode::Pinned;
        f.position = at_ = aim_;
        f.velocity = math::Vec3(0.0f);
        f.swing = -1.0f;
        share_ = 1.0f;
        landed_ = true;
        phase_ = Phase::None;
        return false;
    }
    const float duration = std::max(s.duration, kMinDuration);
    const float lift = std::clamp(s.liftAt, 0.0f, 0.9f);
    const math::Vec3 before = at_;
    time_ = std::min(duration, time_ + dt);
    share_ = time_ / duration;
    const float u = std::clamp((share_ - lift) / (1.0f - lift), 0.0f, 1.0f);
    lastAlong_ = minimumJerk(u);
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
    f.mode = TargetMode::Driven;
    f.position = at_;
    f.velocity = dt > 0.0f ? (at_ - before) / dt : math::Vec3(0.0f);
    f.swing = u;  // timed from the step's start (0 until it lifts): the hips go between the feet
    f.weightPosition = f.weightRotation = 1.0f;
    if (time_ >= duration) phase_ = Phase::Land;  // next tick: pinned
    return true;
}

} // namespace motion
