#pragma once
// The example planner: one foot step as targets for the body's decider
// (DECISIONS 2026-11-10 "Whole-body decider on targets"). It only places the
// stepping foot's target over time on one clock: Start (the foot stays on
// its spot, Driven, so its support weight leaves it: the weight shifts to the
// other foot), Up (the swing: minimum jerk along, an arc up and down, the
// heel rising first over the ball of the foot), Land (Pinned on the spot).
// A new spot mid-way re-aims the rest of the swing from where the target is;
// a cancel brings the foot down under where it is. A new time left (the
// foot lands with the body) keeps the step's progress and changes only the
// rest of its clock. The decider does the rest (hips, trunk, other leg,
// head). Character space. No allocations.

#include "math/math.h"
#include "motion/target_set.h"

namespace motion {

struct StepPlannerSettings {
    float duration = 0.0f;     // s, the whole step (the decider times each step)
    float liftAt = 0.0f;       // share of the step the foot stays down first (the weight shifts)
    float height = 0.2f;       // m, the ankle's apex above the spot
    float peakAt = 0.5f;       // swing share of the apex
    float toeOffPitch = 0.0f;  // rad, the heel's rise at push-off (the foot rolls over its ball)
    float ballReach = 0.0f;    // m, ankle to the ball of the foot, level
};

class StepPlanner {
public:
    enum class Phase { None, Start, Up, Land };

    // Begins a step of `foot` (0 left, 1 right) from `from` (its spot now,
    // the ankle) to `spot` (the ankle where it lands), on ground with
    // `normal` there.
    void start(int foot, math::Vec3 from, math::Vec3 spot, math::Vec3 normal = math::kUp);
    // A new landing spot: the rest of the swing goes there (late in the
    // swing the spot stays; re-planned: up to the landing).
    void retarget(math::Vec3 spot);
    // The foot comes down under where its target is (in Start: it stays).
    void cancel();
    // The rest of the step takes `left` seconds (0: it lands now): its
    // progress stays, its clock (`s.duration`, the step's own timing) is
    // set so the rest lasts that long; the path goes on from where it is.
    void setTimeLeft(float left, StepPlannerSettings& s);
    float timeLeft(const StepPlannerSettings& s) const;  // s until it lands
    // The character root moved (`toNew`: old character space to new).
    void rebase(const math::Transform& toNew);
    // One tick: writes the stepping foot's target into `t` (Driven on its
    // way, Pinned as it lands). True while the step runs.
    bool tick(float dt, const StepPlannerSettings& s, TargetSet& t);

    bool running() const { return phase_ != Phase::None; }
    Phase phase() const { return phase_; }
    int foot() const { return foot_; }
    float share() const { return share_; }      // of the step's clock, 0..1
    bool landed() const { return landed_; }     // the foot landed this tick
    math::Vec3 landing() const { return aim_; } // where it lands (character space)
    math::Vec3 target() const { return at_; }   // the foot's target now
    float along() const { return lastAlong_; }  // of the way from its spot to the landing, 0..1

private:
    float along(float time, const StepPlannerSettings& s) const;

    Phase phase_ = Phase::None;
    int foot_ = -1;
    float time_ = 0.0f, share_ = 0.0f, lastAlong_ = 0.0f, lastU_ = 0.0f;
    bool landed_ = false;
    math::Vec3 from_{0.0f}, aim_{0.0f}, at_{0.0f}, vel_{0.0f};
    math::Vec3 normal_ = math::kUp;
};

} // namespace motion
