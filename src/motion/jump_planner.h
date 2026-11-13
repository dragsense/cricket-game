#pragma once
// A standing jump as one target for the body's decider: the hips (the user's
// design: the hips are the active target, every other part follows by its
// passive share). The hips go down, then are pushed up past standing,
// faster and faster; the physics body flies on that speed; on landing they
// sink and rise back. Numbers BIOMECHANICS 6. Phases:
//  Crouch: down (unweighting and braking); held at the bottom while the
//   jump is held.
//  Push: up past standing; take-off at its end.
//  Flight: until the caller sees the real feet reach the ground.
//  Land: the hips sink to absorb it, then rise back to standing.
// Character space. No allocations.

#include "math/math.h"
#include "motion/target_set.h"

namespace motion {

// config/locomotion.json "jump" (BIOMECHANICS 6).
struct JumpPlannerSettings {
    float crouchDrop = 0.3f;   // m, the hips below standing at the bottom (COM drop 0.36 m, 6.1)
    float crouchTime = 0.7f;   // s, standing to the bottom (countermovement 0.70 s)
    float pushTime = 0.3f;     // s, the bottom to take-off (propulsion 0.30 s)
    float pushLift = 0.1f;     // m, the hips above standing at take-off (heels high)
    float landDrop = 0.15f;    // m, the hips below standing as the landing absorbs
    float landTime = 0.22f;    // s, touchdown to the lowest (peak knee flexion 0.18-0.26 s, 6.3)
    float recoverTime = 0.5f;  // s, the lowest back to standing
    float maxFlight = 1.2f;    // s, at most in the air before the landing starts anyway
    // The plan's hips bounds while jumping (the walk's stay as they are).
    float hipsMaxSpeed = 3.2f, hipsMaxAccel = 15.0f;
};

class JumpPlanner {
public:
    enum class Phase { None, Crouch, Push, Flight, Land };

    // Begins the crouch; `standingHips` is the pelvis's height standing
    // (character space).
    void start(float standingHips);
    // The jump key let go: the push starts (from wherever the crouch is).
    void release();
    // The real feet reached the ground.
    void touchdown();
    // One tick: writes the hips' target into `t`. True while the jump runs.
    bool tick(float dt, const JumpPlannerSettings& s, TargetSet& t);

    bool running() const { return phase_ != Phase::None; }
    Phase phase() const { return phase_; }
    bool airborne() const { return phase_ == Phase::Flight; }
    float phaseTime() const { return time_; }

private:
    Phase phase_ = Phase::None;
    bool held_ = false;
    float time_ = 0.0f;
    float standing_ = 0.0f;
    float bottom_ = 0.0f;  // the crouch's depth when the push starts (m below standing)
};

} // namespace motion
