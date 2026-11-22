#pragma once
// Jog and run as targets for the body's decider (as StepPlanner's: it only
// places the feet's and the hips' targets over time; the decider knows nothing
// of it). A leg's swing takes its own time and its stance ends once the hips
// have gone so far over the foot; at speed the stance is the shorter, so both
// feet are off the ground between two stances (the flight). The hips fall
// freely in the flight and the standing leg's push (a half sine) sends them
// up again. Character space. No allocations.

#include "math/math.h"
#include "motion/target_set.h"

namespace motion {

// locomotion.json "run".
struct RunPlannerSettings {
    float enterSpeed = 1.7f, exitSpeed = 1.6f;  // m/s of the hips: from walking, back to it
    float swingTime = 0.46f;                    // s, a leg's swing
    float stanceWay = 0.42f, stanceWayPerSpeed = 0.08f;  // m the hips go over a foot: stanceWay + PerSpeed x speed
    float stanceAhead = 0.4f;     // the share of that way the foot lands ahead of the hips
    float heelShare = 0.35f;      // the last share of the stance the heel rises in
    float toeOffPitch = 0.61f;    // rad, its rise
    float liftPerSpeed = 0.06f;   // m of swing height per m/s
    float peakAt = 0.4f;          // the swing share of its top
    float width = 0.6f;           // the feet across, share of their hips'
    float legReach = 0.93f;       // share of the leg's length at touchdown
    float blendTime = 0.2f;       // s, the hips' height goes over to it
    float hipsAccel = 400.0f;     // m/s^2, the decider's hips bound meanwhile (its path is physics already)
    float ballReach = 0.0f;       // m, ankle to the ball of the foot, level (the skeleton's)
    // The arms: the upper arm swings armSwing (rad) each way about armMid
    // (forward +) against the legs, the elbow bent; the trunk leans.
    float armSwing = 0.6f, armMid = -0.15f, elbowBend = 1.5f, handIn = 0.05f;
    float leanPerSpeed = 0.026f;  // rad per m/s
    float poseBlend = 0.4f;       // s, the arms and the lean come in and go
};

// The body the run goes with (the decider's plan), character space.
struct RunBody {
    math::Vec3 pelvis{0.0f};
    math::Vec3 hip[2]{};         // left, right hip joint
    float legLength[2] = {0.0f, 0.0f};  // hip to ankle, straight
    float ankleUp = 0.0f;        // a standing ankle's height above the ground
    math::Vec3 shoulder[2]{};    // left, right
    float upperArm = 0.0f, forearm = 0.0f;  // m
    float gravity = 0.0f;        // m/s^2 down
};

class RunPlanner {
public:
    // Both feet on the ground (`feet`: the ankles), the one ahead just landed.
    void start(const RunBody& body, const math::Vec3 feet[2], math::Vec3 velocity);
    void stop() { *this = RunPlanner{}; }
    // The character root moved (`toNew`: old character space to new).
    void rebase(const math::Transform& toNew);
    // One tick: writes the hips' and the feet's targets into `t` (a standing
    // foot's are there already, Pinned). `velocity`: the hips' (level).
    void tick(math::Vec3 velocity, const RunBody& body, const RunPlannerSettings& s, TargetSet& t);

    bool on() const { return on_; }
    // Slowing to a walk: the feet come down, no more lifts.
    bool ending() const { return ending_; }
    bool landed(int foot) const { return landed_[foot]; }  // this tick
    math::Vec3 spot(int foot) const { return spot_[foot]; }
    math::Vec3 hips() const { return hips_; }
    float load() const { return load_; }  // on the standing leg, x the weight (0 in the flight)
    // Stance and flight times (s) at `speed`.
    static void times(float speed, const RunPlannerSettings& s, float& stance, float& flight);

private:
    math::Vec3 landingSpot(int foot, float left, math::Vec3 velocity, const RunBody& body, const RunPlannerSettings& s) const;
    float height(const RunBody& body, const RunPlannerSettings& s);
    void moveFoot(int i, const RunPlannerSettings& s, TargetSet& t);
    void pose(const RunBody& body, const RunPlannerSettings& s, TargetSet& t) const;

    bool on_ = false, ending_ = false;
    // A foot's step: the heel rises over the ball (for `heel` s), then the swing.
    struct Swing {
        bool on = false;
        float time = 0.0f, heel = 0.0f, duration = 0.0f;
        math::Vec3 from{0.0f}, aim{0.0f}, at{0.0f};
        math::Vec3 normal = math::kUp;
        float left() const { return on ? duration - time : 0.0f; }
    };
    Swing swing_[2];
    bool landed_[2] = {false, false};
    math::Vec3 spot_[2]{};
    int foot_ = 0;        // landed last
    float age_ = 0.0f;    // s since then
    float speed_ = 0.0f;  // m/s the stance is planned for
    float blend_ = 0.0f, from_ = 0.0f, load_ = 1.0f, groundY_ = 0.0f;
    math::Vec3 hips_{0.0f};
    float poseWeight_ = 0.0f;
    math::Vec3 way_ = math::kForward;
};

} // namespace motion
