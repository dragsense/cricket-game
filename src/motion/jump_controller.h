#pragma once
// Jump: countermovement (crouch, arms back), push-off (legs extend, arms
// swing), ballistic flight (the body rises and falls under gravity, the feet
// tuck), landing (knees absorb) and recovery. Standing: straight up, or
// toward any direction in character space (left, right, forward, back);
// holding the load deepens the crouch and the take-off speed grows with it.
// Moving (walk, jog, run): a quick dip, the speed carried (by the fielder).
// Arms per docs research: standing, both swing back then up along the jump
// (overhead up, forward forward, less going back; sideways the lead arm out
// and up, the trail arm across the chest), landing forward (out to the sides
// after a side jump); moving, the gait's opposed swing grows (the arm opposite
// the driving knee punches forward-up). Outputs offsets for the body, feet
// and arms; the fielder applies them.
// Tunables in config/locomotion.json "jump". Pure math, no allocations.

#include "math/math.h"

#include <algorithm>

namespace motion {

struct JumpSettings {
    float crouchDepth = 0.18f;     // m the pelvis drops at the deepest load
    float loadTime = 0.35f;        // s to the deepest load while held
    float minLoad = 0.4f;          // share of the deepest load a tap gives
    float moveLoad = 0.6f;         // load share of a jump from walking or running (a quick dip, no hold)
    float crouchLean = 0.35f;      // rad forward trunk lean at the deepest load (hips flex)
    float pushTime = 0.16f;        // s, legs extend from the crouch to take-off
    float movePushTime = 0.08f;    // s, the same moving (one quick push off the stance foot)
    float moveHeight = 0.06f;      // m the body rises in a moving bound (flight ~0.22 s)
    float moveKnee = 0.35f;        // x leg length the driving foot lifts (knee drive) at take-off
    float moveReach = 0.4f;      // x leg length ahead of the body the swing foot drives to (lands on it)
    float moveTrail = 0.5f;        // x leg length behind the body the take-off foot trails in the air
    float pushLift = 0.04f;        // m the pelvis is above standing at take-off
    float height = 0.35f;          // m the body rises after take-off at the deepest load
    float forwardDistance = 0.9f;  // m travelled at the deepest load
    float backDistance = 0.45f;
    float sideDistance = 0.7f;
    float tuck = 0.12f;            // m the feet draw up mid-flight
    float lean = 0.2f;             // rad lean toward the jump direction from push-off
    float landDepth = 0.14f;       // m the pelvis sinks on landing
    float landTime = 0.12f;        // s down into the absorb
    float recoverTime = 0.35f;     // s back up to standing
    float gravity = 9.81f;         // m/s^2
    // Arms (radians). Shoulder flexion: + forward-up; abduction: + out to the side.
    float armBack = 0.87f;         // extension at the deepest load (straight up)
    float armBackForward = 1.2f;   // extension at the deepest load (forward jump)
    float armUp = 2.7f;            // flexion at take-off, straight up
    float armUpForward = 2.2f;     // forward jump
    float armUpBack = 1.55f;       // backward jump
    float armUpSide = 2.4f;        // side jump, lead arm
    float sideOut = 1.0f;          // side jump, lead arm abduction at take-off
    float acrossFlex = 1.2f;       // side jump, trail arm flexion across the chest
    float across = 0.5f;           // side jump, trail arm adduction (toward the other side)
    float flightFall = 0.5f;       // share of the way to the landing pose the arms go in flight
    float landFlex = 1.2f;         // flexion on landing (arms forward)
    float landSideOut = 1.2f;      // side jump, abduction on landing (arms out)
    float landSideFlex = 0.35f;    // side jump, flexion on landing
    float elbowLoad = 0.6f;        // elbow flexion on the back swing
    float elbowTop = 0.25f;        // elbow flexion at take-off
    float elbowLand = 0.6f;        // elbow flexion on landing
    float moveDrive = 1.9f;        // moving: drive arm flexion at full run speed
    float moveBack = 0.8f;         // moving: other arm extension at full run speed
    float moveMin = 0.35f;         // moving: share of the drive at walking pace
    float elbowMove = 1.4f;        // moving: elbow flexion (bent, as when running)
};

enum class JumpStage { None, Load, Push, Flight, Land };

// One arm's jump pose (radians): shoulder flexion (+ forward-up) and
// abduction (+ out to its side), elbow flexion.
struct JumpArm {
    float flex = 0.0f, abduct = 0.0f, elbow = 0.0f;
};

class JumpController {
public:
    // Begins the crouch (Load) if not already jumping. For a moving jump,
    // `driveLeft` is the knee driving forward-up (the swing leg) and
    // `speedShare` 0..1 the speed against full running.
    void start(bool moving = false, bool driveLeft = false, float speedShare = 0.0f);
    // Ends the load: takes off toward `direction` (character space, x = left,
    // y = forward; zero = straight up, length clamped to 1) once at least
    // minLoad deep (or `minShare`, if larger).
    void release(math::Vec2 direction, float minShare = 0.0f);
    void tick(float dt, const JumpSettings& s);

    JumpStage stage() const { return stage_; }
    bool active() const { return stage_ != JumpStage::None; }
    bool loading() const { return stage_ == JumpStage::Load && !released_; }
    bool airborne() const { return stage_ == JumpStage::Flight; }
    // s left until landing (in flight), else 0.
    float flightTime() const { return flightTime_; }
    float timeToLand() const { return airborne() ? std::max(flightTime_ - time_, 0.0f) : 0.0f; }

    float bodyY() const { return bodyY_; }  // m pelvis offset from standing
    float feetY() const { return feetY_; }  // m both feet above the ground
    // Horizontal velocity, character space (x = left, y = forward), m/s.
    math::Vec2 velocity() const { return velocity_; }
    // Lean, radians: x = forward, y = toward the right (as the walk's lean).
    math::Vec2 lean() const { return lean_; }
    // Arm poses, and how much they replace the walk's arm swing (0..1).
    const JumpArm& arm(bool left) const { return arms_[left ? 0 : 1]; }
    float armWeight() const { return armWeight_; }

private:
    void armTargets(const JumpSettings& s);  // take-off and landing poses from the direction

    JumpStage stage_ = JumpStage::None;
    bool released_ = false;
    bool moving_ = false, driveLeft_ = false;
    float speedShare_ = 0.0f;
    float time_ = 0.0f;   // s in the current stage
    float load_ = 0.0f;   // 0..1 crouch share
    float minShare_ = 0.0f;  // load share the take-off waits for (release)
    float share_ = 0.0f;  // load share the take-off used
    float flightTime_ = 0.0f, upSpeed_ = 0.0f;
    float fromY_ = 0.0f;  // body offset when the stage began
    math::Vec2 direction_{0.0f}, travel_{0.0f};  // travel: m over the flight
    float bodyY_ = 0.0f, feetY_ = 0.0f;
    math::Vec2 velocity_{0.0f}, lean_{0.0f};
    JumpArm arms_[2];                       // now [left, right]
    JumpArm from_[2], top_[2], land_[2];    // stage start, take-off, landing
    float armWeight_ = 0.0f;
};

} // namespace motion
