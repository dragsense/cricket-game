#pragma once
// Standing jump: countermovement (crouch, arms back), push-off (legs extend,
// arms swing up), ballistic flight (the body rises and falls under gravity,
// the feet tuck), landing (knees absorb) and recovery to standing. Straight
// up, or toward any direction in character space (left, right, forward,
// back). Holding the load deepens the crouch; the take-off speed grows with
// it. Outputs offsets for the body and feet; the fielder applies them.
// Tunables in config/locomotion.json "jump". Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct JumpSettings {
    float crouchDepth = 0.18f;     // m the pelvis drops at the deepest load
    float loadTime = 0.35f;        // s to the deepest load while held
    float minLoad = 0.4f;          // share of the deepest load a tap gives
    float crouchLean = 0.35f;      // rad forward trunk lean at the deepest load (hips flex)
    float pushTime = 0.16f;        // s, legs extend from the crouch to take-off
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
    float armBack = 0.9f;          // rad shoulder extension at the deepest load
    float armUp = 2.3f;            // rad shoulder flexion at take-off
    float gravity = 9.81f;         // m/s^2
};

enum class JumpStage { None, Load, Push, Flight, Land };

class JumpController {
public:
    // Begins the crouch (Load) if not already jumping.
    void start();
    // Ends the load: takes off toward `direction` (character space, x = left,
    // y = forward; zero = straight up, length clamped to 1) once at least
    // minLoad deep.
    void release(math::Vec2 direction);
    void tick(float dt, const JumpSettings& s);

    JumpStage stage() const { return stage_; }
    bool active() const { return stage_ != JumpStage::None; }
    bool loading() const { return stage_ == JumpStage::Load && !released_; }
    bool airborne() const { return stage_ == JumpStage::Flight; }

    float bodyY() const { return bodyY_; }  // m pelvis offset from standing
    float feetY() const { return feetY_; }  // m both feet above the ground
    // Horizontal velocity, character space (x = left, y = forward), m/s.
    math::Vec2 velocity() const { return velocity_; }
    // Lean, radians: x = forward, y = toward the right (as the walk's lean).
    math::Vec2 lean() const { return lean_; }
    // Arms: -1 fully back (load), 0 hanging, +1 fully up (take-off).
    float arms() const { return arms_; }

private:
    JumpStage stage_ = JumpStage::None;
    bool released_ = false;
    float time_ = 0.0f;   // s in the current stage
    float load_ = 0.0f;   // 0..1 crouch share
    float share_ = 0.0f;  // load share the take-off used
    float flightTime_ = 0.0f, upSpeed_ = 0.0f;
    float fromY_ = 0.0f, fromArms_ = 0.0f;  // values when the stage began
    math::Vec2 direction_{0.0f}, travel_{0.0f};  // travel: m over the flight
    float bodyY_ = 0.0f, feetY_ = 0.0f, arms_ = 0.0f;
    math::Vec2 velocity_{0.0f}, lean_{0.0f};
};

} // namespace motion
