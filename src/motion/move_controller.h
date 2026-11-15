#pragma once
// Move (was the walk controller; the user's name, 2026-11-15): the
// desired velocity that drives the hips target, and gait timing. Walking,
// running and the air are not separate here: the body's decider makes
// them from the speed. Movement is requested in
// character space (forward, back, sidesteps) and the velocity eases toward it
// (smooth start and stop). Turning is separate: when asked to face a heading,
// the character turns toward it at a limited rate, slowing its forward walk on
// sharp turns. Gait timing
// gives step length and swing/stance durations for a speed, from
// docs/BIOMECHANICS.md section 4 (step = k * H * (v / v_ref)^0.42). Running
// uses the same settings with a flight phase (section 5).
// Tunables in config/locomotion.json "move". Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct MoveSettings {
    float speed = 1.35f;         // m/s at full forward input
    float backSpeed = 0.9f;      // m/s walking backward
    float sideSpeed = 0.8f;      // m/s sidestepping
    float jogSpeed = 3.0f, runSpeed = 5.0f;  // m/s forward, jogging (Ctrl) and running (Shift)
    float acceleration = 5.0f;   // m/s^2
    float deceleration = 5.0f;   // m/s^2
    float turnRate = 3.14f;      // rad/s
    float stepK = 0.41f;         // step length = stepK * height at refSpeed
    float refSpeed = 1.35f;      // m/s
    float swingFraction = 0.8f;  // swing time / step period (rest is double support)
    float minSwing = 0.3f;       // s
    float maxSwing = 0.5f;       // s; also the swing of settling steps when stopping
    float stepHeight = 0.08f;    // swing arc apex (m)
    float settleDistance = 0.12f;  // m; when stopped, closer feet stay where they landed
    float bob = 0.025f;          // pelvis vertical bob, peak-to-peak at full speed (m)
    float sway = 0.02f;          // pelvis lateral sway toward the stance foot (m)
    float leanPerSpeed = 0.035f; // forward lean, rad per m/s
    float leanPerAccel = 0.05f;  // forward lean, rad per m/s^2
    float turnLean = 0.05f;      // lean into turns, rad per (rad/s * m/s)
    float maxLean = 0.17f;       // rad
    float leanRate = 6.0f;       // 1/s, lean smoothing
    float armSwing = 0.61f;      // shoulder swing, total range at full forward speed (rad)
    float armBackBias = 0.09f;   // swing centre behind vertical (more extension than flexion)
    float elbowBias = 0.44f;     // mean elbow flexion while walking (rad)
    float elbowSwing = 0.35f;    // extra elbow flexion at the forward end of the swing
    float armRate = 8.0f;        // 1/s, arm smoothing (arms lag slightly)
    float headStabilise = 0.8f;  // 0..1, share of body lean and sway the head cancels
    float headLead = 0.52f;      // max head turn toward a new heading ahead of the body (rad)
    float headRate = 6.0f;       // 1/s, head turn smoothing
    // Running (all 0 / neutral for walking). Use swingFraction 1 with a
    // flight phase (no double support).
    float flightFraction = 0.0f; // share of each swing with both feet off the ground
    float toeOffPitch = 0.0f;    // rad, foot plantarflexion at toe-off
    float landPitch = 0.0f;      // rad, foot pitch at landing (negative = toes up)
    float swingPeakAt = 0.5f;    // swing time share of the arc apex (earlier = heel recovery)
    float pushTime = 0.12f;      // s, push-off before a foot lifts (pre-swing, about 10 % of the stride)
    float liftOffSpeed = 0.0f;   // swing foot's forward speed at lift-off, in shares of its average swing speed
    float widthScale = 1.0f;     // share of standing width the feet land at
    float landAhead = 0.5f;      // share of a foot's ground travel that is ahead of the hips at landing
    float pelvisTwist = 0.0f;    // rad, pelvis turn toward the forward leg
    float chestCounter = 0.0f;   // chest turn against the pelvis, share of pelvisTwist
    float sideFollow = 1.0f;     // share of the sideways shift over the stance foot the pelvis takes
    float pelvisDrop = 0.0f;     // m the pelvis sits lower (stance knee flexes more when running)
};

struct GaitTiming {
    float stepLength = 0.0f;  // m
    float period = 0.0f;      // s per step (one foot strike to the next)
    float swing = 0.0f;       // s a foot is in the air
    float stance = 0.0f;      // s a foot is on the ground per stride
    float flight = 0.0f;      // s both feet are off the ground per step (running)
};

GaitTiming gaitTiming(float speed, float height, const MoveSettings& s);

// Pelvis height over one step, in [-0.5, 0.5], at swing progress t. Walking
// (no flight): lowest in double support, highest mid-swing. Running: lowest
// mid-stance and highest mid-flight (spring-mass); `flightShare` is the
// share of the swing with both feet off the ground.
float gaitBob(float t, float flightShare);

// How far a gait is from walking toward running, [0, 1], by its flight share
// (blends walk-only and run-only behaviour so pace changes never snap).
float runShare(float flightShare);

class MoveController {
public:
    void reset(float yaw);
    // Sets the velocity at once (character space, x = left, y = forward), e.g.
    // momentum handed back after a pause; it then eases as usual.
    void setLocalVelocity(math::Vec2 v) {
        local_ = v;
        gaitSpeed_ = math::length(v);
    }

    // `move` is the requested movement in character space (x = left, y =
    // forward), length 0..1 scales the speed. When `face` is set the
    // character turns toward world heading `faceYaw` (radians about +Y).
    void tick(float dt, math::Vec2 move, bool face, float faceYaw, const MoveSettings& s);

    float yaw() const { return yaw_; }
    float speed() const { return math::length(local_); }
    // Velocity in character space (x = left, y = forward).
    math::Vec2 localVelocity() const { return local_; }
    // Speed being asked for (eases toward it); kept while stopping so the last
    // steps keep the walk's rhythm. Drives gait timing, so steps are full size
    // from the first one.
    float gaitSpeed() const { return gaitSpeed_; }
    // Forward acceleration (m/s^2, negative when braking or backing up).
    float acceleration() const { return accel_; }
    float yawRate() const { return yawRate_; }
    // World-space velocity (forward, back or sideways relative to the facing).
    math::Vec3 velocity() const;

private:
    float yaw_ = 0.0f;
    math::Vec2 local_{0.0f};  // velocity in character space (x = left, y = forward)
    float accel_ = 0.0f;
    float gaitSpeed_ = 0.0f;
    float yawRate_ = 0.0f;
};

} // namespace motion
