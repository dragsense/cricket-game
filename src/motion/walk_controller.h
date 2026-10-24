#pragma once
// Walk: desired-velocity controller and gait timing. Movement is requested in
// character space (forward, back, sidesteps) and the velocity eases toward it
// (smooth start and stop). Turning is separate: when asked to face a heading,
// the character turns toward it at a limited rate, slowing its forward walk on
// sharp turns. Gait timing
// gives step length and swing/stance durations for a speed, from
// docs/BIOMECHANICS.md section 4 (step = k * H * (v / v_ref)^0.42).
// Tunables in config/locomotion.json "walk". Pure math, no allocations.

#include "math/math.h"

namespace motion {

struct WalkSettings {
    float speed = 1.35f;         // m/s at full forward input
    float backSpeed = 0.9f;      // m/s walking backward
    float sideSpeed = 0.8f;      // m/s sidestepping
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
};

struct GaitTiming {
    float stepLength = 0.0f;  // m
    float period = 0.0f;      // s per step (one foot strike to the next)
    float swing = 0.0f;       // s a foot is in the air
    float stance = 0.0f;      // s a foot is on the ground per stride
};

GaitTiming gaitTiming(float speed, float height, const WalkSettings& s);

class WalkController {
public:
    void reset(float yaw);

    // `move` is the requested movement in character space (x = left, y =
    // forward), length 0..1 scales the speed. When `face` is set the
    // character turns toward world heading `faceYaw` (radians about +Y).
    void tick(float dt, math::Vec2 move, bool face, float faceYaw, const WalkSettings& s);

    float yaw() const { return yaw_; }
    float speed() const { return math::length(local_); }
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
