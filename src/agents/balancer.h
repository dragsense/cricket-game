#pragma once
// The body's balancer (DESIGN "Physics body and balance"; DECISIONS
// 2026-11-06). Step 3a: standing, ankle strategy. The support is whatever of
// the stance feet touches the ground (Jolt's contacts); the stance ankles'
// muscles are the balancer's: their torque is the measured human ankle
// feedback T = -(KP sway + KD sway rate) (Peterka 2002), the sway being the
// body's angle about the stance ankles (the inverted pendulum's pivot); with KD/KP
// about 1/omega0 it acts on the capture point (Hof 2008). No outside forces:
// each balance axis is a spring of those gains on its joint (Jolt's
// stiffness-and-damping motor, solved with the contacts), its rest where the
// sway is zero.
// The ankles carry the sway ahead and back; side to side the hips load and
// unload the legs (Winter 1995): on two feet the legs and pelvis turn as a
// parallelogram, the hips' ab/adduction driving it and the ankles' inversion
// free; on one foot its ankle's inversion carries it. The balancer owns only
// those axes; the others stay their muscles' (holding their angle). A joint's
// strength caps its torque; a torque a foot cannot carry tips it (Jolt).
// Not yet: the hip strategy, stepping, release to free fall.
// No allocations after init().

#include "agents/human_body.h"
#include "math/math.h"
#include "physics/core/physics_world.h"

#include <cstdint>
#include <vector>

namespace core {
class Config;
}

namespace agents {

// config/human_joints.json "balance" (converted to radians).
struct BalanceSettings {
    float ankleStiffness = 1066.0f;  // KP, N m/rad (Peterka 2002: 18.6 N m/deg)
    float ankleDamping = 332.0f;     // active KD on the body's sway rate, N m s/rad (5.8 N m s/deg)
    float passiveDamping = 24.6f;    // the joint's own damping B, N m s/rad (0.43 N m s/deg)
    float sideStiffness = 1066.0f;   // side to side (hips on two feet), N m/rad
    float sideDamping = 350.0f;      // N m s/rad
};
// Reads "balance" from human_joints.json (N m/deg there).
BalanceSettings readBalanceSettings(const core::Config& joints);

class Balancer {
public:
    // `foot` and `toe`: the rig joints of the left and right foot (its joint is
    // the ankle) and toe. False if one has no part of its own.
    bool init(const HumanBody& body, const int foot[2], const int toe[2], const BalanceSettings& s);

    // Before HumanBody::compensateGravity and drive: which feet stand (meant
    // to be down and touching the ground), the support and the sway torque.
    // No allocations.
    void plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2]);
    // After drive: the stance ankles' plantar/dorsiflexion motor off, the
    // balance torque on.
    void apply(physics::PhysicsWorld& world, const HumanBody& body) const;

    // Per part: non-zero where the balancer carries the joint's load (pass
    // to compensateGravity).
    const uint8_t* owned() const { return owned_.data(); }
    bool standing() const { return stanceCount_ > 0; }
    math::Vec3 supportCentre() const { return centre_; }   // world, mean of the ground contacts
    math::Vec3 pivot() const { return pivot_; }            // world, between the stance ankles
    math::Vec3 capturePoint() const { return capture_; }   // world, at the ground's height
    math::Vec3 ankleTorque() const { return torque_; }     // N m, ahead/back, on the body above the ankles (all stance feet)
    math::Vec3 sideTorque() const { return sideTorque_; }  // N m, side to side (hips on two feet, else the ankle)

private:
    static constexpr int kPointsPerPart = 8;
    BalanceSettings settings_;
    int footPart_[2] = {-1, -1}, toePart_[2] = {-1, -1};
    int thighPart_[2] = {-1, -1};
    // Joint-frame axes the balancer drives (-1: a hinge): ankle
    // plantar/dorsiflexion and inversion, hip ab/adduction.
    int ankleAheadAxis_[2] = {-1, -1}, ankleSideAxis_[2] = {-1, -1}, hipSideAxis_[2] = {-1, -1};
    std::vector<uint8_t> owned_;
    bool stance_[2] = {false, false};
    int stanceCount_ = 0;
    math::Vec3 centre_{0.0f}, pivot_{0.0f}, capture_{0.0f}, torque_{0.0f}, sideTorque_{0.0f};
    math::Vec3 swayAhead_{0.0f}, swaySide_{0.0f};  // the springs' aim (angle vector) ahead/back and side to side
};

} // namespace agents
