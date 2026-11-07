#pragma once
// The body's balancer (DESIGN "Physics body and balance"; DECISIONS
// 2026-11-06, 2026-11-07 "One centre-of-mass controller"): one controller of
// the centre of mass for every stance, no per-case joints.
//  1. The support: the soles of the foot parts that touch the ground (Jolt's
//     contacts), their convex hull seen from above.
//  2. Where to press (the centre of pressure p): Peterka 2002's measured
//     ankle feedback T = KP theta + KD theta' written as the pressure point it
//     makes (statics: T = m g (p - aim)): p = aim + (KP (x - aim) + KD v) /
//     (m g h); KD/KP ~ 1/omega0, so it acts on the capture point
//     xi = x + v / omega0 (Hof 2008). p is kept inside the support.
//  3. The ground force: the inverted pendulum's (LIPM, Kajita 2003):
//     m omega0^2 (x - p) across, m g up; it passes through the centre of mass.
//  4. Two feet share it by where p lies between their ankles (each its own
//     pressure point inside its own foot): side to side the weight goes from
//     one leg to the other (Winter 1995's load/unload).
//  5. Joint torques through each stance leg (J^T, statics): at its ankle,
//     knee and hip, -(p_i - joint) x F_i minus the weight of the leg below the
//     joint (the stance legs' support torque, Coros et al. 2010), no harder
//     than the joint's strength, as equal and opposite torques on the two parts.
// The axes the pressure moves the body with are the balancer's: the stance
// ankles' plantar/dorsiflexion and inversion, and on two feet the hips'
// ab/adduction (the parallelogram over the feet). The ankle's torque goes
// through its Jolt springs, solved with the foot's contacts (slope Peterka's
// KP, aim set to give the torque now; an added torque on the light foot made
// it chatter and spin on the ground); the hips' axes keep only the passive
// damping B. The other axes stay their muscles' (holding their angle); the
// knee and hip torques ride on top as added torques.
// While a foot is meant to lift but still carries, the aim is the other ankle:
// the weight goes over it first (the anticipatory shift, BIOMECHANICS 3.1),
// then the foot may lift (liftReady) and carries nothing more.
// Not yet: the hip strategy (p held at the support's edge), stepping, release
// to free fall.
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
};
// Reads "balance" from human_joints.json (N m/deg there).
BalanceSettings readBalanceSettings(const core::Config& joints);

class Balancer {
public:
    // `foot` and `toe`: the rig joints of the left and right foot (its joint is
    // the ankle) and toe. False if one has no part of its own.
    bool init(const HumanBody& body, const int foot[2], const int toe[2], const BalanceSettings& s);

    // Before the decider, HumanBody::compensateGravity and drive: which feet
    // stand (touching the ground), the support, the pressure point, the
    // ground force and each foot's share. `down`: the feet meant to stay
    // down; a foot meant to lift that still touches carries on while the
    // weight shifts over the other (liftReady). No allocations.
    void plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2]);
    // The foot may leave the ground now: it is off already, or the pressure
    // the feedback asks for lies inside the other foot (the shift is done:
    // that foot alone can hold the body).
    bool liftReady(int foot) const { return liftReady_[foot & 1]; }
    // After drive: the balancer's axes to passive damping, the stance legs'
    // torques on.
    void apply(physics::PhysicsWorld& world, const HumanBody& body) const;

    // Per part: non-zero where the balancer carries the joint's load (the
    // stance legs; pass to compensateGravity).
    const uint8_t* owned() const { return owned_.data(); }
    // Per part: non-zero where the balancer drives an axis of its joint this
    // step (pass to HumanBody::drive: a joint let go holds its angle then).
    const uint8_t* driven() const { return driven_.data(); }
    bool standing() const { return stanceCount_ > 0; }
    bool footDown(int foot) const { return stance_[foot & 1]; }  // carrying: touching the ground, not lifting (last plan)
    math::Vec3 supportCentre() const { return centre_; }   // world, mean of the soles' corners
    math::Vec3 pivot() const { return aim_; }              // world: where the centre of mass is kept (ground height)
    math::Vec3 capturePoint() const { return capture_; }   // world, at the ground's height
    math::Vec3 pressurePoint() const { return pressure_; } // world: the centre of pressure asked for (inside the support)
    math::Vec3 groundForce() const { return force_; }      // N, on the body from the ground (all stance feet)
    float footShare(int foot) const { return share_[foot & 1]; }  // of the ground force

private:
    static constexpr int kPointsPerFoot = 16;  // foot and toe soles (or contacts)
    // A convex polygon seen from above (x, z), at most kHullPoints corners.
    static constexpr int kHullPoints = 2 * kPointsPerFoot + 1;
    struct Hull {
        math::Vec2 corner[kHullPoints];
        int count = 0;
        void build(const math::Vec3* points, int n);  // Andrew's monotone chain
        bool contains(math::Vec2 q) const;
        math::Vec2 closest(math::Vec2 q) const;  // q if inside, else the nearest point on the edge
    };
    BalanceSettings settings_;
    int footPart_[2] = {-1, -1}, toePart_[2] = {-1, -1};
    int shinPart_[2] = {-1, -1}, thighPart_[2] = {-1, -1};
    // Joint-frame axes the balancer drives (-1: a hinge): ankle
    // plantar/dorsiflexion and inversion, hip ab/adduction.
    int ankleAheadAxis_[2] = {-1, -1}, ankleSideAxis_[2] = {-1, -1}, hipSideAxis_[2] = {-1, -1};
    std::vector<uint8_t> owned_, driven_;
    bool stance_[2] = {false, false};
    bool liftReady_[2] = {false, false};
    bool lifting_[2] = {false, false};  // a lift under way (until the foot is meant down)
    math::Vec3 footPoint_[2][kPointsPerFoot];
    int footPoints_[2] = {0, 0};
    Hull footHull_[2];
    int stanceCount_ = 0;
    math::Vec3 centre_{0.0f}, aim_{0.0f}, capture_{0.0f}, pressure_{0.0f}, force_{0.0f};
    float share_[2] = {0.0f, 0.0f};
    math::Vec3 footPressure_[2] = {math::Vec3(0.0f), math::Vec3(0.0f)};  // each stance foot's own pressure point (world)
};

} // namespace agents
