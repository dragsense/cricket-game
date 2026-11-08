#pragma once
// The body's balancer by visible assists (the user's call, DECISIONS
// 2026-11-08 "Balance by visible assists"): outside forces stand the physics
// body; the muscles only shape it. All scaled by one balance strength (0..1).
//  1. The support: the soles of the foot parts touching the ground (Jolt's
//     contacts), their convex hull seen from above; the capture point
//     xi = x + v / omega0 (Hof 2008) against it.
//  2. Hover: an upward spring-damper force on the hips toward the stand
//     height, carrying a share of the weight (the feet press the rest, so they
//     keep their friction). Only while a foot touches the ground.
//  3. Upright: a torque on the pelvis tilting its up toward the plan's pelvis
//     up (e.g. over a stance hip), sized by the body's inertia about its
//     centre of mass; its turn about up stays free.
//  4. Pull: a horizontal force on the hips bringing the centre of mass over
//     the stance ankles (over the other ankle while one is meant to lift).
//  5. Foot lock: a planted foot is held to the ground by a locked Jolt joint to
//     a hidden kinematic anchor (perfect grip, no sliding or rolling), let go
//     while it lifts or the strength is low. Its ankle is left free (free()):
//     the body pivots over it and the pull places it (a held ankle on a
//     locked foot made the body a rigid statue the pull could not move).
//  6. Step: the capture point outside the support on two feet: a step for
//     the foot that gets there, placed past the capture point (Human moves it).
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

// config/human_joints.json "balance". Feel tunables, not body values.
struct BalanceSettings {
    float hoverShare = 0.7f, hoverMaxShare = 0.95f;  // of the body's weight
    float hoverFrequency = 2.0f, hoverDamping = 1.0f;  // Hz, ratio
    float uprightFrequency = 2.0f, uprightDamping = 1.0f, uprightMaxTorque = 600.0f;
    float pullFrequency = 1.0f, pullDamping = 1.0f, pullMaxForce = 250.0f;
    float footFriction = 1.0f, footRestitution = 0.0f;
    float lockMinStrength = 0.5f;
    float shiftTime = 0.6f;  // s
    float stepMargin = 0.05f, stepMaxLength = 0.6f, stepMinWidth = 0.12f, stepClearance = 0.08f;  // m
    float recoverRate = 1.0f;  // 1/s
};
BalanceSettings readBalanceSettings(const core::Config& joints);

class Balancer {
public:
    // `foot` and `toe`: the rig joints of the left and right foot (its joint is
    // the ankle) and toe. Sets the feet's material and makes their locks
    // (two anchor bodies and joints). The body must stand at rest (its stand
    // height is measured). False if a foot has no part or the world is full.
    bool init(physics::PhysicsWorld& world, const HumanBody& body, const int foot[2], const int toe[2],
              const BalanceSettings& s);

    // Before the decider: the support, the capture point, which feet carry,
    // lifts and the step wanted. `down`: the feet meant to stay down; one
    // meant to lift waits (at most shiftTime) for the weight to go over the
    // other. `dt`: this step (s). No allocations.
    void plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2], float dt);
    // The plan's pelvis bone rotation (world; only its up is used): set before
    // apply each step.
    void setPelvisTarget(math::Quat rotation) { pelvisTarget_ = rotation; }
    // After the muscles: the assists' forces and the feet's locks.
    void apply(physics::PhysicsWorld& world, const HumanBody& body);
    // Physics off or the body reset: the locks let go, the lifts forgotten.
    void release(physics::PhysicsWorld& world);

    // A hit: the strength drops by `amount` (0..1) and climbs back at recoverRate.
    void knock(float amount);
    float strength() const { return strength_; }
    const BalanceSettings& settings() const { return settings_; }

    // The foot may leave the ground now (it is off already, or the weight is over the other).
    bool liftReady(int foot) const { return liftReady_[foot & 1]; }
    bool standing() const { return stanceCount_ > 0; }
    // Balance lost: nothing to stand on, or the capture point outside the
    // support (a step is needed: Hof et al. 2005's margin of stability < 0).
    bool lost() const { return lost_; }
    bool footDown(int foot) const { return stance_[foot & 1]; }  // carrying (last plan)
    bool footLocked(int foot) const { return locked_[foot & 1]; }
    // Per part: non-zero for a joint left free this step (a locked foot's
    // ankle; pass to HumanBody::drive). Set by plan.
    const uint8_t* free() const { return free_.data(); }
    // A step wanted (two feet down, capture point outside): which foot and
    // where its ankle goes (world). False if none.
    bool stepWanted(int& foot, math::Vec3& target) const {
        foot = stepFoot_;
        target = stepTarget_;
        return stepFoot_ >= 0;
    }
    math::Vec3 pivot() const { return aim_; }              // world, ground height: where the centre of mass is pulled
    math::Vec3 capturePoint() const { return capture_; }   // world, ground height
    // The assists' last push (world), for drawing.
    math::Vec3 hoverForce() const { return hover_; }       // N, on the hips
    math::Vec3 pullForce() const { return pull_; }         // N, on the hips
    math::Vec3 uprightTorque() const { return upright_; }  // N m, on the pelvis
    math::Vec3 footAnchor(int foot) const { return anchorAt_[foot & 1]; }  // world, where a locked foot is held

private:
    static constexpr int kPointsPerFoot = 16;
    static constexpr int kHullPoints = 2 * kPointsPerFoot + 1;
    // A convex polygon seen from above (x, z), at most kHullPoints corners.
    struct Hull {
        math::Vec2 corner[kHullPoints];
        int count = 0;
        void build(const math::Vec3* points, int n);  // Andrew's monotone chain
        bool contains(math::Vec2 q) const;
    };
    BalanceSettings settings_;
    int footPart_[2] = {-1, -1}, toePart_[2] = {-1, -1};
    physics::BodyId anchor_[2] = {physics::kInvalidBody, physics::kInvalidBody};
    physics::JointId lock_[2] = {physics::kInvalidJoint, physics::kInvalidJoint};
    bool locked_[2] = {false, false};
    std::vector<uint8_t> free_;  // per part
    math::Vec3 anchorAt_[2] = {math::Vec3(0.0f), math::Vec3(0.0f)};
    float standHeight_ = 0.9f;  // m, the hips above the ground at rest
    float strength_ = 1.0f;
    math::Quat pelvisTarget_ = math::identityQuat();
    bool down_[2] = {true, true};
    bool stance_[2] = {false, false};
    bool liftReady_[2] = {false, false};
    bool lifting_[2] = {false, false};  // a lift under way (until the foot is meant down)
    float shiftTimer_[2] = {0.0f, 0.0f};
    bool lost_ = false;
    int stanceCount_ = 0;
    math::Vec3 footPoint_[2][kPointsPerFoot];
    int footPoints_[2] = {0, 0};
    Hull footHull_[2];
    math::Vec3 aim_{0.0f}, capture_{0.0f};
    int stepFoot_ = -1;
    math::Vec3 stepTarget_{0.0f};
    math::Vec3 hover_{0.0f}, pull_{0.0f}, upright_{0.0f};
};

} // namespace agents
