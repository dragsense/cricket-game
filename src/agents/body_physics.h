#pragma once
// The body's mass in the shared physics world (BIOMECHANICS 12, spring-mass
// model; DECISIONS 2026-11-04): one Jolt body at the centre of mass, pulled
// by the world's gravity, held up by a push-only spring leg from each foot on
// the ground (a kinematic anchor at the foot). A foot in the air holds nothing.
// The body collides with the world's ground like every other body.
// First step: vertical only (the body moves up and down; travel stays with the
// walk). Load-time allocation only.

#include "physics/core/physics_world.h"
#include "math/math.h"

namespace agents {

// config/fielder_joints.json "bodyPhysics".
struct BodyPhysicsSettings {
    float legStiffness = 20000.0f;  // N/m per leg (spring-mass walking, Geyer et al. 2006)
    float legDamping = 0.0f;        // damping ratio (the spring-mass model has none)
};

class BodyPhysics {
public:
    // Creates the body (`mass` kg at `centreOfMass`, world) and a foot anchor
    // and leg spring per leg, slack beyond each foot's rest distance
    // (`restLeg`, centre of mass to ankle at rest). False if the world is full.
    bool init(physics::PhysicsWorld& world, float mass, math::Vec3 centreOfMass, const float restLeg[2],
              const BodyPhysicsSettings& s);
    bool ready() const { return body_ != physics::kInvalidBody; }

    // Before the physics step: each foot's ankle (world) and whether it is on
    // the ground. The anchors keep the feet's offsets from the body sideways
    // and fore-aft (vertical-only body), at the feet's heights.
    void setFeet(physics::PhysicsWorld& world, math::Vec3 bodyRef, const math::Vec3 ankles[2], const bool down[2],
                 float dt);

    // Centre-of-mass height (world) after the last step.
    float height(const physics::PhysicsWorld& world) const;
    float verticalSpeed(const physics::PhysicsWorld& world) const;

private:
    physics::BodyId body_ = physics::kInvalidBody;
    physics::BodyId anchor_[2] = {physics::kInvalidBody, physics::kInvalidBody};
    physics::SpringId leg_[2] = {physics::kInvalidSpring, physics::kInvalidSpring};
    math::Vec3 start_{0.0f};  // the body's (fixed) sideways and fore-aft spot
};

} // namespace agents
