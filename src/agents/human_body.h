#pragma once
// The human body's mass in the shared physics world (BIOMECHANICS 13;
// DECISIONS 2026-11-04): one Jolt body at the centre of mass, pulled by the
// world's gravity, colliding with the world's ground like every other body.
// Nothing holds it up yet: in a real body the legs' bones and muscles do,
// and those come with the ragdoll (one body per segment, muscle motors,
// balance). Until then the mass falls until the ground stops it.
// Vertical only for now. Load-time allocation only.

#include "physics/core/physics_world.h"
#include "math/math.h"

namespace agents {

class HumanBody {
public:
    // Creates the body: `mass` kg at `centreOfMass` (world). False if the
    // world is full.
    bool init(physics::PhysicsWorld& world, float mass, math::Vec3 centreOfMass);
    bool ready() const { return body_ != physics::kInvalidBody; }

    // Centre-of-mass height (world) after the last step.
    float height(const physics::PhysicsWorld& world) const;
    float verticalSpeed(const physics::PhysicsWorld& world) const;

private:
    physics::BodyId body_ = physics::kInvalidBody;
};

} // namespace agents
