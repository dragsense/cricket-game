#pragma once
// Our own solver for a sphere body (the ball; any other small object we
// simulate ourselves): sums the force generators, integrates with
// semi-implicit Euler in substeps short enough that a fast body never skips
// through a collider, and bounces off boxes and spheres (normal restitution,
// no friction yet). step() is a pure function of the state, so a predictor
// (Step 3.4) runs the exact same code. Jolt handles rigid bodies.

#include "physics/sim/colliders.h"
#include "physics/sim/forces.h"

#include <array>

namespace physics {

struct SolverSettings {
    float maxTravelPerSubstep = 0.5f;  // in body radii
    int maxSubsteps = 64;
    float restSpeed = 0.05f;  // m/s; slower normal bounces are absorbed (body settles)
};

class SphereSolver {
public:
    static constexpr int kMaxForces = 8;

    void configure(const SphereBody& body, const WorldSettings& world, const SolverSettings& settings) {
        body_ = body;
        world_ = world;
        settings_ = settings;
    }

    // Generators are not owned and must outlive the solver. Load time.
    bool addForce(const ForceGenerator* force);
    void clearForces() { forceCount_ = 0; }

    // Advances `s` by dt (one sim tick). No allocations.
    void step(BodyState& s, float dt) const;
    // True when the body is touching a collider (within a small skin).
    bool touching(const BodyState& s) const;

    const SphereBody& body() const { return body_; }
    const WorldSettings& world() const { return world_; }
    const SolverSettings& settings() const { return settings_; }
    ColliderSet& colliders() { return colliders_; }
    const ColliderSet& colliders() const { return colliders_; }

private:
    math::Vec3 acceleration(const BodyState& s) const;
    void resolveContacts(BodyState& s) const;

    SphereBody body_;
    WorldSettings world_;
    SolverSettings settings_;
    ColliderSet colliders_;
    std::array<const ForceGenerator*, kMaxForces> forces_{};
    int forceCount_ = 0;
};

} // namespace physics
