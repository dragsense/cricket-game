#pragma once
// Force generators for our own sphere bodies (the ball, and any other small
// object we simulate ourselves). A solver sums every registered generator each
// substep, so new forces (spin, swing, ...) are new generators; the solver
// does not change.

#include "math/math.h"
#include "physics/core/physics_world.h"

namespace physics {

struct BodyState {
    math::Vec3 position{0.0f};         // centre, world space
    math::Vec3 velocity{0.0f};         // m/s
    math::Vec3 angularVelocity{0.0f};  // rad/s, world space
    math::Quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
};

struct SphereBody {
    float mass = 1.0f;              // kg
    float radius = 0.1f;            // m
    float dragCoefficient = 0.47f;  // C_d (smooth sphere)
    float inertiaFactor = 0.4f;     // I = k m r^2 (0.4 solid sphere)
    float area() const { return math::kPi * radius * radius; }
};

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    // Force in newtons on the body in this state.
    virtual math::Vec3 force(const BodyState& s, const SphereBody& body, const WorldSettings& world) const = 0;
};

// m g, with g from the shared world settings.
class GravityForce final : public ForceGenerator {
public:
    math::Vec3 force(const BodyState& s, const SphereBody& body, const WorldSettings& world) const override;
};

// Quadratic air drag: -1/2 rho C_d A |v_rel| v_rel, v_rel = velocity - wind.
class DragForce final : public ForceGenerator {
public:
    math::Vec3 force(const BodyState& s, const SphereBody& body, const WorldSettings& world) const override;
};

} // namespace physics
