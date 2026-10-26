#include "physics/sim/forces.h"

namespace physics {

math::Vec3 GravityForce::force(const BodyState&, const SphereBody& body, const WorldSettings& world) const {
    return body.mass * world.gravity;
}

math::Vec3 DragForce::force(const BodyState& s, const SphereBody& body, const WorldSettings& world) const {
    const math::Vec3 rel = s.velocity - world.wind;
    return -0.5f * world.airDensity * body.dragCoefficient * body.area() * math::length(rel) * rel;
}

} // namespace physics
