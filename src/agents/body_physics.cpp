#include "agents/body_physics.h"

#include <cmath>

namespace agents {

bool BodyPhysics::init(physics::PhysicsWorld& world, float mass, math::Vec3 centreOfMass, const float restLeg[2],
                       const BodyPhysicsSettings& s) {
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.05f;
    d.collides = false;  // the legs hold it, not contacts
    d.verticalOnly = true;
    d.mass = mass;
    d.pose.position = centreOfMass;
    body_ = world.createBody(d);
    if (body_ == physics::kInvalidBody) return false;
    start_ = centreOfMass;

    // Jolt's spring frequency for stiffness k on mass m: f = sqrt(k / m) / 2 pi.
    const float frequency = std::sqrt(s.legStiffness / mass) / math::kTwoPi;
    physics::BodyDesc a;
    a.shape = physics::ShapeType::Sphere;
    a.radius = 0.01f;
    a.kinematic = true;
    a.collides = false;
    for (int i = 0; i < 2; ++i) {
        a.pose.position = centreOfMass - math::kUp * restLeg[i];
        anchor_[i] = world.createBody(a);
        if (anchor_[i] == physics::kInvalidBody) return false;
        // Push only: slack when longer than the rest distance (a leg cannot pull).
        leg_[i] = world.createSpring(anchor_[i], math::Vec3(0.0f), body_, math::Vec3(0.0f), restLeg[i], 100.0f,
                                     frequency, s.legDamping);
        if (leg_[i] == physics::kInvalidSpring) return false;
    }
    return true;
}

void BodyPhysics::setFeet(physics::PhysicsWorld& world, math::Vec3 bodyRef, const math::Vec3 ankles[2],
                          const bool down[2], float dt) {
    if (!ready()) return;
    for (int i = 0; i < 2; ++i) {
        // Same offset from the body as the foot has from bodyRef sideways and
        // fore-aft; the foot's own height.
        const math::Vec3 offset = ankles[i] - bodyRef;
        world.moveKinematic(anchor_[i], math::Vec3(start_.x + offset.x, ankles[i].y, start_.z + offset.z), dt);
        world.setSpringEnabled(leg_[i], down[i]);
    }
}

float BodyPhysics::height(const physics::PhysicsWorld& world) const {
    return ready() ? world.transform(body_).position.y : 0.0f;
}

float BodyPhysics::verticalSpeed(const physics::PhysicsWorld& world) const {
    return ready() ? world.linearVelocity(body_).y : 0.0f;
}

} // namespace agents
