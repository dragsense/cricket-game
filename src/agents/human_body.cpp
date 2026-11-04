#include "agents/human_body.h"

namespace agents {

bool HumanBody::init(physics::PhysicsWorld& world, float mass, math::Vec3 centreOfMass) {
    physics::BodyDesc d;
    d.shape = physics::ShapeType::Sphere;
    d.radius = 0.05f;
    // Up and down only for now (no sliding, no tipping): travel still comes
    // from the walk.
    d.verticalOnly = true;
    // `collides` stays on (default): when it falls, Jolt's ground stops it.
    d.mass = mass;
    d.pose.position = centreOfMass;
    body_ = world.createBody(d);
    return body_ != physics::kInvalidBody;
}

float HumanBody::height(const physics::PhysicsWorld& world) const {
    return ready() ? world.transform(body_).position.y : 0.0f;
}

float HumanBody::verticalSpeed(const physics::PhysicsWorld& world) const {
    return ready() ? world.linearVelocity(body_).y : 0.0f;
}

} // namespace agents
