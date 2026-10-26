#include "match/wickets.h"

#include "core/config.h"

namespace match {

WicketLayout makeWicketLayout(const core::Config& scene, const core::Config& physics) {
    WicketLayout l;
    l.stumpHeight = scene.getFloat("stumps.height", l.stumpHeight);
    l.stumpRadius = scene.getFloat("stumps.radius", l.stumpRadius);
    l.bailLength = scene.getFloat("bails.length", l.bailLength);
    l.bailRadius = scene.getFloat("bails.radius", l.bailRadius);
    float wicketWidth = scene.getFloat("stumps.wicketWidth", 0.2286f);
    float halfPitch = 0.5f * scene.getFloat("pitch.length", 20.12f);
    float bailGap = physics.getFloat("wicket.bailGap", 0.001f);  // between the two bails' inner ends

    float spacing = 0.5f * wicketWidth - l.stumpRadius;  // outer stump centre from the middle stump
    math::Quat alongX = math::fromAxisAngle(math::Vec3(0, 0, 1), -math::kHalfPi);
    float bailX = 0.5f * (l.bailLength + bailGap);
    for (int end = 0; end < 2; ++end) {
        float z = (end == 0 ? 1.0f : -1.0f) * halfPitch;
        int base = end * kPiecesPerWicket;
        for (int s = 0; s < kStumpsPerWicket; ++s) {
            math::Transform& t = l.rest[static_cast<size_t>(base + s)];
            t.position = math::Vec3(static_cast<float>(s - 1) * spacing, 0.5f * l.stumpHeight, z);
        }
        for (int b = 0; b < kBailsPerWicket; ++b) {
            math::Transform& t = l.rest[static_cast<size_t>(base + kStumpsPerWicket + b)];
            t.position = math::Vec3((b == 0 ? -1.0f : 1.0f) * bailX, l.stumpHeight + l.bailRadius, z);
            t.rotation = alongX;
        }
    }
    return l;
}

bool Wickets::build(physics::PhysicsWorld& world, const core::Config& scene, const core::Config& physics) {
    destroy(world);
    layout_ = makeWicketLayout(scene, physics);

    physics::BodyDesc d;
    d.shape = physics::ShapeType::Cylinder;
    d.friction = physics.getFloat("wicket.friction", 0.5f);
    d.restitution = physics.getFloat("wicket.restitution", 0.3f);
    d.convexRadius = physics.getFloat("wicket.convexRadius", 0.002f);
    float stumpMass = physics.getFloat("wicket.stumpMass", 0.6f);
    float bailMass = physics.getFloat("wicket.bailMass", 0.025f);
    bodies_.fill(physics::kInvalidBody);
    built_ = true;
    for (int i = 0; i < kWicketPieces; ++i) {
        bool bail = layout_.isBail(i);
        d.radius = layout_.radius(i);
        d.halfHeight = layout_.halfLength(i);
        d.mass = bail ? bailMass : stumpMass;
        d.pose = layout_.rest[static_cast<size_t>(i)];
        bodies_[static_cast<size_t>(i)] = world.createBody(d);
        if (bodies_[static_cast<size_t>(i)] == physics::kInvalidBody) {
            destroy(world);
            return false;
        }
    }
    return true;
}

void Wickets::destroy(physics::PhysicsWorld& world) {
    if (!built_) return;
    for (physics::BodyId& id : bodies_) {
        world.destroyBody(id);
        id = physics::kInvalidBody;
    }
    built_ = false;
}

void Wickets::reset(physics::PhysicsWorld& world) const {
    if (!built_) return;
    for (int i = 0; i < kWicketPieces; ++i) world.resetBody(body(i), layout_.rest[static_cast<size_t>(i)]);
}

} // namespace match
