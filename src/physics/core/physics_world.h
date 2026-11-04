#pragma once
// Thin wrapper over Jolt Physics for generic rigid bodies (later: player
// bodies, bat). Game code sees only this interface; Jolt stays in the .cpp.
// Stepped by the fixed-timestep sim; keeps the previous and current transform
// of every body so rendering can interpolate. World settings (gravity, air)
// are shared with the ball solver.

#include "math/math.h"

#include <cstdint>
#include <memory>

namespace core {
class Config;
}

namespace physics {

using BodyId = uint32_t;
constexpr BodyId kInvalidBody = 0xffffffffu;
using SpringId = uint32_t;
constexpr SpringId kInvalidSpring = 0xffffffffu;

// config/physics_world.json "world": shared by Jolt and the ball solver.
struct WorldSettings {
    math::Vec3 gravity{0.0f, -9.81f, 0.0f};
    float airDensity = 1.225f;  // kg/m^3
    math::Vec3 wind{0.0f};      // m/s
    float groundHeight = 0.0f;  // m, top of the ground: the one ground everything stays above
};

// config/physics_world.json "jolt" and "ground".
struct PhysicsSettings {
    int maxBodies = 256;
    int maxBodyPairs = 1024;
    int maxContactConstraints = 1024;
    int maxSprings = 32;
    int tempAllocatorKb = 2048;
    int threads = 0;  // 0: single-threaded job system
    int collisionSteps = 1;
    float penetrationSlop = 0.001f;
    float speculativeContactDistance = 0.005f;
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
    float groundFriction = 0.6f;
    float groundRestitution = 0.2f;
};

WorldSettings readWorldSettings(const core::Config& c);
PhysicsSettings readPhysicsSettings(const core::Config& c);

enum class ShapeType { Box, Cylinder, Sphere };

struct BodyDesc {
    ShapeType shape = ShapeType::Box;
    math::Vec3 halfExtents{0.5f};  // box
    float radius = 0.5f;           // cylinder, sphere
    float halfHeight = 0.5f;       // cylinder (along local +Y)
    float convexRadius = 0.002f;   // rounding of box and cylinder edges
    math::Transform pose;          // centre of mass, world space
    bool dynamic = true;        // false: static (unless kinematic)
    bool kinematic = false;     // moved only by moveKinematic (no gravity, nothing pushes it)
    bool collides = true;       // false: touches nothing (e.g. a mass held only by springs)
    bool lockRotation = false;  // dynamic: translates only (no tumbling)
    bool verticalOnly = false;  // dynamic: moves up and down only (no rotation either)
    float mass = 1.0f;  // kg; dynamic only
    float friction = 0.5f;
    float restitution = 0.3f;
};

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // Creates the Jolt system and a static ground whose top is y = 0.
    bool init(const WorldSettings& world, const PhysicsSettings& settings);
    void shutdown();
    bool ready() const { return impl_ != nullptr; }

    const WorldSettings& world() const { return world_; }

    // Load time only (allocates). Returns kInvalidBody when full.
    BodyId createBody(const BodyDesc& desc);
    void destroyBody(BodyId id);

    // One fixed tick. No heap allocations.
    void step(float dt);

    math::Transform transform(BodyId id) const;
    // Blend of the previous and current tick, for rendering.
    math::Transform interpolated(BodyId id, float alpha) const;
    math::Vec3 linearVelocity(BodyId id) const;
    // Teleports the body (velocity zero), for resets; no interpolation smear.
    void resetBody(BodyId id, const math::Transform& pose);
    void addImpulse(BodyId id, math::Vec3 impulse, math::Vec3 point);
    // Kinematic body: reaches `position` at the end of the next step (dt s).
    void moveKinematic(BodyId id, math::Vec3 position, float dt);

    // Springs: Jolt distance constraints with soft limits, between a point on
    // body a and a point on body b (local, from each body's centre of mass).
    // Inside [minDistance, maxDistance] the spring is slack; outside it, it
    // pushes or pulls back at `frequency` Hz with `damping` ratio (Jolt
    // SpringSettings). Load time only (allocates); kInvalidSpring when full.
    SpringId createSpring(BodyId a, math::Vec3 localA, BodyId b, math::Vec3 localB, float minDistance,
                          float maxDistance, float frequency, float damping);
    void setSpringDistance(SpringId id, float minDistance, float maxDistance);
    void setSpringEnabled(SpringId id, bool enabled);

    // The ground (Jolt's static scenery) under `point`: its height there, or
    // the world's groundHeight if nothing is below.
    float groundHeightAt(math::Vec3 point) const;
    // `point` raised, if needed, so it stays `clearance` m above the ground.
    // The one check every mover that is not a colliding body uses.
    math::Vec3 aboveGround(math::Vec3 point, float clearance = 0.0f) const;
    bool awake(BodyId id) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    WorldSettings world_;
};

} // namespace physics
