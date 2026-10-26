#pragma once
// Thin wrapper over Jolt Physics for generic rigid bodies (later: stumps,
// bails, player bodies). Game code sees only this interface; Jolt stays in the .cpp.
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

// config/physics_world.json "world": shared by Jolt and the ball solver.
struct WorldSettings {
    math::Vec3 gravity{0.0f, -9.81f, 0.0f};
    float airDensity = 1.225f;  // kg/m^3
    math::Vec3 wind{0.0f};      // m/s
};

// config/physics_world.json "jolt".
struct PhysicsSettings {
    int maxBodies = 256;
    int maxBodyPairs = 1024;
    int maxContactConstraints = 1024;
    int tempAllocatorKb = 2048;
    int threads = 0;  // 0: single-threaded job system
    int collisionSteps = 1;
    float penetrationSlop = 0.001f;
    float speculativeContactDistance = 0.005f;
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
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
    bool dynamic = true;
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

    // Creates the Jolt system (no bodies yet).
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
    bool awake(BodyId id) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    WorldSettings world_;
};

} // namespace physics
