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
using JointId = uint32_t;
constexpr JointId kInvalidJoint = 0xffffffffu;

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
    int maxJoints = 256;
    int jointVelocitySteps = 0;  // > 0: solver velocity iterations for joints (Jolt override; 0: the world's)
    int jointPositionSteps = 0;  // > 0: solver position iterations for joints
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

enum class ShapeType { Box, Cylinder, Sphere, Capsule };

struct BodyDesc {
    ShapeType shape = ShapeType::Box;
    math::Vec3 halfExtents{0.5f};  // box
    float radius = 0.5f;           // cylinder, sphere, capsule
    float halfHeight = 0.5f;       // cylinder, capsule: half the straight part (along local +Y)
    float convexRadius = 0.002f;   // rounding of box and cylinder edges
    math::Transform pose;          // centre of mass, world space
    bool dynamic = true;        // false: static (unless kinematic)
    bool kinematic = false;     // moved only by moveKinematic (no gravity, nothing pushes it)
    bool collides = true;       // false: touches nothing (e.g. a mass held only by springs)
    bool lockRotation = false;  // dynamic: translates only (no tumbling)
    bool verticalOnly = false;  // dynamic: moves up and down only (no rotation either)
    float mass = 1.0f;  // kg; dynamic only
    uint32_t collisionGroup = 0;  // > 0: never collides with bodies of the same group (parts of one object)
    float friction = 0.5f;
    float restitution = 0.3f;
};

// A joint between two bodies (Jolt constraints), placed in world space when
// created. Limits in radians.
struct HingeDesc {
    math::Vec3 pivot{0.0f};          // world
    math::Vec3 axis{1.0f, 0.0f, 0.0f};    // world: the bodies turn about it
    math::Vec3 normal{0.0f, 1.0f, 0.0f};  // world, perpendicular to axis: angle 0 is when the bodies keep this as created
    float minAngle = -3.14159265f, maxAngle = 3.14159265f;
};
struct SwingTwistDesc {
    math::Vec3 pivot{0.0f};          // world
    math::Vec3 twistAxis{0.0f, 1.0f, 0.0f};  // world: twist about it, swing away from it
    math::Vec3 planeAxis{1.0f, 0.0f, 0.0f};  // world, perpendicular to twistAxis
    float normalHalfCone = 0.0f, planeHalfCone = 0.0f;  // swing limits
    float minTwist = 0.0f, maxTwist = 0.0f;
};
// A ball joint with real, uneven ranges (Jolt SixDOF, pyramid swing): the
// bodies stay together at the pivot and turn about the joint's X, Y and Z
// axes (world, as created; Z = X cross Y), each between its own min and max
// (radians), e.g. a hip that flexes far forward but extends little.
struct SixDofDesc {
    math::Vec3 pivot{0.0f};
    math::Vec3 axisX{1.0f, 0.0f, 0.0f};
    math::Vec3 axisY{0.0f, 1.0f, 0.0f};
    math::Vec3 minAngles{0.0f}, maxAngles{0.0f};  // about X, Y, Z
};
// A joint motor (Jolt MotorSettings): drives the joint toward its target
// like a spring of `frequency` Hz and `damping` ratio, never with more than
// `maxTorque` N m.
struct MotorDesc {
    float frequency = 2.0f;
    float damping = 1.0f;
    float maxTorque = 1e30f;
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
    // Kinematic body: reach this pose (position and rotation) in `dt`.
    void moveKinematic(BodyId id, const math::Transform& pose, float dt);

    // Springs: Jolt distance constraints with soft limits, between a point on
    // body a and a point on body b (local, from each body's centre of mass).
    // Inside [minDistance, maxDistance] the spring is slack; outside it, it
    // pushes or pulls back at `frequency` Hz with `damping` ratio (Jolt
    // SpringSettings). Load time only (allocates); kInvalidSpring when full.
    SpringId createSpring(BodyId a, math::Vec3 localA, BodyId b, math::Vec3 localB, float minDistance,
                          float maxDistance, float frequency, float damping);
    void setSpringDistance(SpringId id, float minDistance, float maxDistance);
    void setSpringEnabled(SpringId id, bool enabled);

    // Joints (load time; kInvalidJoint when full). Generic: any two bodies.
    JointId createHinge(BodyId a, BodyId b, const HingeDesc& d);
    JointId createSwingTwist(BodyId a, BodyId b, const SwingTwistDesc& d);
    JointId createSixDof(BodyId a, BodyId b, const SixDofDesc& d);
    // Motor on (driving to the target set below) or off (the joint is free
    // within its limits).
    void setJointMotor(JointId id, const MotorDesc& motor, bool on);
    // Hinge: target angle (rad, clamped to its limits); current angle.
    void setHingeTarget(JointId id, float angle);
    float hingeAngle(JointId id) const;
    // Swing-twist: target rotation of body b relative to body a, in the
    // joint's frame as created (identity: as created).
    void setSwingTwistTarget(JointId id, math::Quat rotation);
    math::Quat swingTwistRotation(JointId id) const;
    // SixDOF: target rotation of body b relative to body a in the joint's
    // frame as created (clamped by the limits); current rotation.
    void setSixDofTarget(JointId id, math::Quat rotation);
    math::Quat sixDofRotation(JointId id) const;

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
