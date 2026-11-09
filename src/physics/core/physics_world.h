#pragma once
// Thin wrapper over Jolt Physics for generic rigid bodies (later: player
// bodies, bat). Game code sees only this interface; Jolt stays in the .cpp.
// Stepped by the fixed-timestep sim; keeps the previous and current transform
// of every body so rendering can interpolate. World settings (gravity, air)
// are shared with the ball solver.

#include "math/math.h"

#include <cstdint>
#include <memory>
#include <vector>

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
using CharacterId = uint32_t;
constexpr CharacterId kInvalidCharacter = 0xffffffffu;
using RagdollId = uint32_t;
constexpr RagdollId kInvalidRagdoll = 0xffffffffu;
// config/physics_world.json "world": shared by Jolt and the ball solver.
struct WorldSettings {
    math::Vec3 gravity{0.0f, -9.81f, 0.0f};
    float airDensity = 1.225f;  // kg/m^3
    math::Vec3 wind{0.0f};      // m/s
    float groundHeight = 0.0f;  // m, top of the ground: the one ground everything stays above
};

// config/physics_world.json "character": a player's capsule (Jolt
// CharacterVirtual), the one thing that carries a player over the ground. It
// stands upright always, slides along what it hits, climbs small steps and
// stays on slopes up to maxSlope; it collides with the static scenery and
// with other capsules only (a player's own body parts ride on it).
struct CharacterSettings {
    float radius = 0.3f;    // m
    float maxSlope = 0.785f;  // rad, steepest ground it walks on (config in degrees)
    float stepUp = 0.3f;    // m, highest step it walks up
    float stickDown = 0.3f; // m, it follows the ground down this far (no hop off small drops)
    float mass = 73.0f;     // kg, for pushing other capsules
    float maxStrength = 500.0f;  // N, its push on what it touches
    float padding = 0.02f;  // m, gap kept to what it touches
    // Its hold of the body: a spring from the capsule to the pelvis (each of
    // the six axes), so the full-physics body stays up. Hz, damping ratio,
    // force (N) and torque (N m) limits.
    float supportFrequency = 4.0f, supportDamping = 1.0f;
    float supportMaxForce = 2000.0f, supportMaxTorque = 600.0f;
    // m: the hips may sink this far below the pose's pelvis with no hold
    // (their weight goes down the legs); below it the spring catches them.
    float supportFreeDown = 0.03f;
    // The hold lets go once the hips lean past these (rad, seen from the
    // centre of both feet; config in degrees), blended between them.
    bool releaseOn = false;  // off for now: the hold never lets go
    float releaseForward = 0.148f, releaseBack = 0.0785f, releaseLeft = 0.14f, releaseRight = 0.14f;
    // Let go, the body falls; once its centre of mass is slower than
    // restSpeed (m/s) for recoverWait (s), the capsule goes under the pelvis
    // and the hold eases back in over recoverTime (s).
    float restSpeed = 0.2f, recoverWait = 1.0f, recoverTime = 1.0f;
};

// config/physics_world.json "jolt", "ground" and "character".
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
    int maxCharacters = 32;  // player capsules (load time)
    CharacterSettings character;
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
// `maxTorque` N m. With `stiffness` > 0 the spring is that stiff instead
// (N m/rad) and `damping` is its damping (N m s/rad).
struct MotorDesc {
    float frequency = 2.0f;
    float damping = 1.0f;
    float maxTorque = 1e30f;
    float stiffness = 0.0f;
};

// One part of a ragdoll (Jolt Ragdoll), parents before children.
struct RagdollPart {
    BodyDesc body;    // its shape, mass and pose (world, as built); made dynamic
    int parent = -1;  // the part it hangs from (-1: the root part)
    // Its joint to the parent (Jolt SwingTwist), world space as built: the
    // pivot, the parent's joint frame and the part's (twist axis and plane
    // axis each; the normal axis is plane x twist). Swing about the normal
    // axis within +-planeHalfCone, about the plane axis within
    // +-normalHalfCone (0: locked), twist within [twistMin, twistMax]; rad.
    math::Vec3 pivot{0.0f};
    math::Vec3 twistAxis1{1.0f, 0.0f, 0.0f}, planeAxis1{0.0f, 0.0f, 1.0f};
    math::Vec3 twistAxis2{1.0f, 0.0f, 0.0f}, planeAxis2{0.0f, 0.0f, 1.0f};
    float twistMin = 0.0f, twistMax = 0.0f;
    float normalHalfCone = 0.0f, planeHalfCone = 0.0f;
    MotorDesc motor;  // its muscle for the pose drive; maxTorque 0: none
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
    const PhysicsSettings& settings() const;

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
    // Torque (world, N m) on the body for the next step.
    void addTorque(BodyId id, math::Vec3 torque);
    // Force (world, N) at the body's centre of mass for the next step (wakes it).
    void addForce(BodyId id, math::Vec3 force);
    // The body's friction and restitution (Jolt combines two bodies' friction
    // by their geometric mean, restitution by the larger).
    void setMaterial(BodyId id, float friction, float restitution);
    math::Vec3 angularVelocity(BodyId id) const;  // world, rad/s
    // A moving body switches between kinematic (moved only by moveKinematic)
    // and dynamic (gravity, forces, joints, contacts), keeping its mass and
    // velocity: a player's physics on and off.
    void setKinematic(BodyId id, bool kinematic);
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
    // A joint on (solved) or off (the two bodies move freely), e.g. a foot lock.
    void setJointEnabled(JointId id, bool enabled);
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
    // SixDOF: its rotation ranges (rad, per axis of its frame as created),
    // changed while it runs (a range set by another joint's angle).
    void setSixDofLimits(JointId id, math::Vec3 minAngles, math::Vec3 maxAngles);
    // Velocity motors (a muscle turning its joint): the motor turns the joint
    // at this angular velocity, as fast as its torque limit allows, and holds
    // it at 0. Switches the motor from its target (setJointMotor) to velocity.
    // Hinge: about its axis (rad/s). SixDOF: in the joint's frame on body b.
    void setHingeVelocity(JointId id, float angularVelocity);
    void setSixDofVelocity(JointId id, math::Vec3 angularVelocity);
    // A spring on every rotation axis toward a target (Jolt's position motor,
    // stiffness-and-damping mode): `rotation` as setSixDofTarget, `angle` as
    // setHingeTarget; `stiffness` N m/rad, `damping` N m s/rad, never harder
    // than `maxTorque`.
    void setSixDofSpringTarget(JointId id, math::Quat rotation, float stiffness, float damping, float maxTorque);
    void setHingeSpringTarget(JointId id, float angle, float stiffness, float damping, float maxTorque);
    // The torque (N m) the joint's motor applied in the last step of `dt` s.
    float jointMotorTorque(JointId id, float dt) const;

    // The ground (Jolt's static scenery) under `point`: its height there, or
    // the world's groundHeight if nothing is below.
    float groundHeightAt(math::Vec3 point) const;
    // `point` raised, if needed, so it stays `clearance` m above the ground.
    // The one check every mover that is not a colliding body uses.
    math::Vec3 aboveGround(math::Vec3 point, float clearance = 0.0f) const;
    // Where the body rests on the static ground (Jolt's narrow phase): the
    // corners of its touching face that lie on the ground (within the
    // speculative contact distance), else the deepest point. Writes up to
    // `capacity` points (world) to `out`; returns how many (0: not touching).
    // No heap allocations.
    int groundContacts(BodyId id, math::Vec3* out, int capacity) const;
    bool awake(BodyId id) const;

    // Player capsules (Jolt CharacterVirtual; PhysicsSettings::character),
    // `height` m tall (the player's own). Load time; kInvalidCharacter when
    // full. `feet`: world, the bottom of the capsule.
    CharacterId createCharacter(float height, math::Vec3 feet);
    // One tick: moves the capsule at `velocity` (world, m/s; its vertical part
    // is ignored: gravity pulls it down off the ground, the ground holds it),
    // sliding along what it hits. Returns the feet's new position. No heap
    // allocations.
    math::Vec3 moveCharacter(CharacterId id, math::Vec3 velocity, float dt);
    // Teleports the capsule (no sweep), e.g. a reset; vertical speed zero.
    void placeCharacter(CharacterId id, math::Vec3 feet);
    math::Vec3 characterFeet(CharacterId id) const;
    bool characterOnGround(CharacterId id) const;
    // The capsule's height (m, the nearest of its shapes made at load time,
    // 5 cm apart, 40-100 % of its full height); kept if the new shape does not
    // fit where it is (e.g. standing up under something). No allocations.
    void setCharacterHeight(CharacterId id, float height);
    float characterHeight(CharacterId id) const;  // m, of the shape in use

    // A ragdoll (Jolt Ragdoll): one dynamic body per part, joined to its
    // parent by a SwingTwist joint with its motor; its parts never collide
    // with each other (`group` > 0). `bodies` gets each part's body (the
    // usual body calls work on them). Load time; kInvalidRagdoll when full.
    RagdollId createRagdoll(const std::vector<RagdollPart>& parts, uint32_t group, std::vector<BodyId>& bodies);
    // Jolt Ragdoll::DriveToPoseUsingMotors: every joint's motor pulls its part
    // toward `rotations[i]`, the part body's rotation relative to its parent's
    // body (body space; [0], the root, unused), as strong as its MotorDesc.
    // `count`: the ragdoll's parts. No allocations.
    void driveRagdollToPose(RagdollId id, const math::Quat* rotations, int count);

    // A soft hold of body `b` by body `a` (Jolt SixDOF, every axis free, each
    // driven by a spring motor toward its target): `linear` for position
    // (maxTorque read as the force limit, N), `angular` for rotation. Load
    // time; kInvalidJoint when full. Off/on: setJointEnabled. freeDown > 0:
    // up/down has no motor; b is free from its target to freeDown (m) below
    // it, then a soft limit with `linear`'s stiffness and damping (its force
    // limit does not apply there).
    JointId createSupport(BodyId a, BodyId b, const MotorDesc& linear, const MotorDesc& angular, float freeDown = 0.0f);
    // Its target: b's centre of mass at `offset` from a's (a's frame) and b's
    // rotation `rotation` relative to a's.
    void setSupportTarget(JointId id, math::Vec3 offset, math::Quat rotation);
    // Its force (N) and torque (N m) limits, e.g. eased in after a release.
    void setSupportLimits(JointId id, float maxForce, float maxTorque);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    WorldSettings world_;
};

} // namespace physics
