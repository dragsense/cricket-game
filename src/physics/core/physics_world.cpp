#include "physics/core/physics_world.h"

#include "core/config.h"
#include "core/log.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollector.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <vector>

namespace physics {

namespace {

// Object layers: static scenery, moving bodies, and bodies that touch nothing
// (held only by springs); static and moving have a broad-phase layer each.
constexpr JPH::ObjectLayer kLayerStatic = 0;
constexpr JPH::ObjectLayer kLayerMoving = 1;
constexpr JPH::ObjectLayer kLayerNone = 2;
constexpr JPH::uint kNumLayers = 3;
constexpr JPH::uint kNumBroadPhaseLayers = 2;

constexpr float kGroundHalfSize = 400.0f;  // covers the whole ground
constexpr float kGroundHalfDepth = 1.0f;
constexpr float kCharacterHeightStep = 0.05f;  // m between a capsule's shapes

void joltTrace(const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    LOG_INFO("jolt: {}", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
bool joltAssert(const char* expr, const char* message, const char* file, JPH::uint line) {
    LOG_ERROR("jolt assert {} ({}) at {}:{}", expr, message ? message : "", file, line);
    return true;  // break into the debugger
}
#endif

// Jolt's allocator, factory and type registry are process-wide; the first
// world sets them up, the last one tears them down.
int gJoltUsers = 0;

void acquireJolt() {
    if (gJoltUsers++ > 0) return;
    JPH::RegisterDefaultAllocator();
    JPH::Trace = joltTrace;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = joltAssert;)
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
}

void releaseJolt() {
    if (--gJoltUsers > 0) return;
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

JPH::Vec3 toJolt(math::Vec3 v) { return {v.x, v.y, v.z}; }
JPH::Quat toJolt(math::Quat q) { return {q.x, q.y, q.z, q.w}; }
math::Vec3 fromJolt(JPH::Vec3 v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
math::Quat fromJolt(JPH::Quat q) { return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; }

} // namespace

WorldSettings readWorldSettings(const core::Config& c) {
    WorldSettings w;
    w.gravity = c.getVec3("world.gravity", w.gravity);
    w.airDensity = c.getFloat("world.airDensity", w.airDensity);
    w.wind = c.getVec3("world.wind", w.wind);
    w.groundHeight = c.getFloat("world.groundHeight", w.groundHeight);
    return w;
}

PhysicsSettings readPhysicsSettings(const core::Config& c) {
    PhysicsSettings s;
    s.maxBodies = c.getInt("jolt.maxBodies", s.maxBodies);
    s.maxBodyPairs = c.getInt("jolt.maxBodyPairs", s.maxBodyPairs);
    s.maxContactConstraints = c.getInt("jolt.maxContactConstraints", s.maxContactConstraints);
    s.maxSprings = c.getInt("jolt.maxSprings", s.maxSprings);
    s.maxJoints = c.getInt("jolt.maxJoints", s.maxJoints);
    s.jointVelocitySteps = c.getInt("jolt.jointVelocitySteps", s.jointVelocitySteps);
    s.jointPositionSteps = c.getInt("jolt.jointPositionSteps", s.jointPositionSteps);
    s.tempAllocatorKb = c.getInt("jolt.tempAllocatorKb", s.tempAllocatorKb);
    s.threads = c.getInt("jolt.threads", s.threads);
    s.collisionSteps = c.getInt("jolt.collisionSteps", s.collisionSteps);
    s.penetrationSlop = c.getFloat("jolt.penetrationSlop", s.penetrationSlop);
    s.speculativeContactDistance = c.getFloat("jolt.speculativeContactDistance", s.speculativeContactDistance);
    s.linearDamping = c.getFloat("jolt.linearDamping", s.linearDamping);
    s.angularDamping = c.getFloat("jolt.angularDamping", s.angularDamping);
    s.groundFriction = c.getFloat("ground.friction", s.groundFriction);
    s.groundRestitution = c.getFloat("ground.restitution", s.groundRestitution);
    s.maxCharacters = c.getInt("jolt.maxCharacters", s.maxCharacters);
    CharacterSettings& ch = s.character;
    ch.radius = c.getFloat("character.radius", ch.radius);
    ch.maxSlope = math::radians(c.getFloat("character.maxSlopeDeg", math::degrees(ch.maxSlope)));
    ch.stepUp = c.getFloat("character.stepUp", ch.stepUp);
    ch.stickDown = c.getFloat("character.stickDown", ch.stickDown);
    ch.mass = c.getFloat("character.mass", ch.mass);
    ch.maxStrength = c.getFloat("character.maxStrength", ch.maxStrength);
    ch.padding = c.getFloat("character.padding", ch.padding);
    ch.supportFrequency = c.getFloat("character.support.frequency", ch.supportFrequency);
    ch.supportDamping = c.getFloat("character.support.damping", ch.supportDamping);
    ch.supportMaxForce = c.getFloat("character.support.maxForce", ch.supportMaxForce);
    ch.supportMaxTorque = c.getFloat("character.support.maxTorque", ch.supportMaxTorque);
    ch.supportFreeDown = c.getFloat("character.support.freeDown", ch.supportFreeDown);
    ch.errorSmall = c.getFloat("character.pelvisError.small", ch.errorSmall);
    ch.errorMedium = c.getFloat("character.pelvisError.medium", ch.errorMedium);
    ch.errorLarge = c.getFloat("character.pelvisError.large", ch.errorLarge);
    ch.errorTime = c.getFloat("character.pelvisError.sustained", ch.errorTime);
    ch.releaseOn = c.getBool("character.release.on", ch.releaseOn);
    auto deg = [&](const char* key, float rad) { return math::radians(c.getFloat(key, math::degrees(rad))); };
    ch.releaseForward = deg("character.release.forwardDeg", ch.releaseForward);
    ch.releaseBack = deg("character.release.backDeg", ch.releaseBack);
    ch.releaseLeft = deg("character.release.leftDeg", ch.releaseLeft);
    ch.releaseRight = deg("character.release.rightDeg", ch.releaseRight);
    ch.restSpeed = c.getFloat("character.release.restSpeed", ch.restSpeed);
    ch.recoverWait = c.getFloat("character.release.recoverWait", ch.recoverWait);
    ch.recoverTime = c.getFloat("character.release.recoverTime", ch.recoverTime);
    return s;
}


struct PhysicsWorld::Impl {
    struct Slot {
        JPH::BodyID body;  // invalid when the slot is free
        math::Transform previous, current;
    };

    JPH::BroadPhaseLayerInterfaceTable broadPhaseLayers{kNumLayers, kNumBroadPhaseLayers};
    JPH::ObjectLayerPairFilterTable layerPairs{kNumLayers};
    // Built from the two tables above at construction, so only after they are filled.
    std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> layerVsBroadPhase;
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;
    std::unique_ptr<JPH::JobSystem> jobs;
    JPH::PhysicsSystem system;
    std::vector<Slot> slots;  // sized once at init
    std::vector<JPH::Ref<JPH::DistanceConstraint>> springs;  // reserved once at init
    // Joints, reserved once at init; one of hinges / swingTwists is set per joint.
    std::vector<JPH::Ref<JPH::HingeConstraint>> hinges;
    std::vector<JPH::Ref<JPH::SwingTwistConstraint>> swingTwists;
    std::vector<JPH::Ref<JPH::SixDOFConstraint>> sixDofs;
    JPH::BodyID ground;
    PhysicsSettings settings;
    // Parts of one object (same collision group) skip each other (Jolt group filter).
    JPH::Ref<JPH::GroupFilterTable> sameGroup;
    // Player capsules, reserved once at init; they also collide with each other.
    struct Character {
        JPH::Ref<JPH::CharacterVirtual> character;
        float verticalSpeed = 0.0f;  // m/s, falling off the ground
        float stepUp = 0.0f, stickDown = 0.0f;
        // Its shapes, shortest first (setCharacterHeight), made at load time.
        std::vector<JPH::RefConst<JPH::Shape>> shapes;
        float minHeight = 0.0f, fullHeight = 0.0f;
        int shape = -1;  // the one in use
    };
    std::vector<Character> characters;
    JPH::CharacterVsCharacterCollisionSimple characterVsCharacter;
    // Ragdolls (Jolt owns their bodies and joints); reserved once at init.
    struct RagdollSlot {
        JPH::Ref<JPH::Ragdoll> ragdoll;
        JPH::SkeletonPose pose;      // the pose drive's targets, sized at load
        std::vector<BodyId> bodies;  // its parts' slots
        std::vector<float> baseTorque;  // per joint (parts with a parent, in order): its motor's full torque limit
        // Per joint, as baseTorque: RagdollPart's directional strength.
        struct Directional {
            int axis = -1;
            float plus = 0.0f, minus = 0.0f;
        };
        std::vector<Directional> directional;
        std::vector<int> jointPart;      // per joint, as baseTorque: the part it moves
        std::vector<math::Vec3> hold;    // per part: setRagdollHold's torque (world)
        float scale = 1.0f;  // setRagdollStrength
    };
    std::vector<RagdollSlot> ragdolls;

    Impl() {
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kLayerStatic, JPH::BroadPhaseLayer(0));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kLayerMoving, JPH::BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kLayerNone, JPH::BroadPhaseLayer(1));
        layerPairs.EnableCollision(kLayerMoving, kLayerStatic);
        layerPairs.EnableCollision(kLayerMoving, kLayerMoving);
        layerVsBroadPhase = std::make_unique<JPH::ObjectVsBroadPhaseLayerFilterTable>(
            broadPhaseLayers, kNumBroadPhaseLayers, layerPairs, kNumLayers);
    }

    const Slot* slot(BodyId id) const {
        return id < slots.size() && !slots[id].body.IsInvalid() ? &slots[id] : nullptr;
    }

    void capture(Slot& s) {
        JPH::RVec3 p;
        JPH::Quat q;
        system.GetBodyInterfaceNoLock().GetPositionAndRotation(s.body, p, q);
        s.current.position = fromJolt(JPH::Vec3(p));
        s.current.rotation = fromJolt(q);
    }
};

PhysicsWorld::PhysicsWorld() = default;

PhysicsWorld::~PhysicsWorld() { shutdown(); }

bool PhysicsWorld::init(const WorldSettings& world, const PhysicsSettings& settings) {
    shutdown();
    if (settings.maxBodies <= 0 || settings.tempAllocatorKb <= 0) {
        LOG_ERROR("physics: maxBodies and tempAllocatorKb must be positive");
        return false;
    }
    acquireJolt();
    world_ = world;
    impl_ = std::make_unique<Impl>();
    Impl& m = *impl_;
    m.settings = settings;
    m.tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(static_cast<size_t>(settings.tempAllocatorKb) * 1024);
    if (settings.threads > 0)
        m.jobs = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, settings.threads);
    else
        m.jobs = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);

    m.system.Init(static_cast<JPH::uint>(settings.maxBodies), 0, static_cast<JPH::uint>(settings.maxBodyPairs),
                  static_cast<JPH::uint>(settings.maxContactConstraints), m.broadPhaseLayers, *m.layerVsBroadPhase,
                  m.layerPairs);
    m.system.SetGravity(toJolt(world.gravity));
    JPH::PhysicsSettings ps = m.system.GetPhysicsSettings();
    ps.mPenetrationSlop = settings.penetrationSlop;
    ps.mSpeculativeContactDistance = settings.speculativeContactDistance;
    m.system.SetPhysicsSettings(ps);
    m.slots.resize(static_cast<size_t>(settings.maxBodies));
    m.sameGroup = new JPH::GroupFilterTable(1);  // one subgroup: Jolt never collides a subgroup with itself
    m.springs.reserve(static_cast<size_t>(std::max(settings.maxSprings, 0)));
    m.hinges.reserve(static_cast<size_t>(std::max(settings.maxJoints, 0)));
    m.swingTwists.reserve(static_cast<size_t>(std::max(settings.maxJoints, 0)));
    m.sixDofs.reserve(static_cast<size_t>(std::max(settings.maxJoints, 0)));
    m.characters.reserve(static_cast<size_t>(std::max(settings.maxCharacters, 0)));
    m.ragdolls.reserve(static_cast<size_t>(std::max(settings.maxCharacters, 0)));

    // Static ground slab, top face at the world's groundHeight (the one ground).
    JPH::BodyCreationSettings ground(new JPH::BoxShape(JPH::Vec3(kGroundHalfSize, kGroundHalfDepth, kGroundHalfSize)),
                                     JPH::RVec3(0, world.groundHeight - kGroundHalfDepth, 0), JPH::Quat::sIdentity(),
                                     JPH::EMotionType::Static, kLayerStatic);
    ground.mFriction = settings.groundFriction;
    ground.mRestitution = settings.groundRestitution;
    m.ground = m.system.GetBodyInterface().CreateAndAddBody(ground, JPH::EActivation::DontActivate);
    if (m.ground.IsInvalid()) {
        LOG_ERROR("physics: could not create the ground");
        shutdown();
        return false;
    }

    return true;
}

void PhysicsWorld::shutdown() {
    if (!impl_) return;
    impl_->characterVsCharacter.mCharacters.clear();
    impl_->characters.clear();
    for (JPH::Ref<JPH::DistanceConstraint>& c : impl_->springs) impl_->system.RemoveConstraint(c);
    impl_->springs.clear();
    for (JPH::Ref<JPH::HingeConstraint>& c : impl_->hinges)
        if (c) impl_->system.RemoveConstraint(c);
    for (JPH::Ref<JPH::SwingTwistConstraint>& c : impl_->swingTwists)
        if (c) impl_->system.RemoveConstraint(c);
    for (JPH::Ref<JPH::SixDOFConstraint>& c : impl_->sixDofs)
        if (c) impl_->system.RemoveConstraint(c);
    impl_->hinges.clear();
    impl_->swingTwists.clear();
    impl_->sixDofs.clear();
    // A ragdoll destroys its own bodies.
    for (Impl::RagdollSlot& r : impl_->ragdolls) {
        r.ragdoll->RemoveFromPhysicsSystem();
        for (BodyId b : r.bodies) impl_->slots[b].body = JPH::BodyID();
    }
    impl_->ragdolls.clear();
    JPH::BodyInterface& bi = impl_->system.GetBodyInterface();
    for (Impl::Slot& s : impl_->slots) {
        if (s.body.IsInvalid()) continue;
        bi.RemoveBody(s.body);
        bi.DestroyBody(s.body);
    }
    if (!impl_->ground.IsInvalid()) {
        bi.RemoveBody(impl_->ground);
        bi.DestroyBody(impl_->ground);
    }
    impl_.reset();
    releaseJolt();
}

namespace {

// Jolt's creation settings for a body (its shape, motion, layer, mass).
JPH::BodyCreationSettings bodySettings(const BodyDesc& d, const PhysicsSettings& settings,
                                       const JPH::GroupFilter* sameGroup) {
    JPH::Ref<JPH::Shape> shape;
    switch (d.shape) {
        case ShapeType::Box: {
            float maxRound = std::min({d.halfExtents.x, d.halfExtents.y, d.halfExtents.z});
            shape = new JPH::BoxShape(toJolt(d.halfExtents), std::min(d.convexRadius, maxRound));
            break;
        }
        case ShapeType::Cylinder:
            shape = new JPH::CylinderShape(d.halfHeight, d.radius, std::min({d.convexRadius, d.halfHeight, d.radius}));
            break;
        case ShapeType::Sphere: shape = new JPH::SphereShape(d.radius); break;
        case ShapeType::Capsule: shape = new JPH::CapsuleShape(std::max(d.halfHeight, 1e-4f), d.radius); break;
    }

    const JPH::EMotionType motion = d.kinematic ? JPH::EMotionType::Kinematic
                                    : d.dynamic ? JPH::EMotionType::Dynamic
                                                : JPH::EMotionType::Static;
    const JPH::ObjectLayer layer =
        !d.collides ? kLayerNone : motion == JPH::EMotionType::Static ? kLayerStatic : kLayerMoving;
    JPH::BodyCreationSettings bcs(shape, JPH::RVec3(toJolt(d.pose.position)), toJolt(math::normalize(d.pose.rotation)),
                                  motion, layer);
    bcs.mFriction = d.friction;
    bcs.mRestitution = d.restitution;
    if (d.collisionGroup > 0) bcs.mCollisionGroup = JPH::CollisionGroup(sameGroup, d.collisionGroup, 0);
    bcs.mLinearDamping = settings.linearDamping;
    bcs.mAngularDamping = settings.angularDamping;
    // A kinematic body keeps its mass too, for when it turns dynamic (setKinematic).
    if (motion != JPH::EMotionType::Static) {
        bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        bcs.mMassPropertiesOverride.mMass = d.mass;
    }
    if (motion == JPH::EMotionType::Dynamic) {
        if (d.verticalOnly) bcs.mAllowedDOFs = JPH::EAllowedDOFs::TranslationY;
        else if (d.lockRotation)
            bcs.mAllowedDOFs =
                JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY | JPH::EAllowedDOFs::TranslationZ;
    }
    // A body held by springs or moved by hand never sleeps (it would stop following).
    if (motion != JPH::EMotionType::Static) bcs.mAllowSleeping = d.collides && !d.kinematic;
    return bcs;
}

} // namespace

BodyId PhysicsWorld::createBody(const BodyDesc& d) {
    if (!impl_) return kInvalidBody;
    Impl& m = *impl_;
    auto free = std::find_if(m.slots.begin(), m.slots.end(), [](const Impl::Slot& s) { return s.body.IsInvalid(); });
    if (free == m.slots.end()) {
        LOG_ERROR("physics: body limit ({}) reached", m.slots.size());
        return kInvalidBody;
    }

    const JPH::BodyCreationSettings bcs = bodySettings(d, m.settings, m.sameGroup);
    const JPH::EMotionType motion = bcs.mMotionType;
    JPH::BodyID body = m.system.GetBodyInterface().CreateAndAddBody(
        bcs, motion != JPH::EMotionType::Static ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (body.IsInvalid()) {
        LOG_ERROR("physics: Jolt could not create a body");
        return kInvalidBody;
    }
    free->body = body;
    m.capture(*free);
    free->previous = free->current;
    return static_cast<BodyId>(free - m.slots.begin());
}

void PhysicsWorld::destroyBody(BodyId id) {
    if (!impl_ || !impl_->slot(id)) return;
    Impl::Slot& s = impl_->slots[id];
    JPH::BodyInterface& bi = impl_->system.GetBodyInterface();
    bi.RemoveBody(s.body);
    bi.DestroyBody(s.body);
    s = Impl::Slot{};
}

void PhysicsWorld::step(float dt) {
    if (!impl_) return;
    Impl& m = *impl_;
    m.system.Update(dt, m.settings.collisionSteps, m.tempAllocator.get(), m.jobs.get());
    for (Impl::Slot& s : m.slots) {
        if (s.body.IsInvalid()) continue;
        s.previous = s.current;
        m.capture(s);
    }
}

math::Transform PhysicsWorld::transform(BodyId id) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s ? s->current : math::Transform{};
}

math::Transform PhysicsWorld::interpolated(BodyId id, float alpha) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s ? math::lerp(s->previous, s->current, alpha) : math::Transform{};
}

math::Vec3 PhysicsWorld::linearVelocity(BodyId id) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s ? fromJolt(impl_->system.GetBodyInterfaceNoLock().GetLinearVelocity(s->body)) : math::Vec3(0.0f);
}

void PhysicsWorld::resetBody(BodyId id, const math::Transform& pose) {
    if (!impl_ || !impl_->slot(id)) return;
    Impl::Slot& s = impl_->slots[id];
    impl_->system.GetBodyInterface().SetPositionRotationAndVelocity(
        s.body, JPH::RVec3(toJolt(pose.position)), toJolt(math::normalize(pose.rotation)), JPH::Vec3::sZero(),
        JPH::Vec3::sZero());
    impl_->capture(s);
    s.previous = s.current;
}

void PhysicsWorld::addImpulse(BodyId id, math::Vec3 impulse, math::Vec3 point) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().AddImpulse(impl_->slots[id].body, toJolt(impulse), JPH::RVec3(toJolt(point)));
}

const PhysicsSettings& PhysicsWorld::settings() const {
    static const PhysicsSettings kNone;
    return impl_ ? impl_->settings : kNone;
}

void PhysicsWorld::addTorque(BodyId id, math::Vec3 torque) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().AddTorque(impl_->slots[id].body, toJolt(torque));
}

void PhysicsWorld::addForce(BodyId id, math::Vec3 force) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().AddForce(impl_->slots[id].body, toJolt(force), JPH::EActivation::Activate);
}

void PhysicsWorld::setMaterial(BodyId id, float friction, float restitution) {
    if (!impl_ || !impl_->slot(id)) return;
    JPH::BodyInterface& bi = impl_->system.GetBodyInterface();
    bi.SetFriction(impl_->slots[id].body, friction);
    bi.SetRestitution(impl_->slots[id].body, restitution);
}

void PhysicsWorld::setJointEnabled(JointId id, bool enabled) {
    if (!impl_ || id >= impl_->hinges.size()) return;
    JPH::Constraint* c = impl_->hinges[id] ? static_cast<JPH::Constraint*>(impl_->hinges[id].GetPtr())
                         : impl_->swingTwists[id] ? static_cast<JPH::Constraint*>(impl_->swingTwists[id].GetPtr())
                         : static_cast<JPH::Constraint*>(impl_->sixDofs[id].GetPtr());
    if (c) c->SetEnabled(enabled);
}

math::Vec3 PhysicsWorld::angularVelocity(BodyId id) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s ? fromJolt(impl_->system.GetBodyInterfaceNoLock().GetAngularVelocity(s->body)) : math::Vec3(0.0f);
}

void PhysicsWorld::setKinematic(BodyId id, bool kinematic) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().SetMotionType(
        impl_->slots[id].body, kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic,
        JPH::EActivation::Activate);
}

void PhysicsWorld::moveKinematic(BodyId id, math::Vec3 position, float dt) {
    if (!impl_ || !impl_->slot(id)) return;
    JPH::BodyInterface& bi = impl_->system.GetBodyInterface();
    const JPH::BodyID b = impl_->slots[id].body;
    bi.MoveKinematic(b, JPH::RVec3(toJolt(position)), bi.GetRotation(b), dt);
}

void PhysicsWorld::moveKinematic(BodyId id, const math::Transform& pose, float dt) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().MoveKinematic(impl_->slots[id].body, JPH::RVec3(toJolt(pose.position)),
                                                   toJolt(math::normalize(pose.rotation)), dt);
}

SpringId PhysicsWorld::createSpring(BodyId a, math::Vec3 localA, BodyId b, math::Vec3 localB, float minDistance,
                                    float maxDistance, float frequency, float damping) {
    if (!impl_ || !impl_->slot(a) || !impl_->slot(b)) return kInvalidSpring;
    Impl& m = *impl_;
    if (m.springs.size() >= m.springs.capacity()) {
        LOG_ERROR("physics: spring limit ({}) reached", m.springs.capacity());
        return kInvalidSpring;
    }
    JPH::DistanceConstraintSettings s;
    s.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    s.mPoint1 = JPH::RVec3(toJolt(localA));
    s.mPoint2 = JPH::RVec3(toJolt(localB));
    s.mMinDistance = minDistance;
    s.mMaxDistance = std::max(minDistance, maxDistance);
    s.mLimitsSpringSettings = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping, frequency, damping);
    const JPH::BodyID ids[2] = {m.slots[a].body, m.slots[b].body};
    JPH::BodyLockMultiWrite lock(m.system.GetBodyLockInterface(), ids, 2);
    JPH::Body* b1 = lock.GetBody(0);
    JPH::Body* b2 = lock.GetBody(1);
    if (!b1 || !b2) return kInvalidSpring;
    JPH::Ref<JPH::DistanceConstraint> c = static_cast<JPH::DistanceConstraint*>(s.Create(*b1, *b2));
    m.system.AddConstraint(c);
    m.springs.push_back(c);
    return static_cast<SpringId>(m.springs.size() - 1);
}

void PhysicsWorld::setSpringDistance(SpringId id, float minDistance, float maxDistance) {
    if (!impl_ || id >= impl_->springs.size()) return;
    impl_->springs[id]->SetDistance(minDistance, std::max(minDistance, maxDistance));
}

void PhysicsWorld::setSpringEnabled(SpringId id, bool enabled) {
    if (!impl_ || id >= impl_->springs.size()) return;
    impl_->springs[id]->SetEnabled(enabled);
}

namespace {

// Locks bodies a and b and creates the joint from the settings; nullptr if a
// body is missing.
template <class Constraint, class Settings>
JPH::Ref<Constraint> makeJoint(JPH::PhysicsSystem& system, JPH::BodyID a, JPH::BodyID b, const Settings& settings) {
    const JPH::BodyID ids[2] = {a, b};
    JPH::BodyLockMultiWrite lock(system.GetBodyLockInterface(), ids, 2);
    JPH::Body* b1 = lock.GetBody(0);
    JPH::Body* b2 = lock.GetBody(1);
    if (!b1 || !b2) return nullptr;
    return static_cast<Constraint*>(settings.Create(*b1, *b2));
}

// Joint solver iterations (Jolt overrides): more keep a joint inside its
// limits under hard loads (a body landing on its joints).
void applySteps(JPH::Constraint& c, const PhysicsSettings& s) {
    if (s.jointVelocitySteps > 0) c.SetNumVelocityStepsOverride(static_cast<JPH::uint>(std::min(s.jointVelocitySteps, 255)));
    if (s.jointPositionSteps > 0) c.SetNumPositionStepsOverride(static_cast<JPH::uint>(std::min(s.jointPositionSteps, 255)));
}

JPH::MotorSettings toJolt(const MotorDesc& d) {
    JPH::MotorSettings s(std::max(d.frequency, 0.0f), std::max(d.damping, 0.0f));
    if (d.stiffness > 0.0f)
        s.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping, d.stiffness, std::max(d.damping, 0.0f));
    s.SetTorqueLimit(std::max(d.maxTorque, 0.0f));
    return s;
}

} // namespace

JointId PhysicsWorld::createHinge(BodyId a, BodyId b, const HingeDesc& d) {
    if (!impl_ || !impl_->slot(a) || !impl_->slot(b)) return kInvalidJoint;
    Impl& m = *impl_;
    if (m.hinges.size() >= m.hinges.capacity()) {
        LOG_ERROR("physics: joint limit ({}) reached", m.hinges.capacity());
        return kInvalidJoint;
    }
    JPH::HingeConstraintSettings s;
    s.mSpace = JPH::EConstraintSpace::WorldSpace;
    s.mPoint1 = s.mPoint2 = JPH::RVec3(toJolt(d.pivot));
    s.mHingeAxis1 = s.mHingeAxis2 = toJolt(math::normalize(d.axis));
    s.mNormalAxis1 = s.mNormalAxis2 = toJolt(math::normalize(d.normal));
    s.mLimitsMin = d.minAngle;
    s.mLimitsMax = d.maxAngle;
    JPH::Ref<JPH::HingeConstraint> c = makeJoint<JPH::HingeConstraint>(m.system, m.slots[a].body, m.slots[b].body, s);
    if (!c) return kInvalidJoint;
    applySteps(*c, m.settings);
    m.system.AddConstraint(c);
    m.hinges.push_back(c);
    m.swingTwists.push_back(nullptr);
    m.sixDofs.push_back(nullptr);
    return static_cast<JointId>(m.hinges.size() - 1);
}

JointId PhysicsWorld::createSwingTwist(BodyId a, BodyId b, const SwingTwistDesc& d) {
    if (!impl_ || !impl_->slot(a) || !impl_->slot(b)) return kInvalidJoint;
    Impl& m = *impl_;
    if (m.hinges.size() >= m.hinges.capacity()) {
        LOG_ERROR("physics: joint limit ({}) reached", m.hinges.capacity());
        return kInvalidJoint;
    }
    JPH::SwingTwistConstraintSettings s;
    s.mSpace = JPH::EConstraintSpace::WorldSpace;
    s.mPosition1 = s.mPosition2 = JPH::RVec3(toJolt(d.pivot));
    s.mTwistAxis1 = s.mTwistAxis2 = toJolt(math::normalize(d.twistAxis));
    s.mPlaneAxis1 = s.mPlaneAxis2 = toJolt(math::normalize(d.planeAxis));
    s.mNormalHalfConeAngle = d.normalHalfCone;
    s.mPlaneHalfConeAngle = d.planeHalfCone;
    s.mTwistMinAngle = d.minTwist;
    s.mTwistMaxAngle = d.maxTwist;
    JPH::Ref<JPH::SwingTwistConstraint> c =
        makeJoint<JPH::SwingTwistConstraint>(m.system, m.slots[a].body, m.slots[b].body, s);
    if (!c) return kInvalidJoint;
    applySteps(*c, m.settings);
    m.system.AddConstraint(c);
    m.hinges.push_back(nullptr);
    m.swingTwists.push_back(c);
    m.sixDofs.push_back(nullptr);
    return static_cast<JointId>(m.hinges.size() - 1);
}

JointId PhysicsWorld::createSixDof(BodyId a, BodyId b, const SixDofDesc& d) {
    if (!impl_ || !impl_->slot(a) || !impl_->slot(b)) return kInvalidJoint;
    Impl& m = *impl_;
    if (m.hinges.size() >= m.hinges.capacity()) {
        LOG_ERROR("physics: joint limit ({}) reached", m.hinges.capacity());
        return kInvalidJoint;
    }
    JPH::SixDOFConstraintSettings s;
    s.mSpace = JPH::EConstraintSpace::WorldSpace;
    s.mPosition1 = s.mPosition2 = JPH::RVec3(toJolt(d.pivot));
    s.mAxisX1 = s.mAxisX2 = toJolt(math::normalize(d.axisX));
    s.mAxisY1 = s.mAxisY2 = toJolt(math::normalize(d.axisY));
    s.mSwingType = JPH::ESwingType::Pyramid;  // uneven swing ranges
    using A = JPH::SixDOFConstraintSettings::EAxis;
    s.MakeFixedAxis(A::TranslationX);
    s.MakeFixedAxis(A::TranslationY);
    s.MakeFixedAxis(A::TranslationZ);
    s.SetLimitedAxis(A::RotationX, d.minAngles.x, d.maxAngles.x);
    s.SetLimitedAxis(A::RotationY, d.minAngles.y, d.maxAngles.y);
    s.SetLimitedAxis(A::RotationZ, d.minAngles.z, d.maxAngles.z);
    JPH::Ref<JPH::SixDOFConstraint> c = makeJoint<JPH::SixDOFConstraint>(m.system, m.slots[a].body, m.slots[b].body, s);
    if (!c) return kInvalidJoint;
    applySteps(*c, m.settings);
    m.system.AddConstraint(c);
    m.hinges.push_back(nullptr);
    m.swingTwists.push_back(nullptr);
    m.sixDofs.push_back(c);
    return static_cast<JointId>(m.hinges.size() - 1);
}

void PhysicsWorld::setSixDofLimits(JointId id, math::Vec3 minAngles, math::Vec3 maxAngles) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    impl_->sixDofs[id]->SetRotationLimits(toJolt(minAngles), toJolt(maxAngles));
}

void PhysicsWorld::setSixDofTarget(JointId id, math::Quat rotation) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    impl_->sixDofs[id]->SetTargetOrientationCS(toJolt(math::normalize(rotation)));
}

math::Quat PhysicsWorld::sixDofRotation(JointId id) const {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return math::identityQuat();
    return fromJolt(impl_->sixDofs[id]->GetRotationInConstraintSpace());
}

void PhysicsWorld::setJointMotor(JointId id, const MotorDesc& motor, bool on) {
    if (!impl_ || id >= impl_->hinges.size()) return;
    const JPH::EMotorState state = on ? JPH::EMotorState::Position : JPH::EMotorState::Off;
    if (JPH::HingeConstraint* h = impl_->hinges[id]) {
        h->GetMotorSettings() = toJolt(motor);
        h->SetMotorState(state);
    } else if (JPH::SwingTwistConstraint* s = impl_->swingTwists[id]) {
        s->GetSwingMotorSettings() = toJolt(motor);
        s->GetTwistMotorSettings() = toJolt(motor);
        s->SetSwingMotorState(state);
        s->SetTwistMotorState(state);
    } else if (JPH::SixDOFConstraint* x = impl_->sixDofs[id]) {
        using A = JPH::SixDOFConstraintSettings::EAxis;
        for (A axis : {A::RotationX, A::RotationY, A::RotationZ}) {
            x->GetMotorSettings(axis) = toJolt(motor);
            x->SetMotorState(axis, state);
        }
    }
}

void PhysicsWorld::setHingeVelocity(JointId id, float angularVelocity) {
    if (!impl_ || id >= impl_->hinges.size() || !impl_->hinges[id]) return;
    JPH::HingeConstraint* h = impl_->hinges[id];
    h->SetMotorState(JPH::EMotorState::Velocity);
    h->SetTargetAngularVelocity(angularVelocity);
}

void PhysicsWorld::setSixDofVelocity(JointId id, math::Vec3 angularVelocity) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    JPH::SixDOFConstraint* x = impl_->sixDofs[id];
    using A = JPH::SixDOFConstraintSettings::EAxis;
    for (A axis : {A::RotationX, A::RotationY, A::RotationZ}) x->SetMotorState(axis, JPH::EMotorState::Velocity);
    x->SetTargetAngularVelocityCS(toJolt(angularVelocity));
}

namespace {

JPH::MotorSettings springMotor(float stiffness, float damping, float maxTorque) {
    JPH::MotorSettings s;
    s.mSpringSettings.mMode = JPH::ESpringMode::StiffnessAndDamping;
    s.mSpringSettings.mStiffness = std::max(stiffness, 0.0f);
    s.mSpringSettings.mDamping = std::max(damping, 0.0f);
    s.SetTorqueLimit(std::max(maxTorque, 0.0f));
    return s;
}

} // namespace

void PhysicsWorld::setSixDofSpringTarget(JointId id, math::Quat rotation, float stiffness, float damping,
                                         float maxTorque) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    JPH::SixDOFConstraint* c = impl_->sixDofs[id];
    using A = JPH::SixDOFConstraintSettings::EAxis;
    for (A axis : {A::RotationX, A::RotationY, A::RotationZ}) {
        c->GetMotorSettings(axis) = springMotor(stiffness, damping, maxTorque);
        c->SetMotorState(axis, JPH::EMotorState::Position);
    }
    c->SetTargetOrientationCS(toJolt(math::normalize(rotation)));
}

void PhysicsWorld::setHingeSpringTarget(JointId id, float angle, float stiffness, float damping, float maxTorque) {
    if (!impl_ || id >= impl_->hinges.size() || !impl_->hinges[id]) return;
    JPH::HingeConstraint* h = impl_->hinges[id];
    h->GetMotorSettings() = springMotor(stiffness, damping, maxTorque);
    h->SetMotorState(JPH::EMotorState::Position);
    h->SetTargetAngle(angle);
}

float PhysicsWorld::jointMotorTorque(JointId id, float dt) const {
    if (!impl_ || id >= impl_->hinges.size() || dt <= 0.0f) return 0.0f;
    if (const JPH::HingeConstraint* h = impl_->hinges[id]) return std::fabs(h->GetTotalLambdaMotor()) / dt;
    if (const JPH::SixDOFConstraint* x = impl_->sixDofs[id]) return x->GetTotalLambdaMotorRotation().Length() / dt;
    return 0.0f;
}

void PhysicsWorld::setHingeTarget(JointId id, float angle) {
    if (!impl_ || id >= impl_->hinges.size() || !impl_->hinges[id]) return;
    impl_->hinges[id]->SetTargetAngle(angle);
}

float PhysicsWorld::hingeAngle(JointId id) const {
    if (!impl_ || id >= impl_->hinges.size() || !impl_->hinges[id]) return 0.0f;
    return impl_->hinges[id]->GetCurrentAngle();
}

void PhysicsWorld::setSwingTwistTarget(JointId id, math::Quat rotation) {
    if (!impl_ || id >= impl_->swingTwists.size() || !impl_->swingTwists[id]) return;
    impl_->swingTwists[id]->SetTargetOrientationCS(toJolt(math::normalize(rotation)));
}

math::Quat PhysicsWorld::swingTwistRotation(JointId id) const {
    if (!impl_ || id >= impl_->swingTwists.size() || !impl_->swingTwists[id]) return math::identityQuat();
    return fromJolt(impl_->swingTwists[id]->GetRotationInConstraintSpace());
}

float PhysicsWorld::groundHeightAt(math::Vec3 point) const {
    if (!impl_) return world_.groundHeight;
    // Jolt ray straight down onto the static scenery (the ground layer).
    constexpr float kFrom = 100.0f;  // m above and below the point (float precision ~0.01 mm)
    const JPH::RRayCast ray{JPH::RVec3(point.x, point.y + kFrom, point.z), JPH::Vec3(0.0f, -2.0f * kFrom, 0.0f)};
    JPH::RayCastResult hit;
    const JPH::SpecifiedObjectLayerFilter staticOnly(kLayerStatic);
    if (!impl_->system.GetNarrowPhaseQueryNoLock().CastRay(ray, hit, {}, staticOnly)) return world_.groundHeight;
    return static_cast<float>(ray.GetPointOnRay(hit.mFraction).GetY());
}

math::Vec3 PhysicsWorld::aboveGround(math::Vec3 point, float clearance) const {
    point.y = std::max(point.y, groundHeightAt(point) + clearance);
    return point;
}

int PhysicsWorld::groundContacts(BodyId id, math::Vec3* out, int capacity) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    if (!s || !out || capacity <= 0) return 0;
    const JPH::BodyInterface& bodies = impl_->system.GetBodyInterfaceNoLock();
    const JPH::RefConst<JPH::Shape> shape = bodies.GetShape(s->body);
    if (!shape) return 0;
    const float reach = impl_->settings.speculativeContactDistance;
    JPH::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = reach;
    settings.mCollectFacesMode = JPH::ECollectFacesMode::CollectFaces;
    // The deepest touch against the static ground, with the body's touching face.
    struct Collector final : JPH::CollideShapeCollector {
        JPH::CollideShapeResult best;
        bool hit = false;
        void AddHit(const JPH::CollideShapeResult& r) override {
            if (!hit || r.mPenetrationDepth > best.mPenetrationDepth) {
                best = r;
                hit = true;
            }
        }
    } collector;
    const JPH::SpecifiedObjectLayerFilter staticOnly(kLayerStatic);
    impl_->system.GetNarrowPhaseQueryNoLock().CollideShape(shape, JPH::Vec3::sOne(),
                                                           bodies.GetCenterOfMassTransform(s->body), settings,
                                                           JPH::RVec3::sZero(), collector, {}, staticOnly);
    if (!collector.hit) return 0;
    // The face's corners that lie on the ground: no further from the ground's
    // contact plane than the speculative distance.
    const JPH::CollideShapeResult& r = collector.best;
    const JPH::Vec3 normal = r.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());  // into the ground
    const JPH::Vec3 onGround(r.mContactPointOn2);
    int count = 0;
    for (const JPH::Vec3& corner : r.mShape1Face) {
        if (count >= capacity) break;
        if (normal.Dot(corner - onGround) >= -reach) out[count++] = fromJolt(corner);
    }
    if (count == 0) out[count++] = fromJolt(onGround);
    return count;
}

CharacterId PhysicsWorld::createCharacter(float height, math::Vec3 feet) {
    if (!impl_ || impl_->characters.size() >= impl_->characters.capacity()) return kInvalidCharacter;
    Impl& m = *impl_;
    const CharacterSettings& cs = m.settings.character;
    const float radius = std::max(cs.radius, 0.01f);
    // Its shapes, 40-100 % of its height in kCharacterHeightStep steps; each
    // stands on its origin (the feet).
    Impl::Character c;
    c.minHeight = std::max(0.4f * height, 2.0f * radius + 0.02f);
    for (float tall = c.minHeight;; tall += kCharacterHeightStep) {
        const float full = std::min(tall, height);
        const float halfCylinder = std::max(0.5f * full - radius, 0.01f);
        JPH::RotatedTranslatedShapeSettings shape(JPH::Vec3(0.0f, halfCylinder + radius, 0.0f), JPH::Quat::sIdentity(),
                                                  new JPH::CapsuleShape(halfCylinder, radius));
        JPH::ShapeSettings::ShapeResult made = shape.Create();
        if (made.HasError()) {
            LOG_ERROR("physics: character shape: {}", made.GetError().c_str());
            return kInvalidCharacter;
        }
        c.shapes.push_back(made.Get());
        if (full >= height) break;
    }
    c.shape = static_cast<int>(c.shapes.size()) - 1;
    c.fullHeight = height;
    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
    settings->mShape = c.shapes.back();
    settings->mMaxSlopeAngle = cs.maxSlope;
    settings->mMass = cs.mass;
    settings->mMaxStrength = cs.maxStrength;
    settings->mCharacterPadding = cs.padding;
    // Contacts below the lower sphere's centre carry it (the feet).
    settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -radius);
    c.character = new JPH::CharacterVirtual(settings, JPH::RVec3(toJolt(feet)), JPH::Quat::sIdentity(), 0, &m.system);
    c.character->SetCharacterVsCharacterCollision(&m.characterVsCharacter);
    c.stepUp = cs.stepUp;
    c.stickDown = cs.stickDown;
    m.characterVsCharacter.Add(c.character);
    m.characters.push_back(c);
    return static_cast<CharacterId>(m.characters.size() - 1);
}

math::Vec3 PhysicsWorld::moveCharacter(CharacterId id, math::Vec3 velocity, float dt) {
    if (!impl_ || id >= impl_->characters.size() || dt <= 0.0f) return characterFeet(id);
    Impl& m = *impl_;
    Impl::Character& c = m.characters[id];
    JPH::CharacterVirtual& ch = *c.character;
    // On the ground it stays there; off it, gravity pulls it down.
    if (ch.GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround)
        c.verticalSpeed = 0.0f;
    else
        c.verticalSpeed += world_.gravity.y * dt;
    ch.SetLinearVelocity(JPH::Vec3(velocity.x, c.verticalSpeed, velocity.z));
    JPH::CharacterVirtual::ExtendedUpdateSettings update;
    update.mStickToFloorStepDown = JPH::Vec3(0.0f, -c.stickDown, 0.0f);
    update.mWalkStairsStepUp = JPH::Vec3(0.0f, c.stepUp, 0.0f);
    // Static scenery only (and the other capsules): a player's body parts
    // and loose objects do not stop it.
    const JPH::SpecifiedBroadPhaseLayerFilter broadPhase(JPH::BroadPhaseLayer(0));
    const JPH::SpecifiedObjectLayerFilter layer(kLayerStatic);
    ch.ExtendedUpdate(dt, toJolt(world_.gravity), update, broadPhase, layer, JPH::BodyFilter(), JPH::ShapeFilter(),
                      *m.tempAllocator);
    return characterFeet(id);
}

void PhysicsWorld::placeCharacter(CharacterId id, math::Vec3 feet) {
    if (!impl_ || id >= impl_->characters.size()) return;
    Impl::Character& c = impl_->characters[id];
    c.character->SetPosition(JPH::RVec3(toJolt(feet)));
    c.character->SetLinearVelocity(JPH::Vec3::sZero());
    c.verticalSpeed = 0.0f;
}

math::Vec3 PhysicsWorld::characterFeet(CharacterId id) const {
    if (!impl_ || id >= impl_->characters.size()) return math::Vec3(0.0f);
    return fromJolt(JPH::Vec3(impl_->characters[id].character->GetPosition()));
}

bool PhysicsWorld::characterOnGround(CharacterId id) const {
    if (!impl_ || id >= impl_->characters.size()) return false;
    return impl_->characters[id].character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround;
}

bool PhysicsWorld::awake(BodyId id) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s && impl_->system.GetBodyInterfaceNoLock().IsActive(s->body);
}

void PhysicsWorld::setCharacterHeight(CharacterId id, float height) {
    if (!impl_ || id >= impl_->characters.size()) return;
    Impl& m = *impl_;
    Impl::Character& c = m.characters[id];
    const int last = static_cast<int>(c.shapes.size()) - 1;
    const int want = std::clamp(static_cast<int>(std::lround((height - c.minHeight) / kCharacterHeightStep)), 0, last);
    if (want == c.shape) return;
    const JPH::SpecifiedBroadPhaseLayerFilter broadPhase(JPH::BroadPhaseLayer(0));
    const JPH::SpecifiedObjectLayerFilter layer(kLayerStatic);
    if (c.character->SetShape(c.shapes[static_cast<size_t>(want)], 1.5f * m.settings.character.padding, broadPhase,
                              layer, JPH::BodyFilter(), JPH::ShapeFilter(), *m.tempAllocator))
        c.shape = want;
}

float PhysicsWorld::characterHeight(CharacterId id) const {
    if (!impl_ || id >= impl_->characters.size()) return 0.0f;
    const Impl::Character& c = impl_->characters[id];
    return std::min(c.minHeight + static_cast<float>(c.shape) * kCharacterHeightStep, c.fullHeight);
}

RagdollId PhysicsWorld::createRagdoll(const std::vector<RagdollPart>& parts, uint32_t group, std::vector<BodyId>& bodies) {
    bodies.clear();
    if (!impl_ || parts.empty()) return kInvalidRagdoll;
    Impl& m = *impl_;
    if (m.ragdolls.size() >= m.ragdolls.capacity()) {
        LOG_ERROR("physics: ragdoll limit ({}) reached", m.ragdolls.capacity());
        return kInvalidRagdoll;
    }
    const size_t freeSlots = static_cast<size_t>(
        std::count_if(m.slots.begin(), m.slots.end(), [](const Impl::Slot& s) { return s.body.IsInvalid(); }));
    if (freeSlots < parts.size()) {
        LOG_ERROR("physics: body limit ({}) reached", m.slots.size());
        return kInvalidRagdoll;
    }
    JPH::Ref<JPH::Skeleton> skeleton = new JPH::Skeleton();
    JPH::Ref<JPH::RagdollSettings> settings = new JPH::RagdollSettings();
    settings->mSkeleton = skeleton;
    settings->mParts.resize(parts.size());
    for (size_t i = 0; i < parts.size(); ++i) {
        const RagdollPart& p = parts[i];
        char name[16];
        std::snprintf(name, sizeof(name), "part%zu", i);
        skeleton->AddJoint(name, p.parent);
        BodyDesc d = p.body;
        d.dynamic = true;
        d.kinematic = false;
        d.collides = true;
        d.collisionGroup = std::max(group, 1u);
        JPH::BodyCreationSettings bcs = bodySettings(d, m.settings, m.sameGroup);
        bcs.mAllowSleeping = false;  // its motors keep driving it
        static_cast<JPH::BodyCreationSettings&>(settings->mParts[i]) = bcs;
        if (p.parent < 0) continue;
        JPH::Ref<JPH::SwingTwistConstraintSettings> c = new JPH::SwingTwistConstraintSettings();
        c->mSpace = JPH::EConstraintSpace::WorldSpace;
        c->mPosition1 = c->mPosition2 = JPH::RVec3(toJolt(p.pivot));
        c->mTwistAxis1 = toJolt(math::normalize(p.twistAxis1));
        c->mPlaneAxis1 = toJolt(math::normalize(p.planeAxis1));
        c->mTwistAxis2 = toJolt(math::normalize(p.twistAxis2));
        c->mPlaneAxis2 = toJolt(math::normalize(p.planeAxis2));
        c->mSwingType = JPH::ESwingType::Pyramid;  // each swing axis its own range
        c->mNormalHalfConeAngle = p.normalHalfCone;
        c->mPlaneHalfConeAngle = p.planeHalfCone;
        c->mTwistMinAngle = p.twistMin;
        c->mTwistMaxAngle = p.twistMax;
        c->mSwingMotorSettings = c->mTwistMotorSettings = toJolt(p.motor);
        settings->mParts[i].mToParent = c;
    }
    settings->CalculateBodyIndexToConstraintIndex();
    settings->CalculateConstraintIndexToBodyIdxPair();
    JPH::Ref<JPH::Ragdoll> ragdoll = settings->CreateRagdoll(std::max(group, 1u), 0, &m.system);
    if (!ragdoll) {
        LOG_ERROR("physics: Jolt could not create a ragdoll");
        return kInvalidRagdoll;
    }
    for (JPH::uint i = 0; i < ragdoll->GetConstraintCount(); ++i)
        applySteps(*ragdoll->GetConstraint(static_cast<int>(i)), m.settings);
    ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
    Impl::RagdollSlot r;
    r.ragdoll = ragdoll;
    r.pose.SetSkeleton(skeleton);
    r.hold.assign(parts.size(), math::Vec3(0.0f));
    for (size_t i = 0; i < parts.size(); ++i)
        if (const RagdollPart& p = parts[i]; p.parent >= 0) {
            r.jointPart.push_back(static_cast<int>(i));
            r.baseTorque.push_back(std::max(p.motor.maxTorque, 0.0f));
            Impl::RagdollSlot::Directional d;
            if (p.directionalAxis >= 0 && p.directionalAxis <= 2) {
                d.axis = p.directionalAxis;
                d.plus = std::max(p.plusTorque, 0.0f);
                d.minus = std::max(p.minusTorque, 0.0f);
            }
            r.directional.push_back(d);
        }
    for (size_t i = 0; i < parts.size(); ++i) {
        auto free = std::find_if(m.slots.begin(), m.slots.end(), [](const Impl::Slot& s) { return s.body.IsInvalid(); });
        free->body = ragdoll->GetBodyID(static_cast<int>(i));
        m.capture(*free);
        free->previous = free->current;
        const BodyId id = static_cast<BodyId>(free - m.slots.begin());
        r.bodies.push_back(id);
        bodies.push_back(id);
    }
    m.ragdolls.push_back(std::move(r));
    const RagdollId id = static_cast<RagdollId>(m.ragdolls.size() - 1);
    setRagdollStrength(id, 1.0f);
    return id;
}

namespace {

// One joint's muscles this tick. The gravity hold (`hold`, world: the torque
// the part needs about its joint) and the pose motor share each axis's
// strength (base, or the directional pair on `axis`; at the body's strength
// `k`): the hold stays within it and the motor gets what is left, so the two
// never give more than the muscle. Jolt keeps a min and a max on the twist
// axis. Both swing axes share one limit: a directional joint's is what is
// left for the muscle the motor pulls with about its axis now (the spring's
// push toward the target less its damping of the turn; frequency mode: the
// gap's way alone), any other joint's the least left on either swing axis.
void applyMuscles(JPH::SwingTwistConstraint& c, float base, int axis, float plus, float minus, math::Vec3 hold,
                  float k) {
    JPH::Body* b1 = c.GetBody1();
    JPH::Body* b2 = c.GetBody2();
    const JPH::Quat toWorld = b2->GetRotation() * c.GetConstraintToBody2Matrix().GetQuaternion();
    float lo[3], hi[3];
    for (int a = 0; a < 3; ++a) {
        lo[a] = -base * k;
        hi[a] = base * k;
    }
    if (axis >= 0 && axis <= 2) {
        lo[axis] = -minus * k;
        hi[axis] = plus * k;
    }
    JPH::Vec3 h = toWorld.Conjugated() * toJolt(hold * k);
    for (int a = 0; a < 3; ++a) h.SetComponent(static_cast<JPH::uint>(a), std::clamp(h[a], lo[a], hi[a]));
    const JPH::Vec3 applied = toWorld * h;
    if (!applied.IsNearZero()) {
        b2->AddTorque(applied);
        b1->AddTorque(-applied);
    }
    c.GetTwistMotorSettings().SetTorqueLimits(lo[0] - h[0], hi[0] - h[0]);
    float swing = 0.0f;
    if (axis >= 1 && axis <= 2) {
        const JPH::Quat q = c.GetRotationInConstraintSpace();
        JPH::Quat target = c.GetTargetOrientationCS();
        if (q.Dot(target) < 0.0f) target = -target;  // the shortest way, as Jolt drives it
        const float gap = 2.0f * (q.Conjugated() * target).GetXYZ()[axis];  // rad about the axis, body 2's frame
        const float turn = (toWorld.Conjugated() * (b2->GetAngularVelocity() - b1->GetAngularVelocity()))[axis];
        const JPH::SpringSettings& s = c.GetSwingMotorSettings().mSpringSettings;
        const float pull = s.mMode == JPH::ESpringMode::StiffnessAndDamping ? s.mStiffness * gap - s.mDamping * turn : gap;
        swing = pull >= 0.0f ? hi[axis] - h[axis] : h[axis] - lo[axis];
    } else {
        swing = std::min({hi[1] - h[1], h[1] - lo[1], hi[2] - h[2], h[2] - lo[2]});
    }
    c.GetSwingMotorSettings().SetTorqueLimit(std::max(swing, 0.0f));
}

} // namespace

void PhysicsWorld::driveRagdollToPose(RagdollId id, const math::Quat* rotations, int count) {
    if (!impl_ || id >= impl_->ragdolls.size() || !rotations) return;
    Impl::RagdollSlot& r = impl_->ragdolls[id];
    const int n = std::min(count, static_cast<int>(r.pose.GetJointCount()));
    for (int i = 0; i < n; ++i) r.pose.GetJoint(i).mRotation = toJolt(math::normalize(rotations[i]));
    r.ragdoll->DriveToPoseUsingMotors(r.pose);
    const size_t joints = std::min({static_cast<size_t>(r.ragdoll->GetConstraintCount()), r.baseTorque.size(),
                                    r.directional.size(), r.jointPart.size()});
    for (size_t i = 0; i < joints; ++i) {
        JPH::TwoBodyConstraint* c = r.ragdoll->GetConstraint(static_cast<int>(i));
        if (!c || c->GetSubType() != JPH::EConstraintSubType::SwingTwist) continue;
        const Impl::RagdollSlot::Directional& d = r.directional[i];
        applyMuscles(*static_cast<JPH::SwingTwistConstraint*>(c), r.baseTorque[i], d.axis, d.plus, d.minus,
                     r.hold[static_cast<size_t>(r.jointPart[i])], r.scale);
    }
}

void PhysicsWorld::setRagdollHold(RagdollId id, const math::Vec3* torques, int count) {
    if (!impl_ || id >= impl_->ragdolls.size() || !torques) return;
    Impl::RagdollSlot& r = impl_->ragdolls[id];
    const size_t n = std::min(static_cast<size_t>(std::max(count, 0)), r.hold.size());
    for (size_t i = 0; i < n; ++i) r.hold[i] = torques[i];
}

bool PhysicsWorld::ragdollJointState(RagdollId id, int part, float dt, RagdollJointState& out) const {
    if (!impl_ || id >= impl_->ragdolls.size() || dt <= 0.0f) return false;
    const Impl::RagdollSlot& r = impl_->ragdolls[id];
    const size_t n = std::min(static_cast<size_t>(r.ragdoll->GetConstraintCount()), r.jointPart.size());
    for (size_t i = 0; i < n; ++i) {
        if (r.jointPart[i] != part) continue;
        const JPH::TwoBodyConstraint* c = r.ragdoll->GetConstraint(static_cast<int>(i));
        if (!c || c->GetSubType() != JPH::EConstraintSubType::SwingTwist) return false;
        const JPH::SwingTwistConstraint* st = static_cast<const JPH::SwingTwistConstraint*>(c);
        const JPH::Vec3 m = st->GetTotalLambdaMotor() / dt;
        out.motor = math::Vec3(m.GetX(), m.GetY(), m.GetZ());
        out.twistLimit = st->GetTwistMotorSettings().mMaxTorqueLimit;
        out.swingLimit = st->GetSwingMotorSettings().mMaxTorqueLimit;
        out.range = math::Vec3(st->GetTotalLambdaTwist(), st->GetTotalLambdaSwingY(), st->GetTotalLambdaSwingZ()) / dt;
        // The pose's target in constraint space (as SetTargetOrientationBS
        // turns it), against Jolt's clamped target and the joint now.
        const JPH::Quat asked = st->GetConstraintToBody1Matrix().GetQuaternion().Conjugated() *
                                r.pose.GetJoint(part).mRotation * st->GetConstraintToBody2Matrix().GetQuaternion();
        const JPH::Quat target = st->GetTargetOrientationCS(), now = st->GetRotationInConstraintSpace();
        auto apart = [](JPH::QuatArg a, JPH::QuatArg b) {
            return 2.0f * std::acos(std::clamp(std::abs(a.Dot(b)), 0.0f, 1.0f));
        };
        out.clamped = apart(asked, target);
        out.tracking = apart(target, now);
        return true;
    }
    return false;
}

void PhysicsWorld::setRagdollStrength(RagdollId id, float scale) {
    if (!impl_ || id >= impl_->ragdolls.size()) return;
    Impl::RagdollSlot& r = impl_->ragdolls[id];
    constexpr float kMostScale = 10.0f;  // above 1: more of the muscles recruited (a faster pace)
    const float k = std::clamp(scale, 0.0f, kMostScale);
    r.scale = k;
    const size_t n = std::min({static_cast<size_t>(r.ragdoll->GetConstraintCount()), r.baseTorque.size(), r.directional.size()});
    for (size_t i = 0; i < n; ++i) {
        JPH::TwoBodyConstraint* c = r.ragdoll->GetConstraint(static_cast<int>(i));
        if (!c || c->GetSubType() != JPH::EConstraintSubType::SwingTwist) continue;
        JPH::SwingTwistConstraint* st = static_cast<JPH::SwingTwistConstraint*>(c);
        st->GetSwingMotorSettings().SetTorqueLimit(r.baseTorque[i] * k);
        st->GetTwistMotorSettings().SetTorqueLimit(r.baseTorque[i] * k);
    }
}

JointId PhysicsWorld::createSupport(BodyId a, BodyId b, const MotorDesc& linear, const MotorDesc& angular,
                                    float freeDown) {
    if (!impl_ || !impl_->slot(a) || !impl_->slot(b)) return kInvalidJoint;
    Impl& m = *impl_;
    if (m.hinges.size() >= m.hinges.capacity()) {
        LOG_ERROR("physics: joint limit ({}) reached", m.hinges.capacity());
        return kInvalidJoint;
    }
    // Every axis free (the settings' defaults), each driven by its motor.
    JPH::SixDOFConstraintSettings s;
    s.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    using A = JPH::SixDOFConstraintSettings::EAxis;
    for (A axis : {A::TranslationX, A::TranslationY, A::TranslationZ}) {
        JPH::MotorSettings& motor = s.mMotorSettings[axis];
        motor = toJolt(linear);  // stiffness: N/m, damping N s/m
        motor.SetForceLimit(std::max(linear.maxTorque, 0.0f));
    }
    for (A axis : {A::RotationX, A::RotationY, A::RotationZ}) s.mMotorSettings[axis] = toJolt(angular);
    JPH::Ref<JPH::SixDOFConstraint> c = makeJoint<JPH::SixDOFConstraint>(m.system, m.slots[a].body, m.slots[b].body, s);
    if (!c) return kInvalidJoint;
    applySteps(*c, m.settings);
    for (int axis = 0; axis < 6; ++axis) c->SetMotorState(static_cast<A>(axis), JPH::EMotorState::Position);
    if (freeDown > 0.0f) {
        // Up/down: no motor; b moves freely from its target down to freeDown
        // below it, then Jolt's soft limit (the linear spring) holds it.
        c->SetMotorState(A::TranslationY, JPH::EMotorState::Off);
        c->SetLimitsSpringSettings(A::TranslationY, JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping,
                                                                        std::max(linear.stiffness, 0.0f),
                                                                        std::max(linear.damping, 0.0f)));
        const JPH::BodyInterface& bi = m.system.GetBodyInterface();
        const JPH::Vec3 offset = bi.GetRotation(m.slots[a].body).Conjugated() *
                                 JPH::Vec3(bi.GetCenterOfMassPosition(m.slots[b].body) -
                                           bi.GetCenterOfMassPosition(m.slots[a].body));
        c->SetTranslationLimits(JPH::Vec3(-FLT_MAX, offset.GetY() - freeDown, -FLT_MAX),
                                JPH::Vec3(FLT_MAX, offset.GetY(), FLT_MAX));
    }
    m.system.AddConstraint(c);
    m.hinges.push_back(nullptr);
    m.swingTwists.push_back(nullptr);
    m.sixDofs.push_back(c);
    return static_cast<JointId>(m.hinges.size() - 1);
}

void PhysicsWorld::setSupportTarget(JointId id, math::Vec3 offset, math::Quat rotation) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    JPH::SixDOFConstraint& c = *impl_->sixDofs[id];
    c.SetTargetPositionCS(toJolt(offset));
    c.SetTargetOrientationCS(toJolt(math::normalize(rotation)));
    using A = JPH::SixDOFConstraintSettings::EAxis;
    if (c.GetMotorState(A::TranslationY) == JPH::EMotorState::Off) {  // free down: the band follows the target
        const float freeDown = c.GetLimitsMax(A::TranslationY) - c.GetLimitsMin(A::TranslationY);
        c.SetTranslationLimits(JPH::Vec3(-FLT_MAX, offset.y - freeDown, -FLT_MAX), JPH::Vec3(FLT_MAX, offset.y, FLT_MAX));
    }
}

void PhysicsWorld::setSupportLimits(JointId id, float maxForce, float maxTorque) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id]) return;
    JPH::SixDOFConstraint& c = *impl_->sixDofs[id];
    using A = JPH::SixDOFConstraintSettings::EAxis;
    for (A axis : {A::TranslationX, A::TranslationY, A::TranslationZ})
        c.GetMotorSettings(axis).SetForceLimit(std::max(maxForce, 0.0f));
    for (A axis : {A::RotationX, A::RotationY, A::RotationZ})
        c.GetMotorSettings(axis).SetTorqueLimit(std::max(maxTorque, 0.0f));
}

bool PhysicsWorld::supportLoad(JointId id, float dt, math::Vec3& force, float& torque) const {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id] || dt <= 0.0f) return false;
    const JPH::SixDOFConstraint& c = *impl_->sixDofs[id];
    // Jolt's translation parts push body 2 along body 1's constraint axes.
    force = fromJolt(JPH::Vec3(c.GetTotalLambdaPosition() + c.GetTotalLambdaMotorTranslation())) / dt;
    torque = (c.GetTotalLambdaRotation() + c.GetTotalLambdaMotorRotation()).Length() / dt;
    return true;
}

} // namespace physics
