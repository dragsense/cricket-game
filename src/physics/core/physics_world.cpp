#include "physics/core/physics_world.h"

#include "core/config.h"
#include "core/log.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
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
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
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

BodyId PhysicsWorld::createBody(const BodyDesc& d) {
    if (!impl_) return kInvalidBody;
    Impl& m = *impl_;
    auto free = std::find_if(m.slots.begin(), m.slots.end(), [](const Impl::Slot& s) { return s.body.IsInvalid(); });
    if (free == m.slots.end()) {
        LOG_ERROR("physics: body limit ({}) reached", m.slots.size());
        return kInvalidBody;
    }

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
    if (d.collisionGroup > 0) bcs.mCollisionGroup = JPH::CollisionGroup(m.sameGroup, d.collisionGroup, 0);
    bcs.mLinearDamping = m.settings.linearDamping;
    bcs.mAngularDamping = m.settings.angularDamping;
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

void PhysicsWorld::addTorque(BodyId id, math::Vec3 torque) {
    if (!impl_ || !impl_->slot(id)) return;
    impl_->system.GetBodyInterface().AddTorque(impl_->slots[id].body, toJolt(torque));
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

void PhysicsWorld::setSixDofAxisSpring(JointId id, int axis, float offset, float stiffness, float damping,
                                       float maxTorque) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id] || axis < 0 || axis > 2) return;
    JPH::SixDOFConstraint* c = impl_->sixDofs[id];
    using A = JPH::SixDOFConstraintSettings::EAxis;
    const A a = static_cast<A>(static_cast<int>(A::RotationX) + axis);
    const bool othersHeld = [&] {  // another axis drives toward the target orientation
        for (int k = 0; k < 3; ++k)
            if (k != axis && c->GetMotorState(static_cast<A>(static_cast<int>(A::RotationX) + k)) == JPH::EMotorState::Position)
                return true;
        return false;
    }();
    c->GetMotorSettings(a) = springMotor(stiffness, damping, maxTorque);
    c->SetMotorState(a, JPH::EMotorState::Position);
    // Target: this axis turned by `offset` from now. The target orientation
    // is shared by the axes: those driving to a position keep what they have
    // still to go. Measured the way Jolt measures a motor's error (the turn
    // from now to the target in the joint's own frame, now^-1 * target), so
    // each axis gets its own offset at any joint angle (adding to the
    // rotation's components instead leaked the torque to the other axes:
    // the one-foot ankle got ~80 % of its roll torque, 2026-11-08).
    const JPH::Quat now = c->GetRotationInConstraintSpace();
    math::Vec3 toGo(0.0f);
    if (othersHeld) toGo = math::rotationVector(fromJolt(now.Conjugated() * c->GetTargetOrientationCS()));
    toGo[axis] = offset;
    const float angle = math::length(toGo);
    c->SetTargetOrientationCS(now * toJolt(angle > 1e-6f ? math::fromAxisAngle(toGo / angle, angle) : math::identityQuat()));
}

void PhysicsWorld::setSixDofAxisFree(JointId id, int axis) {
    if (!impl_ || id >= impl_->sixDofs.size() || !impl_->sixDofs[id] || axis < 0 || axis > 2) return;
    using A = JPH::SixDOFConstraintSettings::EAxis;
    impl_->sixDofs[id]->SetMotorState(static_cast<A>(static_cast<int>(A::RotationX) + axis), JPH::EMotorState::Off);
}

void PhysicsWorld::setHingeSpring(JointId id, float offset, float stiffness, float damping, float maxTorque) {
    if (!impl_ || id >= impl_->hinges.size() || !impl_->hinges[id]) return;
    JPH::HingeConstraint* h = impl_->hinges[id];
    h->GetMotorSettings() = springMotor(stiffness, damping, maxTorque);
    h->SetMotorState(JPH::EMotorState::Position);
    h->SetTargetAngle(h->GetCurrentAngle() + offset);
}

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

bool PhysicsWorld::awake(BodyId id) const {
    const Impl::Slot* s = impl_ ? impl_->slot(id) : nullptr;
    return s && impl_->system.GetBodyInterfaceNoLock().IsActive(s->body);
}

} // namespace physics
