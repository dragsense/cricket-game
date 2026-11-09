#include "agents/human_body.h"

#include "core/log.h"

#include <algorithm>
#include <cmath>

namespace agents {

namespace {

math::Vec3 axisVec(int axis) { return axis == 0 ? math::kRight : axis == 1 ? math::kUp : math::kForward; }

// The two swing axes beside the twist axis, in x, y, z order (as
// ik::clampRotation reads swing1 / swing2).
void swingAxes(int twist, int& first, int& second) {
    first = twist == 0 ? 1 : 0;
    second = twist == 2 ? 1 : 2;
}

// The bone's shape on its body: Jolt description (shape, size, mass) and its
// pose in the bone's frame (character axes at rest), metres.
physics::BodyDesc shapeFor(const rig::BoneVisual& v, float h, math::Transform& local) {
    physics::BodyDesc d;
    local = math::Transform::identity();
    if (v.shape == rig::VisualShape::Capsule) {
        const math::Vec3 from = v.from * h, to = v.to * h;
        const float len = math::length(to - from);
        local.position = 0.5f * (from + to);
        local.rotation = math::fromTo(math::kUp, math::safeNormalize(to - from, math::kUp));
        d.shape = len > 1e-4f ? physics::ShapeType::Capsule : physics::ShapeType::Sphere;
        d.halfHeight = 0.5f * len;
        d.radius = std::max(v.radius * h, 0.005f);
    } else if (v.shape == rig::VisualShape::Box) {
        local.position = v.center * h;
        d.shape = physics::ShapeType::Box;
        const math::Vec3 half = 0.5f * v.size * h;
        d.halfExtents = math::Vec3(std::max(half.x, 0.005f), std::max(half.y, 0.005f), std::max(half.z, 0.005f));
    } else {
        d.shape = physics::ShapeType::Sphere;
        d.radius = 0.02f;
    }
    return d;
}

// The bone's name without its side suffix (thigh_l -> thigh).
std::string baseName(const std::string& name) {
    const size_t n = name.size();
    return n > 2 && name[n - 2] == '_' && (name[n - 1] == 'l' || name[n - 1] == 'r') ? name.substr(0, n - 2) : name;
}

// A per-bone value (by name without its side), 0 if not listed.
float valueOf(const std::vector<std::pair<std::string, float>>& list, const std::string& name) {
    const std::string base = baseName(name);
    for (const auto& [bone, value] : list)
        if (bone == base) return value;
    return 0.0f;
}

// Jolt's swing limits are symmetric (half cones) and at most a half turn.
float halfRange(float lo, float hi) { return std::clamp(0.5f * (hi - lo), 0.0f, math::kPi); }

} // namespace

bool HumanBody::init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
                     const ik::JointLimits& limits, const MuscleSettings& muscles, const math::Transform& root,
                     uint32_t group) {
    parts_.clear();
    totalMass_ = 0.0f;
    std::vector<math::Mat4> models(static_cast<size_t>(rig.numJoints()));
    if (!rig.localToModel(rig.restPose(), models)) return false;
    const float h = rig.height();
    // Its muscle: no stronger than the joint really is, pulling as fast as
    // muscle activation allows (a critically damped drive with time constant
    // activationTime: f = 1 / (2 pi t)).
    const float frequency = 1.0f / (math::kTwoPi * std::max(muscles.activationTime, 1e-3f));

    // Parts: every bone with mass; partOf_[joint] = its part (or its nearest
    // ancestor's, for a bone without mass).
    std::vector<physics::RagdollPart> ragdoll;
    std::vector<int> owner(static_cast<size_t>(rig.numJoints()), -1);
    partOf_.assign(static_cast<size_t>(rig.numJoints()), -1);
    for (int j = 0; j < rig.numJoints(); ++j) {  // parents come before children
        const rig::BoneDef& bone = def.bones[static_cast<size_t>(rig.boneDefIndex(j))];
        const int parentJoint = rig.parent(j);
        const int parentPart = parentJoint >= 0 ? owner[static_cast<size_t>(parentJoint)] : -1;
        if (bone.massPercent <= 0.0f) {
            owner[static_cast<size_t>(j)] = parentPart;  // rides its parent
            continue;
        }
        math::Transform local;
        physics::RagdollPart r;
        r.body = shapeFor(bone.visual, h, local);
        r.body.mass = bone.massPercent * 0.01f * def.bodyMass;
        r.body.pose = root * math::toTransform(models[static_cast<size_t>(j)]) * local;
        r.parent = parentPart;
        Part p;
        p.joint = j;
        p.mass = r.body.mass;
        p.parent = parentPart;
        p.shape = local;
        p.shapeType = r.body.shape;
        p.radius = r.body.radius;
        p.halfHeight = r.body.halfHeight;
        p.halfExtents = r.body.halfExtents;
        if (parentPart >= 0) {
            // Its joint, in the world as built (at rest every bone frame has
            // the character's axes). Jolt's joint frame: X twist, Y normal,
            // Z plane; the part's frame is the joint's rest frame, the
            // parent's is turned to the middle of each swing range.
            r.pivot = root.transformPoint(math::Vec3(models[static_cast<size_t>(j)][3]));
            const ik::JointLimit& lim = limits.limit(j);
            math::Vec3 x = math::kRight, y = math::kUp;
            math::Quat centre = math::identityQuat();
            if (lim.type == ik::LimitType::Hinge) {
                x = axisVec(lim.axis);
                y = axisVec((lim.axis + 1) % 3);
                r.twistMin = lim.twistMin;
                r.twistMax = lim.twistMax;
                p.hinge = true;  // both swings locked (0)
            } else if (lim.type == ik::LimitType::SwingTwist) {
                int a1 = 0, a2 = 0;
                swingAxes(lim.axis, a1, a2);
                x = axisVec(lim.axis);
                y = axisVec(a1);
                // Z = X x Y: the second swing axis or its opposite (then its range flips).
                const bool flip = math::dot(math::cross(x, y), axisVec(a2)) < 0.0f;
                const float zMin = flip ? -lim.swing2Max : lim.swing2Min, zMax = flip ? -lim.swing2Min : lim.swing2Max;
                r.twistMin = lim.twistMin;
                r.twistMax = lim.twistMax;
                r.planeHalfCone = halfRange(lim.swing1Min, lim.swing1Max);  // swing about Y
                r.normalHalfCone = halfRange(zMin, zMax);                   // swing about Z
                centre = math::fromAxisAngle(y, 0.5f * (lim.swing1Min + lim.swing1Max)) *
                         math::fromAxisAngle(math::cross(x, y), 0.5f * (zMin + zMax));
            } else {  // free
                r.twistMin = -math::kPi;
                r.twistMax = math::kPi;
                r.planeHalfCone = r.normalHalfCone = math::kPi;
            }
            const math::Vec3 z = math::cross(x, y);
            r.twistAxis2 = root.transformDirection(x);
            r.planeAxis2 = root.transformDirection(z);
            r.twistAxis1 = root.transformDirection(centre * x);
            r.planeAxis1 = root.transformDirection(centre * z);
            p.frameX = x;
            p.frameY = y;
            p.frameZ = z;
            const float strength = valueOf(muscles.maxTorque, bone.name);
            r.motor.frequency = frequency;
            r.motor.damping = muscles.damping;
            r.motor.maxTorque = strength;  // 0: no muscle (the joint hangs free)
            r.motor.stiffness = valueOf(muscles.stiffness, bone.name);
            if (r.motor.stiffness > 0.0f) r.motor.damping = valueOf(muscles.jointDamping, bone.name);
            p.muscle = strength > 0.0f;
            p.strength = strength;
        }
        owner[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        partOf_[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        parts_.push_back(p);
        ragdoll.push_back(r);
        totalMass_ += p.mass;
    }
    std::vector<physics::BodyId> bodies;
    ragdoll_ = world.createRagdoll(ragdoll, group, bodies);
    if (ragdoll_ == physics::kInvalidRagdoll || bodies.size() != parts_.size()) {
        parts_.clear();
        return false;
    }
    for (size_t i = 0; i < parts_.size(); ++i) parts_[i].body = bodies[i];
    rotations_.assign(parts_.size(), math::identityQuat());
    beyondMass_.assign(parts_.size(), 0.0f);
    beyondMoment_.assign(parts_.size(), math::Vec3(0.0f));
    LOG_INFO("human body: Jolt ragdoll, {} parts, {:.1f} kg", parts_.size(), totalMass_);
    return true;
}

math::Transform HumanBody::partPose(int part, const std::vector<math::Mat4>& models) const {
    const Part& p = parts_[static_cast<size_t>(part)];
    return math::toTransform(models[static_cast<size_t>(p.joint)]) * p.shape;
}

void HumanBody::holdWeight(physics::PhysicsWorld& world) const {
    // Mass and moment of each part's subtree (children come after parents).
    for (size_t i = 0; i < parts_.size(); ++i) {
        beyondMass_[i] = parts_[i].mass;
        beyondMoment_[i] = world.transform(parts_[i].body).position * parts_[i].mass;
    }
    for (size_t i = parts_.size(); i-- > 0;) {
        const int parent = parts_[i].parent;
        if (parent < 0) continue;
        beyondMass_[static_cast<size_t>(parent)] += beyondMass_[i];
        beyondMoment_[static_cast<size_t>(parent)] += beyondMoment_[i];
    }
    const math::Vec3 g = world.world().gravity;
    for (size_t i = 0; i < parts_.size(); ++i) {
        const Part& p = parts_[i];
        if (p.parent < 0 || !p.muscle || beyondMass_[i] <= 0.0f) continue;
        // Gravity's turn on everything beyond the joint, about the joint.
        const math::Vec3 pivot = boneWorld(world, static_cast<int>(i)).position;
        const math::Vec3 centre = beyondMoment_[i] / beyondMass_[i];
        math::Vec3 hold = -math::cross(centre - pivot, g * beyondMass_[i]);
        const float size = math::length(hold);
        if (size > p.strength) hold *= p.strength / size;
        world.addTorque(p.body, hold);
        world.addTorque(parts_[static_cast<size_t>(p.parent)].body, -hold);
    }
}

void HumanBody::drivePose(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models) const {
    // Each part body's rotation relative to its parent's body (the root's
    // turn cancels, so character space will do).
    for (size_t i = 0; i < parts_.size(); ++i) {
        const Part& p = parts_[i];
        if (p.parent < 0) continue;
        const math::Quat child = partPose(static_cast<int>(i), models).rotation;
        const math::Quat parent = partPose(p.parent, models).rotation;
        rotations_[i] = math::inverse(parent) * child;
    }
    world.driveRagdollToPose(ragdoll_, rotations_.data(), static_cast<int>(rotations_.size()));
}

void HumanBody::reset(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root) const {
    for (size_t i = 0; i < parts_.size(); ++i)
        world.resetBody(parts_[i].body, root * partPose(static_cast<int>(i), models));
}

void HumanBody::boneModels(const physics::PhysicsWorld& world, float alpha, const rig::Rig& rig,
                           const rig::Pose& pose, const math::Transform& root,
                           std::vector<math::Mat4>& models) const {
    const math::Transform toRoot = math::inverse(root);
    for (int j = 0; j < rig.numJoints(); ++j) {  // parents come before children
        const int part = partOf_[static_cast<size_t>(j)];
        math::Transform t;
        if (part >= 0) {
            const Part& p = parts_[static_cast<size_t>(part)];
            t = toRoot * world.interpolated(p.body, alpha) * math::inverse(p.shape);
        } else {
            const int parent = rig.parent(j);
            const math::Transform up = parent >= 0 ? math::toTransform(models[static_cast<size_t>(parent)]) : math::Transform{};
            t = up * pose.locals[static_cast<size_t>(j)];
        }
        models[static_cast<size_t>(j)] = t.toMat4();
    }
}

void HumanBody::jointMasses(int numJoints, std::vector<float>& mass, std::vector<math::Vec3>& centre) const {
    mass.assign(static_cast<size_t>(std::max(numJoints, 0)), 0.0f);
    centre.assign(mass.size(), math::Vec3(0.0f));
    for (const Part& p : parts_) {
        if (p.joint < 0 || static_cast<size_t>(p.joint) >= mass.size()) continue;
        mass[static_cast<size_t>(p.joint)] = p.mass;
        centre[static_cast<size_t>(p.joint)] = p.shape.position;
    }
}

math::Vec3 HumanBody::centreOfMass(const physics::PhysicsWorld& world) const {
    math::Vec3 sum(0.0f);
    for (const Part& p : parts_) sum += world.transform(p.body).position * p.mass;
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

math::Vec3 HumanBody::centreOfMassVelocity(const physics::PhysicsWorld& world) const {
    math::Vec3 sum(0.0f);
    for (const Part& p : parts_) sum += world.linearVelocity(p.body) * p.mass;
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

math::Transform HumanBody::boneWorld(const physics::PhysicsWorld& world, int part) const {
    const Part& p = parts_[static_cast<size_t>(part)];
    return world.transform(p.body) * math::inverse(p.shape);
}

} // namespace agents
