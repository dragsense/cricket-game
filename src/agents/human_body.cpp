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

} // namespace

namespace {

// The bone's name without its side suffix (thigh_l -> thigh).
std::string baseName(const std::string& name) {
    const size_t n = name.size();
    return n > 2 && name[n - 2] == '_' && (name[n - 1] == 'l' || name[n - 1] == 'r') ? name.substr(0, n - 2) : name;
}

float strengthOf(const MuscleSettings& m, const std::string& name) {
    const std::string base = baseName(name);
    for (const auto& [bone, torque] : m.maxTorque)
        if (bone == base) return torque;
    return 0.0f;
}

} // namespace

bool HumanBody::init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
                     const ik::JointLimits& limits, const MuscleSettings& muscles, const math::Transform& root,
                     uint32_t group) {
    parts_.clear();
    totalMass_ = 0.0f;
    std::vector<math::Mat4> models(static_cast<size_t>(rig.numJoints()));
    if (!rig.localToModel(rig.restPose(), models)) return false;
    const float h = rig.height();

    // Bodies: every bone with mass; partOf[joint] = its part (or its nearest
    // ancestor's, for a bone without mass).
    std::vector<int> partOf(static_cast<size_t>(rig.numJoints()), -1);
    partOf_.assign(static_cast<size_t>(rig.numJoints()), -1);
    for (int j = 0; j < rig.numJoints(); ++j) {  // parents come before children
        const rig::BoneDef& bone = def.bones[static_cast<size_t>(rig.boneDefIndex(j))];
        const int parentJoint = rig.parent(j);
        const int parentPart = parentJoint >= 0 ? partOf[static_cast<size_t>(parentJoint)] : -1;
        if (bone.massPercent <= 0.0f) {
            partOf[static_cast<size_t>(j)] = parentPart;  // rides its parent
            continue;
        }
        math::Transform local;
        physics::BodyDesc d = shapeFor(bone.visual, h, local);
        d.mass = bone.massPercent * 0.01f * def.bodyMass;
        d.collisionGroup = group;
        d.kinematic = true;  // for now (the user, 2026-11-05): held at the planned pose (follow), so it does not drop
        d.pose = root * math::toTransform(models[static_cast<size_t>(j)]) * local;
        Part p;
        p.joint = j;
        p.mass = d.mass;
        p.parent = parentPart;
        p.shape = local;
        p.shapeType = d.shape;
        p.radius = d.radius;
        p.halfHeight = d.halfHeight;
        p.halfExtents = d.halfExtents;
        p.body = world.createBody(d);
        if (p.body == physics::kInvalidBody) return false;
        partOf[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        partOf_[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        parts_.push_back(p);
        totalMass_ += d.mass;
    }

    // Joints: each part to its parent part at its own joint, with its real
    // range (human_joints.json). At rest every bone frame has the character's
    // axes, so the joint's axes are the root's.
    for (Part& p : parts_) {
        if (p.parent < 0) continue;
        const math::Vec3 pivot = root.transformPoint(math::Vec3(models[static_cast<size_t>(p.joint)][3]));
        const ik::JointLimit& lim = limits.limit(p.joint);
        const physics::BodyId parent = parts_[static_cast<size_t>(p.parent)].body;
        if (lim.type == ik::LimitType::Hinge) {
            physics::HingeDesc d;
            d.pivot = pivot;
            d.axis = root.transformDirection(axisVec(lim.axis));
            d.normal = root.transformDirection(axisVec((lim.axis + 1) % 3));
            d.minAngle = lim.twistMin;
            d.maxAngle = lim.twistMax;
            p.link = world.createHinge(parent, p.body, d);
            p.hinge = true;
            p.frameX = axisVec(lim.axis);
        } else {
            // Joint frame: X = the twist axis, Y = the first swing axis, Z = X x Y
            // (the second swing axis or its opposite: then its range flips).
            physics::SixDofDesc d;
            d.pivot = pivot;
            int a1 = 0, a2 = 0;
            if (lim.type == ik::LimitType::SwingTwist) {
                swingAxes(lim.axis, a1, a2);
                const math::Vec3 x = axisVec(lim.axis), y = axisVec(a1);
                const bool flip = math::dot(math::cross(x, y), axisVec(a2)) < 0.0f;
                d.axisX = root.transformDirection(x);
                d.axisY = root.transformDirection(y);
                d.minAngles = math::Vec3(lim.twistMin, lim.swing1Min, flip ? -lim.swing2Max : lim.swing2Min);
                d.maxAngles = math::Vec3(lim.twistMax, lim.swing1Max, flip ? -lim.swing2Min : lim.swing2Max);
                p.frameX = x;
                p.frameY = y;
                p.frameZ = math::cross(x, y);
            } else {  // free
                d.axisX = root.transformDirection(math::kRight);
                d.axisY = root.transformDirection(math::kUp);
                d.minAngles = math::Vec3(-math::kPi);
                d.maxAngles = math::Vec3(math::kPi);
                p.frameX = math::kRight;
                p.frameY = math::kUp;
                p.frameZ = math::kForward;
            }
            p.link = world.createSixDof(parent, p.body, d);
        }
        if (p.link == physics::kInvalidJoint) return false;

        // Its muscle: no stronger than the joint really is, pulling as fast as
        // muscle activation allows (a critically damped drive with time
        // constant activationTime: f = 1 / (2 pi t)).
        const float strength = strengthOf(muscles, def.bones[static_cast<size_t>(rig.boneDefIndex(p.joint))].name);
        if (strength > 0.0f) {
            physics::MotorDesc m;
            m.frequency = 1.0f / (math::kTwoPi * std::max(muscles.activationTime, 1e-3f));
            m.damping = muscles.damping;
            m.maxTorque = strength;
            world.setJointMotor(p.link, m, true);
            p.muscle = true;
        }
    }
    LOG_INFO("human body: {} parts, {:.1f} kg", parts_.size(), totalMass_);
    return true;
}

void HumanBody::follow(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root,
                       float dt) const {
    for (const Part& p : parts_)
        world.moveKinematic(p.body, root * math::toTransform(models[static_cast<size_t>(p.joint)]) * p.shape, dt);
}

void HumanBody::setTargets(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models) const {
    for (const Part& p : parts_) {
        if (!p.muscle) continue;
        // The bone relative to its parent part's bone in the planned pose.
        const math::Quat parent = math::toTransform(models[static_cast<size_t>(parts_[static_cast<size_t>(p.parent)].joint)]).rotation;
        math::Quat q = math::normalize(math::inverse(parent) * math::toTransform(models[static_cast<size_t>(p.joint)]).rotation);
        if (q.w < 0.0f) q = -q;
        const math::Vec3 v(q.x, q.y, q.z);
        if (p.hinge) {
            world.setHingeTarget(p.link, 2.0f * std::atan2(math::dot(v, p.frameX), q.w));
        } else {
            // In the joint's frame (its axes in the bone frame).
            world.setSixDofTarget(p.link, math::Quat(q.w, math::dot(v, p.frameX), math::dot(v, p.frameY), math::dot(v, p.frameZ)));
        }
    }
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

math::Vec3 HumanBody::centreOfMass(const physics::PhysicsWorld& world) const {
    math::Vec3 sum(0.0f);
    for (const Part& p : parts_) sum += world.transform(p.body).position * p.mass;
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

} // namespace agents
