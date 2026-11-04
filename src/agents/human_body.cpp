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

bool HumanBody::init(physics::PhysicsWorld& world, const rig::SkeletonDef& def, const rig::Rig& rig,
                     const ik::JointLimits& limits, const math::Transform& root, uint32_t group) {
    parts_.clear();
    totalMass_ = 0.0f;
    std::vector<math::Mat4> models(static_cast<size_t>(rig.numJoints()));
    if (!rig.localToModel(rig.restPose(), models)) return false;
    const float h = rig.height();

    // Bodies: every bone with mass; partOf[joint] = its part (or its nearest
    // ancestor's, for a bone without mass).
    std::vector<int> partOf(static_cast<size_t>(rig.numJoints()), -1);
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
        d.pose = root * math::toTransform(models[static_cast<size_t>(j)]) * local;
        Part p;
        p.joint = j;
        p.mass = d.mass;
        p.parent = parentPart;
        p.shape = local;
        p.body = world.createBody(d);
        if (p.body == physics::kInvalidBody) return false;
        partOf[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
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
            } else {  // free
                d.axisX = root.transformDirection(math::kRight);
                d.axisY = root.transformDirection(math::kUp);
                d.minAngles = math::Vec3(-math::kPi);
                d.maxAngles = math::Vec3(math::kPi);
            }
            p.link = world.createSixDof(parent, p.body, d);
        }
        if (p.link == physics::kInvalidJoint) return false;
    }
    LOG_INFO("human body: {} parts, {:.1f} kg", parts_.size(), totalMass_);
    return true;
}

math::Vec3 HumanBody::centreOfMass(const physics::PhysicsWorld& world) const {
    math::Vec3 sum(0.0f);
    for (const Part& p : parts_) sum += world.transform(p.body).position * p.mass;
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

} // namespace agents
