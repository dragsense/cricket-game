#include "agents/human_body.h"

#include "core/log.h"
#include "motion/reach.h"

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

// A per-bone value (by name without its side), 0 if not listed.
float valueOf(const std::vector<std::pair<std::string, float>>& list, const std::string& name) {
    const std::string base = baseName(name);
    for (const auto& [bone, value] : list)
        if (bone == base) return value;
    return 0.0f;
}

// A part's bone rotation on its parent part's bone (the rig's local
// rotation) as its joint's target: a hinge's angle, a SixDOF's rotation in
// the joint's frame.
void setJointTarget(physics::PhysicsWorld& world, const HumanBody::Part& p, math::Quat local, bool spring,
                    float maxTorque, float stiffness = 0.0f) {
    math::Quat q = math::normalize(local);
    if (q.w < 0.0f) q = -q;
    const math::Vec3 v(q.x, q.y, q.z);
    if (p.hinge) {
        const float angle = 2.0f * std::atan2(math::dot(v, p.frameX), q.w);
        if (spring)
            world.setHingeSpringTarget(p.link, angle, stiffness, p.damping, maxTorque);
        else
            world.setHingeTarget(p.link, angle);
        return;
    }
    const math::Quat inFrame(q.w, math::dot(v, p.frameX), math::dot(v, p.frameY), math::dot(v, p.frameZ));
    if (spring)
        world.setSixDofSpringTarget(p.link, inFrame, stiffness, p.damping, maxTorque);
    else
        world.setSixDofTarget(p.link, inFrame);
}

// A swing-twist limit as a SixDOF joint's ranges in its frame (X = the
// twist axis, Y = the first swing axis, Z = X x Y: the second swing axis or
// its opposite, then its range flips).
void sixDofRanges(const ik::JointLimit& lim, int a1, int a2, math::Vec3& lo, math::Vec3& hi) {
    const math::Vec3 x = axisVec(lim.axis), y = axisVec(a1);
    const bool flip = math::dot(math::cross(x, y), axisVec(a2)) < 0.0f;
    lo = math::Vec3(lim.twistMin, lim.swing1Min, flip ? -lim.swing2Max : lim.swing2Min);
    hi = math::Vec3(lim.twistMax, lim.swing1Max, flip ? -lim.swing2Min : lim.swing2Max);
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
                d.axisX = root.transformDirection(x);
                d.axisY = root.transformDirection(y);
                sixDofRanges(lim, a1, a2, d.minAngles, d.maxAngles);
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
        const std::string& bone = def.bones[static_cast<size_t>(rig.boneDefIndex(p.joint))].name;
        const float strength = valueOf(muscles.maxTorque, bone);
        p.stiffness = valueOf(muscles.stiffness, bone);
        p.stiffnessPerTorque = valueOf(muscles.stiffnessPerTorque, bone);
        p.damping = valueOf(muscles.jointDamping, bone);
        p.limit = lim;
        if (strength > 0.0f) {
            physics::MotorDesc m;
            m.frequency = 1.0f / (math::kTwoPi * std::max(muscles.activationTime, 1e-3f));
            m.damping = muscles.damping;
            m.maxTorque = strength;
            world.setJointMotor(p.link, m, true);
            p.muscle = true;
            p.strength = strength;
        }
    }
    for (Part& p : parts_)
        if (p.limit.coupling.joint >= 0) p.coupledPart = partOf_[static_cast<size_t>(p.limit.coupling.joint)];
    omega_.assign(parts_.size(), math::Vec3(0.0f));
    ordered_.assign(parts_.size(), 0);
    targeted_.assign(parts_.size(), 0);
    targetLocal_.assign(parts_.size(), math::identityQuat());
    paths_.assign(parts_.size(), JointPath{});
    // Each bone's length: to its farthest child joint at rest (its tip).
    lever_.assign(parts_.size(), 0.0f);
    for (size_t i = 0; i < parts_.size(); ++i) {
        const int j = parts_[i].joint;
        const math::Vec3 at(models[static_cast<size_t>(j)][3]);
        for (int c = 0; c < rig.numJoints(); ++c)
            if (rig.parent(c) == j)
                lever_[i] = std::max(lever_[i], math::distance(at, math::Vec3(models[static_cast<size_t>(c)][3])));
    }
    holding_.assign(parts_.size(), 0);
    heldRotation_.assign(parts_.size(), math::identityQuat());
    heldAngle_.assign(parts_.size(), 0.0f);
    hold_.assign(parts_.size(), 0.0f);
    beyondMass_.assign(parts_.size(), 0.0f);
    beyondMoment_.assign(parts_.size(), math::Vec3(0.0f));
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
        setJointTarget(world, p, math::inverse(parent) * math::toTransform(models[static_cast<size_t>(p.joint)]).rotation,
                       false, 0.0f);
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

void HumanBody::reset(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root) {
    for (const Part& p : parts_)
        world.resetBody(p.body, root * math::toTransform(models[static_cast<size_t>(p.joint)]) * p.shape);
    std::fill(holding_.begin(), holding_.end(), uint8_t{0});
}

math::Vec3 HumanBody::centreOfMassVelocity(const physics::PhysicsWorld& world) const {
    math::Vec3 sum(0.0f);
    for (const Part& p : parts_) sum += world.linearVelocity(p.body) * p.mass;
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

void HumanBody::setPhysics(physics::PhysicsWorld& world, bool on) {
    for (const Part& p : parts_) world.setKinematic(p.body, !on);
    std::fill(holding_.begin(), holding_.end(), uint8_t{0});
}

void HumanBody::compensateGravity(physics::PhysicsWorld& world) const {
    const math::Vec3 g = world.world().gravity;
    // Every part's weight, then (children come after parents) each part's
    // added into its parent's: the mass beyond each joint.
    for (size_t i = 0; i < parts_.size(); ++i) {
        beyondMass_[i] = parts_[i].mass;
        beyondMoment_[i] = parts_[i].mass * world.transform(parts_[i].body).position;
    }
    for (size_t i = parts_.size(); i-- > 0;) {
        const int parent = parts_[i].parent;
        if (parent < 0) continue;
        beyondMass_[static_cast<size_t>(parent)] += beyondMass_[i];
        beyondMoment_[static_cast<size_t>(parent)] += beyondMoment_[i];
    }
    for (size_t i = 0; i < parts_.size(); ++i) {
        const Part& p = parts_[i];
        hold_[i] = 0.0f;
        if (p.parent < 0 || !p.muscle || beyondMass_[i] <= 0.0f) continue;
        // The joint (the bone's origin) and gravity's torque about it.
        const math::Vec3 pivot = (world.transform(p.body) * math::inverse(p.shape)).position;
        const math::Vec3 centre = beyondMoment_[i] / beyondMass_[i];
        math::Vec3 hold = -math::cross(centre - pivot, beyondMass_[i] * g);
        const float size = math::length(hold);
        if (size > p.strength) hold *= p.strength / size;  // no stronger than the muscle
        hold_[i] = std::min(size, p.strength);
        world.addTorque(p.body, hold);
        world.addTorque(parts_[static_cast<size_t>(p.parent)].body, -hold);
    }
}

void HumanBody::drive(physics::PhysicsWorld& world, const motion::BodyQueue& queue, const motion::BodyGoals& goals,
                      const motion::BodyDeciderJoints& joints, const std::vector<math::Mat4>& models,
                      const MuscleTiming& timing, float dt, const uint8_t* free) {
    auto pos = [&](int j) { return math::Vec3(models[static_cast<size_t>(j)][3]); };
    // A range set by another joint (the ankle's dorsiflexion by the knee: the
    // gastrocnemius) follows that joint's angle now.
    for (const Part& p : parts_) {
        if (p.coupledPart < 0 || p.hinge || p.limit.type != ik::LimitType::SwingTwist) continue;
        const Part& c = parts_[static_cast<size_t>(p.coupledPart)];
        if (!c.hinge) continue;
        int a1 = 0, a2 = 0;
        swingAxes(p.limit.axis, a1, a2);
        math::Vec3 lo, hi;
        sixDofRanges(ik::coupledLimit(p.limit, world.hingeAngle(c.link)), a1, a2, lo, hi);
        world.setSixDofLimits(p.link, lo, hi);
    }
    auto rot = [&](int j) { return math::toTransform(models[static_cast<size_t>(j)]).rotation; };
    for (math::Vec3& w : omega_) w = math::Vec3(0.0f);
    std::fill(ordered_.begin(), ordered_.end(), uint8_t{0});
    std::fill(targeted_.begin(), targeted_.end(), uint8_t{0});

    // The brain's target pose: each planned joint's bone on its parent part's
    // bone, both as planned (the clavicle rides the chest: the shoulder's
    // parent part).
    if (queue.plan) {
        const std::vector<math::Mat4>& plan = *queue.plan;
        auto planRot = [&](int j) { return math::toTransform(plan[static_cast<size_t>(j)]).rotation; };
        for (size_t i = 0; i < parts_.size(); ++i) {
            const Part& p = parts_[i];
            if (p.parent < 0 || !p.muscle || p.stiffness <= 0.0f || p.joint >= motion::BodyQueue::kMaxJoints ||
                !queue.planned[static_cast<size_t>(p.joint)])
                continue;
            targetLocal_[i] = math::inverse(planRot(parts_[static_cast<size_t>(p.parent)].joint)) * planRot(p.joint);
            targeted_[i] = 1;
            ordered_[i] = 1;
        }
    }

    for (int k = 0; k < queue.count; ++k) {
        const motion::PartOrder& o = queue.orders[static_cast<size_t>(k)];
        if (!o.active || o.kind != motion::PartMove::Turn || o.joint < 0) continue;
        const int part = partOf_[static_cast<size_t>(o.joint)];
        if (part < 0 || targeted_[static_cast<size_t>(part)]) continue;  // no body, or it follows the plan
        const Part& p = parts_[static_cast<size_t>(part)];
        if (p.joint != o.joint || !p.muscle) continue;  // rides its parent, or no muscle (the pelvis)
        float speed = 0.0f;                              // rad/s about the order's direction
        if (o.goal == motion::Goal::Chest || o.goal == motion::Goal::Head) {
            // A turn target: its angular gap closed in fittsA, plus its own turning.
            const bool chest = o.goal == motion::Goal::Chest;
            const motion::BodyGoals::Turn& t = chest ? goals.chest : goals.head;
            const int turned = chest ? joints.spine[2] : joints.head;
            const math::Vec3 gap = math::rotationVector(t.rotation * math::inverse(rot(turned)));
            speed = math::dot(t.angularVelocity + gap / std::max(timing.fittsA, 1e-3f), o.direction);
        } else {
            // A point target (hand, foot) the trunk joins (a far reach): the
            // end's needed velocity (its gap in Fitts' time plus the target's
            // own motion) across this joint's lever.
            const int h = o.goal == motion::Goal::HandL ? 0 : o.goal == motion::Goal::HandR ? 1 : -1;
            const int f = o.goal == motion::Goal::FootL ? 0 : o.goal == motion::Goal::FootR ? 1 : -1;
            const motion::BodyGoals::Point* g = h >= 0 ? &goals.hand[h] : f >= 0 ? &goals.foot[f] : nullptr;
            const int end = h >= 0 ? joints.arms[h].end : f >= 0 ? joints.legs[f].end : -1;
            if (!g || end < 0) continue;
            const math::Vec3 gapVec = g->position - pos(end);
            const float time = std::max(
                motion::movementTime(math::length(gapVec), timing.fittsWidth, timing.fittsA, timing.fittsB), 1e-3f);
            const math::Vec3 lever = math::cross(o.direction, pos(end) - pos(o.joint));
            const float reach = math::dot(lever, lever);
            if (reach > 1e-6f) speed = math::dot(g->velocity + gapVec / time, lever) / reach;
        }
        omega_[static_cast<size_t>(part)] += o.direction * (o.share * speed);
        ordered_[static_cast<size_t>(part)] = 1;
    }

    // Each muscle: its velocity in its joint's frame (on its own bone), its
    // pull no harder than the strength left after holding the weight. With no
    // order it holds the angle it had when its last order ended.
    for (size_t i = 0; i < parts_.size(); ++i) {
        const Part& p = parts_[i];
        if (p.parent < 0 || !p.muscle) continue;
        physics::MotorDesc m;
        if (free && free[i]) {
            world.setJointMotor(p.link, m, false);
            holding_[i] = 0;
            paths_[i].on = false;
            continue;
        }
        m.maxTorque = std::max(p.strength - hold_[i], 0.0f);
        world.setJointMotor(p.link, m, true);
        if (targeted_[i]) {
            // Its target pose: a spring on every axis (Jolt's position motor),
            // no harder than the strength left after holding the weight.
            // Its commanded angle travels to the target on a minimum-jerk path
            // in Fitts' time for the arc the bone's tip moves; a target that
            // moves by less than the target width keeps its path (retargeted),
            // more starts a new one from where the command is (as GoalPath).
            JointPath& path = paths_[i];
            const float lever = std::max(lever_[i], 1e-3f);
            const float width = timing.fittsWidth / lever;  // rad: the target's width at the tip
            auto startPath = [&](math::Quat from) {
                path.from = from;
                path.to = targetLocal_[i];
                path.time = 0.0f;
                path.duration = motion::movementTime(math::angleBetween(from, path.to) * lever, timing.fittsWidth,
                                                     timing.fittsA, timing.fittsB);
                path.on = true;
            };
            if (!path.on) {
                const math::Quat parentNow = rot(parts_[static_cast<size_t>(p.parent)].joint);
                startPath(math::normalize(math::inverse(parentNow) * rot(p.joint)));
            } else if (math::angleBetween(targetLocal_[i], path.to) > width) {
                startPath(path.at);
            } else {
                path.to = targetLocal_[i];
            }
            path.time += dt;
            const float s = std::clamp(path.time / std::max(path.duration, 1e-3f), 0.0f, 1.0f);
            path.at = math::slerp(path.from, path.to, s * s * s * (10.0f + s * (6.0f * s - 15.0f)));  // minimum jerk
            // Its stiffness rises with the torque it holds (measured linear).
            holding_[i] = 0;
            const float torque = hold_[i];
            setJointTarget(world, p, path.at, true, m.maxTorque, p.stiffness + p.stiffnessPerTorque * torque);
            continue;
        }
        paths_[i].on = false;
        if (!ordered_[i]) {
            // The held angle is a turn target: its gap closed in fittsA (as
            // any turn target), at full strength by the velocity motor.
            if (!holding_[i]) {
                heldRotation_[i] = p.hinge ? math::identityQuat() : world.sixDofRotation(p.link);
                heldAngle_[i] = p.hinge ? world.hingeAngle(p.link) : 0.0f;
                holding_[i] = 1;
            }
            const float close = 1.0f / std::max(timing.fittsA, 1e-3f);
            if (p.hinge) {
                world.setHingeVelocity(p.link, (heldAngle_[i] - world.hingeAngle(p.link)) * close);
            } else {
                // Jolt's gap: body 2's turn to the target, in its joint frame.
                math::Quat diff = math::inverse(world.sixDofRotation(p.link)) * heldRotation_[i];
                if (diff.w < 0.0f) diff = -diff;
                world.setSixDofVelocity(p.link, math::rotationVector(diff) * close);
            }
            continue;
        }
        holding_[i] = 0;
        const math::Vec3 w = math::inverse(rot(p.joint)) * omega_[i];  // the bone's frame
        if (p.hinge)
            world.setHingeVelocity(p.link, math::dot(w, p.frameX));
        else
            world.setSixDofVelocity(p.link,
                                    math::Vec3(math::dot(w, p.frameX), math::dot(w, p.frameY), math::dot(w, p.frameZ)));
    }
}

void HumanBody::blockedJoints(const physics::PhysicsWorld& world, const motion::BodyQueue& queue,
                              const std::vector<math::Mat4>& models, const ik::JointLimits& limits, float dt,
                              std::vector<uint8_t>& out) const {
    std::fill(out.begin(), out.end(), uint8_t{0});
    constexpr float kNudge = 0.01f;  // rad: a joint that cannot turn half of this further is at its limit
    for (int k = 0; k < queue.count; ++k) {
        const motion::PartOrder& o = queue.orders[static_cast<size_t>(k)];
        if (o.kind != motion::PartMove::Turn || o.joint < 0 || static_cast<size_t>(o.joint) >= out.size()) continue;
        if (math::dot(o.direction, o.direction) <= 0.0f) continue;  // at its target: no way to be pulled
        uint8_t& blocked = out[static_cast<size_t>(o.joint)];
        const int part = partOf_[static_cast<size_t>(o.joint)];
        if (part < 0 || parts_[static_cast<size_t>(part)].joint != o.joint) {
            blocked = 1;  // rides its parent: no muscle of its own
            continue;
        }
        const Part& p = parts_[static_cast<size_t>(part)];
        if (p.parent < 0 || !p.muscle) {
            blocked = 1;
            continue;
        }
        // At its limit: nudged the way it is pulled, the joint's range stops it.
        const math::Quat parent =
            math::toTransform(models[static_cast<size_t>(parts_[static_cast<size_t>(p.parent)].joint)]).rotation;
        const math::Quat bone = math::toTransform(models[static_cast<size_t>(p.joint)]).rotation;
        const math::Quat local = math::normalize(math::inverse(parent) * bone);
        const math::Quat nudged = math::normalize(math::inverse(parent) * math::fromAxisAngle(o.direction, kNudge) * bone);
        const bool atLimit =
            math::angleBetween(local, ik::clampRotation(nudged, limits.limit(p.joint))) < 0.5f * kNudge;
        // At full strength: its motor pushed as hard as it may.
        const float may = std::max(p.strength - hold_[static_cast<size_t>(part)], 0.0f);
        const bool spent = may <= 0.0f || world.jointMotorTorque(p.link, dt) >= 0.99f * may;
        if (atLimit || spent) blocked = 1;
    }
}

} // namespace agents
