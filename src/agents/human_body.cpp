#include "agents/human_body.h"

#include "core/log.h"

#include <algorithm>
#include <cmath>

namespace agents {

namespace {

math::Vec3 axisVec(int axis) { return axis == 0 ? math::kRight : axis == 1 ? math::kUp : math::kForward; }

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

// A rotation given in a joint's frame (columns x, y, z) in the bone's frame.
math::Quat fromFrame(math::Quat q, math::Vec3 x, math::Vec3 y, math::Vec3 z) {
    const math::Vec3 v = q.x * x + q.y * y + q.z * z;
    return math::Quat(q.w, v.x, v.y, v.z);
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
            // the character's axes): Jolt SixDOF in the plan's own frame and
            // ranges (ik::limitFrame, the joint's rest frame on both bodies),
            // so Jolt's clamp is the plan's (ik::clampRotation).
            r.pivot = root.transformPoint(math::Vec3(models[static_cast<size_t>(j)][3]));
            const ik::JointLimit& lim = limits.limit(j);
            const ik::LimitFrame f = ik::limitFrame(lim);
            r.axisX = root.transformDirection(f.x);
            r.axisY = root.transformDirection(f.y);
            r.minAngles = f.min;
            r.maxAngles = f.max;
            p.hinge = lim.type == ik::LimitType::Hinge;
            p.frameX = f.x;
            p.frameY = f.y;
            p.frameZ = f.z;
            const math::Vec3 x = f.x, y = f.y, z = f.z;
            const float strength = valueOf(muscles.maxTorque, bone.name);
            r.motor.frequency = frequency;
            r.motor.damping = muscles.damping;
            r.motor.maxTorque = strength;  // 0: no muscle (the joint hangs free)
            p.muscle = strength > 0.0f;
            // Two muscles about one axis: which axis of the joint's frame it
            // is (X, Y, Z) and which way is +.
            const std::string base = baseName(bone.name);
            for (const MuscleSettings::Directional& d : muscles.directional) {
                if (d.bone != base || !p.muscle) continue;
                const math::Vec3 a = axisVec(d.axis);
                const float dots[3] = {math::dot(a, x), math::dot(a, y), math::dot(a, z)};
                int k = 0;
                for (int i = 1; i < 3; ++i)
                    if (std::abs(dots[i]) > std::abs(dots[k])) k = i;
                // The right side is the left mirrored: rotations about Y and Z turn the other way.
                const bool right = base != bone.name && bone.name.back() == 'r';
                const bool flipped = (dots[k] < 0.0f) != (right && d.axis != 0);
                r.directionalAxis = k;
                r.plusTorque = flipped ? d.minus : d.plus;
                r.minusTorque = flipped ? d.plus : d.minus;
            }
        }
        owner[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        partOf_[static_cast<size_t>(j)] = static_cast<int>(parts_.size());
        parts_.push_back(p);
        ragdoll.push_back(r);
        totalMass_ += p.mass;
    }
    // A range set by another joint's angle (ik::LimitCoupling): the part
    // whose joint is that one, read from the physics body each tick.
    for (Part& p : parts_) {
        const int cj = p.parent >= 0 ? limits.limit(p.joint).coupling.joint : -1;
        if (cj < 0) continue;
        const int cp = partOf_[static_cast<size_t>(cj)];
        if (cp < 0 || rig.parent(cj) < 0 || partOf_[static_cast<size_t>(rig.parent(cj))] != parts_[static_cast<size_t>(cp)].parent) {
            LOG_WARN("human body: {}'s range follows {}, which has no joint of its own", rig.jointName(p.joint),
                     rig.jointName(cj));
            continue;
        }
        p.coupledPart = cp;
    }
    // Each muscle's stiffness from what it moves: the inertia of every part
    // beyond its joint, about the joint (each part as a point mass at its
    // centre, as built), at the activation frequency: K = I (2 pi f)^2,
    // critically damped (2 sqrt(K I)); its torque stays capped at maxTorque.
    const float w2 = (math::kTwoPi * frequency) * (math::kTwoPi * frequency);
    std::vector<float> inertia(ragdoll.size(), 0.0f);
    for (size_t i = 0; i < ragdoll.size(); ++i)
        for (int j = static_cast<int>(i); j >= 0 && ragdoll[static_cast<size_t>(j)].parent >= 0;
             j = ragdoll[static_cast<size_t>(j)].parent)
            inertia[static_cast<size_t>(j)] +=
                ragdoll[i].body.mass * math::length2(ragdoll[i].body.pose.position - ragdoll[static_cast<size_t>(j)].pivot);
    for (size_t i = 0; i < ragdoll.size(); ++i) {
        if (ragdoll[i].parent < 0 || inertia[i] <= 0.0f) continue;
        ragdoll[i].motor.stiffness = inertia[i] * w2;
        ragdoll[i].motor.damping = 2.0f * std::sqrt(ragdoll[i].motor.stiffness * inertia[i]) * muscles.damping;
    }
    std::vector<physics::BodyId> bodies;
    limits_ = &limits;
    ragdoll_ = world.createRagdoll(ragdoll, group, bodies);
    if (ragdoll_ == physics::kInvalidRagdoll || bodies.size() != parts_.size()) {
        parts_.clear();
        return false;
    }
    for (size_t i = 0; i < parts_.size(); ++i) parts_[i].body = bodies[i];
    rotations_.assign(parts_.size(), math::identityQuat());
    lastTarget_.assign(parts_.size(), math::identityQuat());
    leadReady_ = false;
    lead_.assign(parts_.size(), 0.0f);
    for (size_t i = 0; i < parts_.size() && i < ragdoll.size(); ++i)
        if (ragdoll[i].parent >= 0 && ragdoll[i].motor.stiffness > 0.0f)
            lead_[i] = ragdoll[i].motor.damping / ragdoll[i].motor.stiffness;
    beyondMass_.assign(parts_.size(), 0.0f);
    beyondMoment_.assign(parts_.size(), math::Vec3(0.0f));
    hold_.assign(parts_.size(), math::Vec3(0.0f));
    ground_.assign(parts_.size(), math::Vec3(0.0f));
    LOG_INFO("human body: Jolt ragdoll, {} parts, {:.1f} kg", parts_.size(), totalMass_);
    return true;
}

math::Transform HumanBody::partPose(int part, const std::vector<math::Mat4>& models) const {
    const Part& p = parts_[static_cast<size_t>(part)];
    return math::toTransform(models[static_cast<size_t>(p.joint)]) * p.shape;
}

void HumanBody::holdWeight(physics::PhysicsWorld& world, const StanceFeet& feet) const {
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
        hold_[i] = ground_[i] = math::Vec3(0.0f);
        if (p.parent < 0 || !p.muscle || beyondMass_[i] <= 0.0f) continue;
        // Gravity's turn on everything beyond the joint, about the joint.
        const math::Vec3 pivot = boneWorld(world, static_cast<int>(i)).position;
        const math::Vec3 centre = beyondMoment_[i] / beyondMass_[i];
        hold_[i] = -math::cross(centre - pivot, g * beyondMass_[i]);
    }
    // A foot on the ground: the weight comes down its leg the other way (the
    // foot the root; Winter 1995). The ground pushes up its share of the
    // whole body's weight at its centre of pressure. Standing feet share by
    // the lever rule along the line between them, within the decider's support
    // weights (a foot lifting or landing eases out or in). The feet's pushes
    // together act below the centre of mass: every foot's centre of pressure
    // is its centre moved by the same shift (the centre of mass off the
    // shares' centre), inside its outline. One rule for one foot or two: on
    // two side by side it moves them along with the body, on one it is the
    // centre of mass below that foot. Each joint from the ankle up to the hip
    // carries that push's turn about it as well (the static balance of the
    // part and all below it). What the pushes leave unbalanced the support
    // spring takes, as before.
    // All of it from the plan (its centre of mass, feet and joints), never
    // the real body: a push that followed the real body's shake fed the shake
    // (the feet trembled) and followed its lean (a foot rolled onto its heel).
    if (parts_.empty() || beyondMass_[0] <= 0.0f || !feet.plan) {
        world.setRagdollHold(ragdoll_, hold_.data(), ground_.data(), static_cast<int>(hold_.size()));
        return;
    }
    const std::vector<math::Mat4>& plan = *feet.plan;
    const auto planJoint = [&](int part) {
        return feet.root.transformPoint(math::Vec3(plan[static_cast<size_t>(parts_[static_cast<size_t>(part)].joint)][3]));
    };
    const float total = beyondMass_[0];
    math::Vec3 com(0.0f);
    for (size_t i = 0; i < parts_.size(); ++i)
        com += (feet.root * partPose(static_cast<int>(i), plan)).position * parts_[i].mass;
    com /= total;
    const math::Vec3 fwd = feet.facing * math::kForward, side = feet.facing * math::kRight;
    math::Vec3 centre[2], cop[2];
    float w[2], share[2] = {0.0f, 0.0f};
    for (int f = 0; f < 2; ++f) {
        w[f] = feet.part[f] >= 0 ? std::clamp(feet.weight[f], 0.0f, 1.0f) : 0.0f;
        centre[f] = feet.part[f] >= 0 ? planJoint(feet.part[f]) + fwd * feet.ahead : com;
    }
    float lever = 0.5f;
    if (w[0] > 0.0f && w[1] > 0.0f) {
        const math::Vec3 line = centre[1] - centre[0];
        const float len2 = line.x * line.x + line.z * line.z;
        const math::Vec3 d = com - centre[0];
        if (len2 > 1e-6f) lever = std::clamp((d.x * line.x + d.z * line.z) / len2, 0.0f, 1.0f);
    }
    // No foot carries more than its support weight lets it (a foot landing
    // takes the weight as the decider gives it, never all at once); both
    // under full: by those weights alone.
    share[1] = w[0] + w[1] >= 1.0f ? std::clamp(lever, 1.0f - w[0], w[1]) : w[1] / std::max(w[0] + w[1], 1e-6f);
    share[0] = w[0] + w[1] > 1e-6f ? 1.0f - share[1] : 0.0f;
    if (const float all = share[0] + share[1]; all > 1e-6f) {
        const math::Vec3 shift = com - (share[0] * centre[0] + share[1] * centre[1]) / all;
        for (int f = 0; f < 2; ++f) {
            const float x = std::clamp(math::dot(shift, side), -feet.halfWidth, feet.halfWidth);
            const float z = std::clamp(math::dot(shift, fwd), -feet.halfLength, feet.halfLength);
            cop[f] = centre[f] + side * x + fwd * z;
        }
    }
    const float sum = share[0] + share[1];
    for (int f = 0; f < 2 && sum > 1e-6f; ++f) {
        const float m = total * share[f] / sum * std::max(feet.load, 0.0f);
        if (m <= 0.0f) continue;
        for (int i = feet.part[f]; i >= 0 && parts_[static_cast<size_t>(i)].parent >= 0; i = parts_[static_cast<size_t>(i)].parent) {
            if (!parts_[static_cast<size_t>(i)].muscle) continue;
            // The foot itself stands on the ground, which gives it its share:
            // the ankle's muscles pull only the shin by it (a foot pressed
            // between the push and the ground shook within the contact).
            const math::Vec3 push = math::cross(cop[f] - planJoint(i), g * m);
            (i == feet.part[f] ? ground_ : hold_)[static_cast<size_t>(i)] += push;
        }
    }
    world.setRagdollHold(ragdoll_, hold_.data(), ground_.data(), static_cast<int>(hold_.size()));
}

ik::JointLimit HumanBody::limitNow(const physics::PhysicsWorld& world, int part) const {
    const Part& p = parts_[static_cast<size_t>(part)];
    const ik::JointLimit& lim = limits_->limit(p.joint);
    if (p.coupledPart < 0) return lim;
    // The coupled joint's angle in the real body (its rotation in its joint's
    // frame, back in its bone's frame), mapped as JointLimits::limitIn.
    const Part& c = parts_[static_cast<size_t>(p.coupledPart)];
    const math::Quat local = fromFrame(world.ragdollJointRotation(ragdoll_, p.coupledPart), c.frameX, c.frameY, c.frameZ);
    return ik::coupledLimit(lim, ik::hingeAngle(local, limits_->limit(c.joint)));
}

void HumanBody::drivePose(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root,
                          int headJoint, float dt) const {
    // The pose's own velocity, not eased: an eased one still leads after the
    // pose has stopped (measured: the swing foot landed 8 cm past its spot,
    // the knee 6 deg past the plan's).
    const bool lead = leadReady_ && dt > 0.0f;
    for (size_t i = 0; i < parts_.size(); ++i) {
        const Part& p = parts_[i];
        if (p.parent < 0) continue;
        if (p.coupledPart >= 0) {
            const ik::LimitFrame f = ik::limitFrame(limitNow(world, static_cast<int>(i)));
            world.setRagdollJointLimits(ragdoll_, static_cast<int>(i), f.min, f.max);
        }
        const math::Quat child = partPose(static_cast<int>(i), models).rotation;
        if (p.joint == headJoint) {
            // Held in space from the real body below, within the head's range
            // (the same clamp the plan's head has).
            const Part& up = parts_[static_cast<size_t>(p.parent)];
            const math::Quat realParent = world.transform(up.body).rotation;
            const math::Quat rel = math::normalize(math::inverse(realParent) * (root.rotation * child));
            const math::Quat local = ik::clampRotation(up.shape.rotation * rel * math::inverse(p.shape.rotation),
                                                       limitNow(world, static_cast<int>(i)));
            rotations_[i] = math::normalize(math::inverse(up.shape.rotation) * local * p.shape.rotation);
        } else {
            rotations_[i] = math::inverse(partPose(p.parent, models).rotation) * child;
        }
        // The lead along the pose's own motion (not the head's reflex).
        math::Quat now = math::normalize(rotations_[i]);
        if (lead && p.joint != headJoint) {
            math::Quat last = lastTarget_[i];
            if (math::dot(now, last) < 0.0f) last = -last;
            const math::Vec3 w = math::rotationVector(math::normalize(now * math::inverse(last))) / dt;
            const math::Vec3 ahead = w * lead_[i];
            const float angle = math::length(ahead);
            if (angle > 1e-6f) rotations_[i] = math::normalize(math::fromAxisAngle(ahead / angle, angle) * now);
        }
        lastTarget_[i] = now;
    }
    leadReady_ = true;
    world.driveRagdollToPose(ragdoll_, rotations_.data(), static_cast<int>(rotations_.size()));
}

void HumanBody::reset(physics::PhysicsWorld& world, const std::vector<math::Mat4>& models, const math::Transform& root) const {
    leadReady_ = false;  // the pose jumps: no motion to lead from
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
