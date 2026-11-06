#include "motion/body_decider.h"

#include "motion/reach.h"

#include <cassert>

namespace motion {

namespace {

// Turns smaller than this (rad) are no direction at all.
constexpr float kMinAngle = 1e-4f;

void add(BodyQueue& out, int joint, PartMove kind, math::Vec3 direction, math::Vec3 velocity, Goal goal, bool active) {
    assert(out.count < BodyQueue::kCapacity);
    if (out.count >= BodyQueue::kCapacity) return;
    out.orders[static_cast<size_t>(out.count++)] = {joint, kind, direction, velocity, goal, active};
}

// The unit axis of a turn, zero for none.
math::Vec3 axisOf(math::Quat q) {
    const math::Vec3 v = math::rotationVector(q);
    const float angle = math::length(v);
    return angle > kMinAngle ? v / angle : math::Vec3(0.0f);
}

} // namespace

bool BodyDecider::init(const rig::Rig& rig, const BodyDeciderJoints& joints) {
    rig_ = &rig;
    joints_ = joints;
    if (joints.pelvis < 0) return false;
    const size_t n = static_cast<size_t>(rig.numJoints());
    models_.resize(n);
    solved_.resize(n);
    scratch_ = rig.restPose();
    if (!rig.localToModel(rig.restPose(), models_)) return false;
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints.legs[i];
        if (leg.start >= 0)
            legLength_[i] = math::distance(pos(leg.start), pos(leg.mid)) + math::distance(pos(leg.mid), pos(leg.end));
        const ik::TwoBoneChain& arm = joints.arms[i];
        if (arm.start >= 0)
            armLength_[i] = math::distance(pos(arm.start), pos(arm.mid)) + math::distance(pos(arm.mid), pos(arm.end));
    }
    if (joints.legs[0].start >= 0 && joints.legs[1].start >= 0) {
        const math::Vec3 mid = 0.5f * (pos(joints.legs[0].start) + pos(joints.legs[1].start));
        hipCentre_ = math::inverse(rot(joints.pelvis)) * (mid - pos(joints.pelvis));
    }
    return true;
}

math::Vec3 BodyDecider::pos(int joint) const { return math::Vec3(models_[static_cast<size_t>(joint)][3]); }

math::Quat BodyDecider::rot(int joint) const {
    return math::toTransform(models_[static_cast<size_t>(joint)]).rotation;
}

bool BodyDecider::isBlocked(const std::vector<uint8_t>& blocked, int joint) const {
    return joint >= 0 && static_cast<size_t>(joint) < blocked.size() && blocked[static_cast<size_t>(joint)] != 0;
}

void BodyDecider::decide(const rig::Pose& pose, const BodyGoals& goals, const std::vector<uint8_t>& blocked,
                         const BodyDeciderSettings& s, BodyQueue& out) {
    out.count = 0;
    if (!rig_ || pose.locals.size() != scratch_.locals.size()) return;
    rig_->localToModel(pose, models_);
    scratch_.locals = pose.locals;  // same size: no allocation

    for (int h = 0; h < 2; ++h)
        if (goals.hand[h].active) handOrders(h, goals.hand[h], blocked, s, out);

    // Chest: chest, spine_02, spine_01, then the pelvis.
    if (goals.chest.active && joints_.spine[2] >= 0) {
        const int chain[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
        turnChain(chain, 4, goals.chest.rotation * math::inverse(rot(joints_.spine[2])), goals.chest.angularVelocity,
                  Goal::Chest, blocked, out);
    }
    // Head: head, then neck.
    if (goals.head.active && joints_.head >= 0) {
        const int chain[2] = {joints_.head, joints_.neck};
        turnChain(chain, 2, goals.head.rotation * math::inverse(rot(joints_.head)), goals.head.angularVelocity,
                  Goal::Head, blocked, out);
    }

    for (int i = 0; i < 2; ++i) {
        if (goals.foot[i].active) footOrders(i, goals, blocked, s, out);
        // Push-off: the calf points the unloading foot (toes down is + about the foot's X).
        const int ankle = joints_.legs[i].end;
        if (goals.push[i] > 0.0f && ankle >= 0)
            add(out, ankle, PartMove::Turn, math::normalize(rot(ankle) * math::kRight), math::Vec3(0.0f),
                i == 0 ? Goal::FootL : Goal::FootR, true);
    }

    // Hips: the legs carry them toward their target.
    if (goals.hips.active) {
        const math::Vec3 dir = math::safeNormalize(goals.hips.position - pos(joints_.pelvis));
        if (math::length(dir) > 0.0f)
            add(out, joints_.pelvis, PartMove::Move, dir, goals.hips.velocity, Goal::Hips, true);
    }
}

void BodyDecider::handOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                             const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    if (arm.start < 0) return;
    const Goal which = h == 0 ? Goal::HandL : Goal::HandR;
    const math::Vec3 shoulder = pos(arm.start), target = goal.position;

    // Shoulder and elbow, elbow back and out (square to the shoulder-to-hand line).
    const math::Vec3 dir = math::safeNormalize(target - shoulder, -math::kUp);
    const math::Vec3 side(h == 0 ? s.elbowPole.x : -s.elbowPole.x, s.elbowPole.y, s.elbowPole.z);
    const math::Vec3 pole = math::safeNormalize(side - dir * math::dot(side, dir), side);
    limbOrders(arm, target, pole, goal.velocity, which, out);

    // Out of the arm's reach: the trunk joins from the start, each part once
    // every part before it is blocked; a part that cannot bring the shoulder
    // closer is passed over.
    const float reach = armLength_[h];
    if (math::distance(shoulder, target) <= reach) return;
    bool before = true;  // every part before this one is blocked
    const int trunk[4] = {joints_.clavicles[h], joints_.spine[2], joints_.spine[1], joints_.spine[0]};
    for (int j : trunk) {
        if (j < 0) continue;
        const Swing sw = swingToward(pos(j), shoulder, target, reach);
        if (sw.angle <= kMinAngle) continue;
        add(out, j, PartMove::Turn, sw.axis, goal.velocity, which, before);
        before = before && isBlocked(blocked, j);
    }
    // Then the pelvis tilts at the hips.
    const math::Vec3 hips = pos(joints_.pelvis) + rot(joints_.pelvis) * hipCentre_;
    const Swing sw = swingToward(hips, shoulder, target, reach);
    if (sw.angle > kMinAngle) {
        add(out, joints_.pelvis, PartMove::Turn, sw.axis, goal.velocity, which, before);
        before = before && isBlocked(blocked, joints_.pelvis);
    }
    // Then the hips move toward the hand (the planted feet stay: the legs bend).
    add(out, joints_.pelvis, PartMove::Move, math::normalize(target - shoulder), goal.velocity, which, before);
}

void BodyDecider::footOrders(int i, const BodyGoals& goals, const std::vector<uint8_t>& blocked,
                             const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& leg = joints_.legs[i];
    if (leg.start < 0) return;
    const Goal which = i == 0 ? Goal::FootL : Goal::FootR;
    const BodyGoals::Point& goal = goals.foot[i];
    const math::Vec3 hip = pos(leg.start), target = goal.position;

    // Hip and knee, knee forward.
    limbOrders(leg, target, rot(joints_.pelvis) * s.kneePole, goal.velocity, which, out);

    // Past the hip's range: the pelvis tilts so the leg points at the target.
    const math::Vec3 axis = math::safeNormalize(math::cross(pos(leg.end) - hip, target - hip));
    if (math::length(axis) > 0.0f)
        add(out, joints_.pelvis, PartMove::Turn, axis, goal.velocity, which, isBlocked(blocked, leg.start));

    // A foot on the ground beyond the leg's length draws the hips toward it.
    if (goals.down[i] && math::distance(hip, target) > legLength_[i])
        add(out, joints_.pelvis, PartMove::Move, math::normalize(target - hip), goal.velocity, which, true);
}

void BodyDecider::limbOrders(const ik::TwoBoneChain& chain, math::Vec3 target, math::Vec3 pole, math::Vec3 velocity,
                             Goal goal, BodyQueue& out) {
    // The turns a two-bone solve would make give the directions (only those).
    const size_t a = static_cast<size_t>(chain.start), b = static_cast<size_t>(chain.mid);
    const math::Transform keepA = scratch_.locals[a], keepB = scratch_.locals[b];
    if (ik::solveTwoBone(chain, models_, target, pole, 1.0f, scratch_) && rig_->localToModel(scratch_, solved_)) {
        for (int j : {chain.start, chain.mid}) {
            const math::Quat now = rot(j);
            const math::Quat then = math::toTransform(solved_[static_cast<size_t>(j)]).rotation;
            const math::Vec3 axis = axisOf(then * math::inverse(now));
            if (math::length(axis) > 0.0f) add(out, j, PartMove::Turn, axis, velocity, goal, true);
        }
    }
    scratch_.locals[a] = keepA;
    scratch_.locals[b] = keepB;
}

void BodyDecider::turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                            const std::vector<uint8_t>& blocked, BodyQueue& out) {
    const math::Vec3 axis = axisOf(delta);
    if (math::length(axis) <= 0.0f) return;
    bool before = true;  // every part before this one is blocked
    for (int k = 0; k < count; ++k) {
        const int j = joints[k];
        if (j < 0) continue;
        add(out, j, PartMove::Turn, axis, velocity, goal, before);
        before = before && isBlocked(blocked, j);
    }
}

} // namespace motion
