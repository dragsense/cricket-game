#include "motion/body_decider.h"

#include "ik/joint_limits.h"
#include "motion/reach.h"

#include <cassert>
#include <cmath>

namespace motion {

namespace {

// Turns smaller than this (rad) are no direction at all.
constexpr float kMinAngle = 1e-4f;

void add(BodyQueue& out, int joint, PartMove kind, math::Vec3 direction, math::Vec3 velocity, Goal goal, bool active,
         float share = 1.0f) {
    assert(out.count < BodyQueue::kCapacity);
    if (out.count >= BodyQueue::kCapacity) return;
    out.orders[static_cast<size_t>(out.count++)] = {joint, kind, direction, velocity, goal, active, share};
}

// The unit axis of a turn, zero for none.
math::Vec3 axisOf(math::Quat q) {
    const math::Vec3 v = math::rotationVector(q);
    const float angle = math::length(v);
    return angle > kMinAngle ? v / angle : math::Vec3(0.0f);
}

} // namespace

bool BodyDecider::init(const rig::Rig& rig, const BodyDeciderJoints& joints, const ik::JointLimits* limits) {
    rig_ = &rig;
    limits_ = limits;
    joints_ = joints;
    if (joints.pelvis < 0 || rig.numJoints() > BodyQueue::kMaxJoints) return false;
    const size_t n = static_cast<size_t>(rig.numJoints());
    models_.resize(n);
    solved_.resize(n);
    scratch_ = rig.restPose();
    if (!rig.localToModel(rig.restPose(), models_)) return false;
    uprightPelvis_ = rot(joints.pelvis);
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints.legs[i];
        if (leg.start >= 0)
            legLength_[i] = math::distance(pos(leg.start), pos(leg.mid)) + math::distance(pos(leg.mid), pos(leg.end));
        const ik::TwoBoneChain& arm = joints.arms[i];
        if (arm.start >= 0) {
            armLength_[i] = math::distance(pos(arm.start), pos(arm.mid)) + math::distance(pos(arm.mid), pos(arm.end));
            const math::Quat toChest = joints.spine[2] >= 0 ? math::inverse(rot(joints.spine[2])) : math::identityQuat();
            const math::Vec3 hang = math::safeNormalize(pos(arm.end) - pos(arm.start), -math::kUp);
            const math::Vec3 back = -math::kForward - hang * math::dot(-math::kForward, hang);
            hangDir_[i] = toChest * hang;
            hangBend_[i] = toChest * math::safeNormalize(back, -math::kForward);
        }
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

math::Vec3 BodyDecider::planPos(int joint) const { return math::Vec3(solved_[static_cast<size_t>(joint)][3]); }

math::Quat BodyDecider::planRot(int joint) const {
    return math::toTransform(solved_[static_cast<size_t>(joint)]).rotation;
}

void BodyDecider::markPlanned(int joint, BodyQueue& out) const {
    if (joint >= 0 && joint < BodyQueue::kMaxJoints) out.planned[static_cast<size_t>(joint)] = 1;
}

void BodyDecider::turnPlanned(int j, math::Quat turn) {
    const int parent = rig_->parent(j);
    const math::Quat up = parent >= 0 ? planRot(parent) : math::identityQuat();
    math::Quat local = math::normalize(math::inverse(up) * turn * planRot(j));
    if (limits_) local = ik::clampRotation(local, limits_->limit(j));
    scratch_.locals[static_cast<size_t>(j)].rotation = local;
    rig_->localToModel(scratch_, solved_);
}

void BodyDecider::placePlannedPelvis(const math::Transform& t) {
    const int parent = rig_->parent(joints_.pelvis);
    math::Transform& local = scratch_.locals[static_cast<size_t>(joints_.pelvis)];
    const math::Transform placed =
        parent >= 0 ? math::inverse(math::toTransform(solved_[static_cast<size_t>(parent)])) * t : t;
    local.position = placed.position;
    local.rotation = math::normalize(placed.rotation);
    rig_->localToModel(scratch_, solved_);
    pelvisPlanned_ = true;
}

bool BodyDecider::isBlocked(const std::vector<uint8_t>& blocked, int joint) const {
    return joint >= 0 && static_cast<size_t>(joint) < blocked.size() && blocked[static_cast<size_t>(joint)] != 0;
}

void BodyDecider::decide(const rig::Pose& pose, const BodyGoals& goals, const std::vector<uint8_t>& blocked,
                         const BodyDeciderSettings& s, BodyQueue& out) {
    out.count = 0;
    out.plan = nullptr;
    out.planned.fill(0);
    if (!rig_ || pose.locals.size() != scratch_.locals.size()) return;
    rig_->localToModel(pose, models_);
    // The plan starts as the body now; the parts a goal needs are replaced.
    scratch_.locals = pose.locals;  // same size: no allocation
    rig_->localToModel(scratch_, solved_);
    pelvisPlanned_ = false;
    out.plan = &solved_;

    // Hips: the legs carry them to their target (the planned pelvis there).
    if (goals.hips.active) {
        const math::Vec3 dir = math::safeNormalize(goals.hips.position - pos(joints_.pelvis));
        if (math::length(dir) > 0.0f)
            add(out, joints_.pelvis, PartMove::Move, dir, goals.hips.velocity, Goal::Hips, true);
        math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
        t.position = goals.hips.position;
        placePlannedPelvis(t);
    }

    // The trunk for the hands, then the legs under it (the plan's centre of
    // mass kept over the planted feet), then the arms from the planned
    // shoulders.
    for (int h = 0; h < 2; ++h)
        if (goals.hand[h].active) handTrunk(h, goals.hand[h], blocked, s, out);
    planLegs(goals, s, out);
    for (int h = 0; h < 2; ++h)
        if (goals.hand[h].active) handOrders(h, goals.hand[h], blocked, s, out);

    // Chest: chest, spine_02, spine_01, then the pelvis.
    if (goals.chest.active && joints_.spine[2] >= 0) {
        const int chain[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
        turnChain(chain, 4, goals.chest.rotation * math::inverse(rot(joints_.spine[2])), goals.chest.angularVelocity,
                  Goal::Chest, blocked, s, out);
    }
    // Head: head, then neck.
    if (goals.head.active && joints_.head >= 0) {
        const int chain[2] = {joints_.head, joints_.neck};
        turnChain(chain, 2, goals.head.rotation * math::inverse(rot(joints_.head)), goals.head.angularVelocity,
                  Goal::Head, blocked, s, out);
    }

    for (int i = 0; i < 2; ++i) {
        if (goals.foot[i].active) footOrders(i, goals, blocked, s, out);
        // Push-off: the calf points the unloading foot (toes down is + about the foot's X).
        const int ankle = joints_.legs[i].end;
        if (goals.push[i] > 0.0f && ankle >= 0)
            add(out, ankle, PartMove::Turn, math::normalize(rot(ankle) * math::kRight), math::Vec3(0.0f),
                i == 0 ? Goal::FootL : Goal::FootR, true);
    }

}

void BodyDecider::handTrunk(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                            const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    if (arm.start < 0) return;
    const math::Vec3 target = goal.position;
    const float reach = armLength_[h];

    // While a hand has a goal the trunk is planned from upright (so it
    // straightens again after a far reach): the spine at rest, the pelvis
    // upright about the hips where they are.
    const rig::Pose& rest = rig_->restPose();
    for (int j : joints_.spine)
        if (j >= 0) {
            scratch_.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            markPlanned(j, out);
        }
    {
        const math::Transform now = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
        const math::Vec3 hipsNow = now.position + now.rotation * hipCentre_;
        placePlannedPelvis({hipsNow - uprightPelvis_ * hipCentre_, uprightPelvis_, now.scale});
    }
    const math::Vec3 shoulder = planPos(arm.start);

    // Out of the arm's reach every part of the queue moves from the start,
    // shared by its place (shareQueue). The queue is led by where the target
    // is (BIOMECHANICS 9.1, 9.3):
    //  - below the hips: a deep bend is mostly at the hips, and the legs lower
    //    the body (Esola et al. 1996; fielders bend the legs, section 8): pelvis
    //    tilt, hips lower, spine_01, spine_02, chest, clavicle;
    //  - behind the shoulders: a turn is mostly thoracic: chest, spine_02,
    //    spine_01, clavicle, pelvis, hips;
    //  - ahead or to the side: the shoulder girdle, then the trunk leans, then
    //    the whole body (Levin et al. 2001): clavicle, chest, spine_02,
    //    spine_01, pelvis tilt, hips.
    // A part that cannot bring the shoulder closer is left out. Each part's
    // share of the swing that brings the shoulder within reach is how far it
    // turns in the plan (planTrunk).
    if (math::distance(shoulder, target) > reach) trunkOrders(h, goal, blocked, s, out);
}

void BodyDecider::handOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& /*blocked*/,
                             const BodyDeciderSettings& /*s*/, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    if (arm.start < 0) return;
    const Goal which = h == 0 ? Goal::HandL : Goal::HandR;
    const math::Vec3 target = goal.position;

    // Shoulder and elbow, from the planned shoulder. The elbow's side (the
    // solve's pole): the arm turns from hanging relaxed off the chest to the
    // target by the shortest swing, adding no twist (the shoulder's neutral
    // rotation), and the elbow's bend turns with it: beside the thigh it
    // points back (palm to the thigh), at the face below and ahead, behind the
    // back back and up, overhead ahead. No fixed direction, so none the arm
    // can line up with (a fixed pole flips the elbow where the arm points
    // along it). Exactly overhead the swing goes through the front (shoulder
    // flexion).
    const math::Vec3 from = planPos(arm.start);
    const math::Quat chest = joints_.spine[2] >= 0 ? planRot(joints_.spine[2]) : math::identityQuat();
    const math::Vec3 hang = chest * hangDir_[h], bend = chest * hangBend_[h];
    const math::Vec3 dir = math::safeNormalize(target - from, hang);
    const math::Quat swing = math::dot(hang, dir) < -0.9999f
                                 ? math::fromAxisAngle(math::safeNormalize(math::cross(hang, -bend), math::kRight), math::kPi)
                                 : math::fromTo(hang, dir);
    limbOrders(arm, target, swing * bend, goal.velocity, which, out);
}

void BodyDecider::trunkOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                              const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    const Goal which = h == 0 ? Goal::HandL : Goal::HandR;
    const math::Vec3 target = goal.position, shoulder = planPos(arm.start);
    const float reach = armLength_[h];
    const math::Vec3 hips = planPos(joints_.pelvis) + planRot(joints_.pelvis) * hipCentre_;
    const math::Vec3 forward = planRot(joints_.pelvis) * math::kForward;
    enum Stage { Clavicle, Chest, Spine2, Spine1, Tilt, Move };
    static constexpr Stage kLow[6] = {Tilt, Move, Spine1, Spine2, Chest, Clavicle};
    static constexpr Stage kBehind[6] = {Chest, Spine2, Spine1, Clavicle, Tilt, Move};
    static constexpr Stage kAhead[6] = {Clavicle, Chest, Spine2, Spine1, Tilt, Move};
    const Stage* order = target.y < hips.y ? kLow : math::dot(target - shoulder, forward) < 0.0f ? kBehind : kAhead;

    const int first = out.count;
    int lumbar = -1, tilt = -1;  // their orders, for the measured split
    for (int k = 0; k < 6; ++k) {
        if (order[k] == Move) {
            // The legs move the hips toward the hand (the planted feet stay:
            // the legs bend); they cannot once both knees are blocked.
            const bool legsBlocked = isBlocked(blocked, joints_.legs[0].mid) && isBlocked(blocked, joints_.legs[1].mid);
            add(out, joints_.pelvis, PartMove::Move, math::normalize(target - shoulder), goal.velocity, which, false,
                legsBlocked ? 0.0f : 1.0f);
            continue;
        }
        const int j = order[k] == Clavicle ? joints_.clavicles[h]
                    : order[k] == Chest    ? joints_.spine[2]
                    : order[k] == Spine2   ? joints_.spine[1]
                    : order[k] == Spine1   ? joints_.spine[0]
                                           : joints_.pelvis;  // Tilt: the pelvis tilts at the hips
        if (j < 0) continue;
        const Swing sw = swingToward(order[k] == Tilt ? hips : planPos(j), shoulder, target, reach);
        if (sw.angle <= kMinAngle) continue;
        if (order[k] == Spine1) lumbar = out.count;
        if (order[k] == Tilt) tilt = out.count;
        add(out, j, PartMove::Turn, sw.axis, goal.velocity, which, false, isBlocked(blocked, j) ? 0.0f : 1.0f);
    }
    shareQueue(first, s, out);

    // A bend (not a turn): the lumbar spine and the hips split their combined
    // share by the measured lumbopelvic rhythm at the trunk's flexion now.
    if (order != kBehind && lumbar >= 0 && tilt >= 0 && joints_.spine[2] >= 0) {
        PartOrder& l = out.orders[static_cast<size_t>(lumbar)];
        PartOrder& p = out.orders[static_cast<size_t>(tilt)];
        if (l.share > 0.0f && p.share > 0.0f) {
            const math::Vec3 trunk = math::safeNormalize(pos(joints_.spine[2]) - pos(joints_.pelvis), math::kUp);
            const float flexion = std::acos(math::clamp(math::dot(trunk, math::kUp), -1.0f, 1.0f));
            int band = 0;
            while (band < 2 && flexion > s.lumbarHipBand[band]) ++band;
            const float ratio = s.lumbarHipRatio[band], both = l.share + p.share;
            l.share = both * ratio / (1.0f + ratio);
            p.share = both / (1.0f + ratio);
        }
    }
    planTrunk(h, target, first, out);
}

void BodyDecider::planTrunk(int h, math::Vec3 target, int first, BodyQueue& out) {
    // In passes: each part of the queue turns by its share of the swing that
    // would bring the shoulder within reach on its own (the hips lower by
    // their share of the gap), within its range; the next pass takes what is
    // left (a part at its range leaves its part to the others).
    const ik::TwoBoneChain& arm = joints_.arms[h];
    const float reach = armLength_[h];
    constexpr int kPasses = 4;
    constexpr float kClose = 1e-3f;  // m: within reach
    for (int pass = 0; pass < kPasses; ++pass) {
        for (int k = first; k < out.count; ++k) {
            const PartOrder& o = out.orders[static_cast<size_t>(k)];
            if (o.share <= 0.0f) continue;
            const math::Vec3 shoulder = planPos(arm.start);
            const float gap = math::distance(shoulder, target) - reach;
            if (gap <= kClose) return;
            if (o.joint == joints_.pelvis && o.kind == PartMove::Move) {
                // The legs lower the hips (they bend; the planted feet stay).
                // Shifting them over the feet is balance's, not the reach's.
                math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
                const float down = std::min(0.0f, math::safeNormalize(target - shoulder).y) * gap * o.share;
                t.position.y += down;
                placePlannedPelvis(t);
            } else if (o.joint == joints_.pelvis) {
                // The pelvis tilts at the hips (the hips stay).
                const math::Vec3 hips = planPos(joints_.pelvis) + planRot(joints_.pelvis) * hipCentre_;
                const Swing sw = swingToward(hips, shoulder, target, reach);
                const math::Quat turn = math::fromAxisAngle(sw.axis, sw.angle * o.share);
                math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
                t.rotation = turn * t.rotation;
                t.position = hips + turn * (t.position - hips);
                placePlannedPelvis(t);
            } else if (o.joint == joints_.spine[0] || o.joint == joints_.spine[1] || o.joint == joints_.spine[2]) {
                // A spine joint (the clavicle has no body yet: it rides the
                // chest as built, so it is not planned).
                const Swing sw = swingToward(planPos(o.joint), shoulder, target, reach);
                turnPlanned(o.joint, math::fromAxisAngle(sw.axis, sw.angle * o.share));
            }
        }
    }
}

void BodyDecider::setMasses(const std::vector<float>& mass, const std::vector<math::Vec3>& centre) {
    mass_ = mass;
    centre_ = centre;
    totalMass_ = 0.0f;
    for (float m : mass_) totalMass_ += m;
}

math::Vec3 BodyDecider::planCentreOfMass() const {
    math::Vec3 sum(0.0f);
    for (size_t j = 0; j < mass_.size() && j < solved_.size() && j < centre_.size(); ++j)
        if (mass_[j] > 0.0f) sum += mass_[j] * math::toTransform(solved_[j]).transformPoint(centre_[j]);
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

void BodyDecider::planLegs(const BodyGoals& goals, const BodyDeciderSettings& s, BodyQueue& out) {
    if (!pelvisPlanned_) return;
    // The planted feet stay where they are; the legs reach them from the
    // planned hips, and the hips go where the plan's centre of mass stays over
    // the feet (a bend takes the hips back: anticipatory posture, Massion
    // 1992). The hips cannot go higher than the straight legs allow.
    math::Vec3 support(0.0f);
    int planted = 0;
    for (int i = 0; i < 2; ++i)
        if (joints_.legs[i].end >= 0 && !goals.foot[i].active) {
            support += pos(joints_.legs[i].end);
            ++planted;
        }
    if (planted == 0) return;
    support /= static_cast<float>(planted);

    const rig::Pose& rest = rig_->restPose();
    auto solveLegs = [&] {
        for (int pass = 0; pass < 2; ++pass)
            for (int i = 0; i < 2; ++i) {
                const ik::TwoBoneChain& leg = joints_.legs[i];
                if (leg.start < 0 || goals.foot[i].active) continue;
                const math::Vec3 hip = planPos(leg.start), ankle = pos(leg.end);
                const float over = math::distance(hip, ankle) - legLength_[i];
                if (over <= 0.0f) continue;
                math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
                t.position += math::normalize(ankle - hip) * over;
                placePlannedPelvis(t);
            }
        for (int i = 0; i < 2; ++i) {
            const ik::TwoBoneChain& leg = joints_.legs[i];
            if (leg.start < 0 || goals.foot[i].active) continue;
            for (int j : {leg.start, leg.mid})
                scratch_.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            rig_->localToModel(scratch_, solved_);
            if (ik::solveTwoBone(leg, solved_, pos(leg.end), planRot(joints_.pelvis) * s.kneePole, 1.0f, scratch_))
                rig_->localToModel(scratch_, solved_);
        }
    };
    // How far the planted legs are past their real ranges (rad): hip, knee,
    // and the ankle under a foot that stays as it is on the ground.
    auto pastRange = [&] {
        if (!limits_) return 0.0f;
        float worst = 0.0f;
        auto check = [&](int joint, math::Quat local) {
            const math::Quat q = math::normalize(local);
            worst = std::max(worst, math::angleBetween(q, ik::clampRotation(q, limits_->limit(joint))));
        };
        for (int i = 0; i < 2; ++i) {
            const ik::TwoBoneChain& leg = joints_.legs[i];
            if (leg.start < 0 || goals.foot[i].active) continue;
            check(leg.start, scratch_.locals[static_cast<size_t>(leg.start)].rotation);
            check(leg.mid, scratch_.locals[static_cast<size_t>(leg.mid)].rotation);
            check(leg.end, math::inverse(planRot(leg.mid)) * rot(leg.end));
        }
        return worst;
    };
    // Legs within range: the hips rise until the legs can hold them.
    constexpr int kRaises = 24;
    constexpr float kRaise = 0.02f;      // m per step
    constexpr float kInRange = 0.01f;    // rad
    auto solveInRange = [&] {
        solveLegs();
        for (int k = 0; k < kRaises && pastRange() > kInRange; ++k) {
            math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
            t.position.y += kRaise;
            placePlannedPelvis(t);
            solveLegs();
        }
    };
    constexpr int kPasses = 4;
    constexpr float kOver = 1e-3f;  // m: the centre of mass over the feet
    for (int pass = 0; pass < kPasses; ++pass) {
        solveInRange();
        if (totalMass_ <= 0.0f) break;
        const math::Vec3 com = planCentreOfMass();
        const math::Vec3 off(support.x - com.x, 0.0f, support.z - com.z);
        if (math::length(off) <= kOver) break;
        math::Transform t = math::toTransform(solved_[static_cast<size_t>(joints_.pelvis)]);
        t.position += off;  // the upper body moves with the hips; the next pass measures again
        placePlannedPelvis(t);
        if (pass == kPasses - 1) solveInRange();  // the legs under the last placement
    }
    for (int i = 0; i < 2; ++i)
        if (joints_.legs[i].start >= 0 && !goals.foot[i].active) {
            markPlanned(joints_.legs[i].start, out);
            markPlanned(joints_.legs[i].mid, out);
        }
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
    // The brain's decision for the limb (DECISIONS 2026-11-07): the pose a
    // two-bone solve gives the shoulder and elbow (hip and knee), as each
    // joint's target rotation; the muscles pull to it and hold it (Jolt's
    // position motors). The solve starts from the limb's rest rotations on
    // the body's real parents (clavicle, pelvis), so the target never follows
    // the limb's own drift (a solve from the drifting pose fed it back: the
    // forearm's twist crept and the shoulder chased it).
    const size_t a = static_cast<size_t>(chain.start), b = static_cast<size_t>(chain.mid);
    scratch_.locals[a].rotation = rig_->restPose().locals[a].rotation;
    scratch_.locals[b].rotation = rig_->restPose().locals[b].rotation;
    if (rig_->localToModel(scratch_, solved_) && ik::solveTwoBone(chain, solved_, target, pole, 1.0f, scratch_) &&
        rig_->localToModel(scratch_, solved_)) {
        for (int j : {chain.start, chain.mid}) {
            // The way it turns now: its own turn (its parent's turn in the
            // solve taken out), in character space.
            const int parent = rig_->parent(j);
            const math::Quat now = rot(j);
            const math::Quat then = math::toTransform(solved_[static_cast<size_t>(j)]).rotation;
            const math::Quat parentTurn =
                parent >= 0 ? math::toTransform(solved_[static_cast<size_t>(parent)]).rotation * math::inverse(rot(parent))
                            : math::identityQuat();
            const math::Vec3 turn = math::rotationVector(math::inverse(parentTurn) * then * math::inverse(now));
            const float angle = math::length(turn);
            if (angle > kMinAngle) add(out, j, PartMove::Turn, turn / angle, velocity, goal, true);
            markPlanned(j, out);  // at its target or not, it follows the plan
        }
    } else {
        rig_->localToModel(scratch_, solved_);  // the plan stays whole (the limb at rest)
        markPlanned(chain.start, out);
        markPlanned(chain.mid, out);
    }
}

void BodyDecider::turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                            const std::vector<uint8_t>& blocked, const BodyDeciderSettings& s, BodyQueue& out) {
    const math::Vec3 axis = axisOf(delta);
    if (math::length(axis) <= 0.0f) return;
    const int first = out.count;
    for (int k = 0; k < count; ++k) {
        const int j = joints[k];
        if (j < 0) continue;
        add(out, j, PartMove::Turn, axis, velocity, goal, false, isBlocked(blocked, j) ? 0.0f : 1.0f);
    }
    shareQueue(first, s, out);
}

void BodyDecider::shareQueue(int first, const BodyDeciderSettings& s, BodyQueue& out) const {
    // Orders come in with share 1 (free) or 0 (blocked); the k-th free one by
    // place carries queueShare^k, then all are normalised to sum to 1.
    float weight = 1.0f, total = 0.0f;
    for (int i = first; i < out.count; ++i) {
        PartOrder& o = out.orders[static_cast<size_t>(i)];
        o.share *= weight;
        total += o.share;
        weight *= s.queueShare;
    }
    for (int i = first; i < out.count; ++i) {
        PartOrder& o = out.orders[static_cast<size_t>(i)];
        o.share = total > 0.0f ? o.share / total : 0.0f;
        o.active = o.share > 0.0f;
    }
}

} // namespace motion
