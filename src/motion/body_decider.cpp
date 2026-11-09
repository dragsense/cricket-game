#include "motion/body_decider.h"

#include "ik/joint_limits.h"
#include "motion/reach.h"

#include <algorithm>
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
    lastCentre_.assign(n, math::Vec3(0.0f));
    armOf_.assign(n, -1);
    for (int j = 0; j < static_cast<int>(n); ++j)
        for (int a = j; a >= 0; a = rig.parent(a))
            if (a == joints.arms[0].start || a == joints.arms[1].start) {
                armOf_[static_cast<size_t>(j)] = static_cast<int8_t>(a == joints.arms[0].start ? 0 : 1);
                break;
            }
    scratch_ = rig.restPose();
    if (!rig.localToModel(rig.restPose(), models_)) return false;
    restPelvis_ = math::toTransform(models_[static_cast<size_t>(joints.pelvis)]);
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints.legs[i];
        if (leg.start >= 0) {
            legLength_[i] = math::distance(pos(leg.start), pos(leg.mid)) + math::distance(pos(leg.mid), pos(leg.end));
            footToPelvis_[i] = math::inverse(rot(leg.end)) * rot(joints.pelvis);
            restFoot_[i] = rot(leg.end);
        }
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
    if (joints.head >= 0) restHead_ = rot(joints.head);
    for (int h = 0; h < 2; ++h)
        if (joints.arms[h].end >= 0) restHand_[h] = pos(joints.arms[h].end);
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

bool BodyDecider::refreshPlan(int from) { return rig_->localToModel(scratch_, solved_, from); }

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
    if (limits_) local = ik::clampRotation(local, limits_->limitIn(j, scratch_));
    scratch_.locals[static_cast<size_t>(j)].rotation = local;
    refreshPlan(j);
}

void BodyDecider::placePlannedPelvis(const math::Transform& t) {
    const int parent = rig_->parent(joints_.pelvis);
    math::Transform& local = scratch_.locals[static_cast<size_t>(joints_.pelvis)];
    const math::Transform placed =
        parent >= 0 ? math::inverse(math::toTransform(solved_[static_cast<size_t>(parent)])) * t : t;
    local.position = placed.position;
    local.rotation = math::normalize(placed.rotation);
    refreshPlan(joints_.pelvis);
    pelvisPlanned_ = true;
}

void BodyDecider::movePlannedPelvis(math::Vec3 delta) {
    // A move without a turn shifts the pelvis and everything below it by
    // `delta` (no skeleton pass needed: the same result).
    const int pelvis = joints_.pelvis;
    const int parent = rig_->parent(pelvis);
    const math::Vec3 local =
        parent >= 0 ? math::inverse(math::toTransform(solved_[static_cast<size_t>(parent)]).rotation) * delta : delta;
    scratch_.locals[static_cast<size_t>(pelvis)].position += local;
    for (int j = pelvis; j < rig_->subtreeEnd(pelvis); ++j) solved_[static_cast<size_t>(j)][3] += math::Vec4(delta, 0.0f);
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
    refreshPlan();
    pelvisPlanned_ = false;
    out.plan = &solved_;

    // Hips: the legs carry them to their target (the planned pelvis there).
    if (goals.hips.active) {
        const math::Vec3 dir = math::safeNormalize(goals.hips.position - pos(joints_.pelvis));
        if (math::length(dir) > 0.0f)
            add(out, joints_.pelvis, PartMove::Move, dir, goals.hips.velocity, Goal::Hips, true);
        movePlannedPelvis(goals.hips.position - planPos(joints_.pelvis));
    }

    // The trunk for the hands and the lifted feet, then the planted legs
    // under it (the plan's centre of mass kept over the planted feet), then
    // the arms from the planned shoulders and the lifted legs from the
    // planned hips. The plan comes from the targets, the planted feet and the
    // joint limits only, never from the body as it lags behind it (a plan
    // from the moving body fed its lag back and jumped from tick to tick):
    // while a hand or a lifted foot has a goal the trunk starts from standing
    // (the spine at rest, the pelvis as it stands), so it also straightens
    // after a far reach.
    // A foot moves when it has a goal and is off the ground; a foot down with
    // a goal stands pinned on that spot.
    auto moving = [&](int i) { return goals.foot[i].active && !goals.down[i]; };
    const bool planning = goals.hand[0].active || goals.hand[1].active || goals.foot[0].active || goals.foot[1].active;
    for (int i = 0; i < 2; ++i) {
        const int ankle = joints_.legs[i].end;
        const bool planted = ankle >= 0 && goals.down[i];
        if (!planted || !planning) anchored_[i] = false;
        if (planted && !anchored_[i]) {
            anchor_[i] = pos(ankle);
            // Flat on the ground and facing the body's way, as standing (the
            // plan's foot, not the body's: a foot the body twisted or tilted
            // on its way down is not pinned so; that carried from step to
            // step). The root turning later leaves it where it is (rebase).
            anchorRot_[i] = restFoot_[i];
            anchored_[i] = true;
        }
        if (planted && goals.foot[i].active) anchor_[i] = goals.foot[i].position;  // pinned on its given spot
    }
    if (planning) {
        const rig::Pose& rest = rig_->restPose();
        for (int j : joints_.spine)
            if (j >= 0) {
                scratch_.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
                markPlanned(j, out);
            }
        placePlannedPelvis(restPelvis_);
    }
    for (int e = 0; e < kEnds; ++e) trunkFirst_[e] = trunkEnd_[e] = -1;
    for (int h = 0; h < 2; ++h)
        if (goals.hand[h].active) handTrunk(h, goals.hand[h], blocked, s, out);
    for (int i = 0; i < 2; ++i)
        if (moving(i)) footTrunk(i, goals.foot[i], s, out);
    planLegs(goals, s, out);
    // The reach completes: the legs' balance moves the hips (and the
    // shoulders and the lifted leg's hip with them), so the trunk and the
    // legs go again, each part within its range, until every end's root is
    // within reach or no part gives any more (all at their limits).
    auto endGoal = [&](int e) -> const BodyGoals::Point& { return e < 2 ? goals.hand[e] : goals.foot[e - 2]; };
    if (planning) {
        auto shortOf = [&] {
            float sum = 0.0f;
            for (int e = 0; e < kEnds; ++e)
                if ((e < 2 ? endGoal(e).active : moving(e - 2)) && endRoot(e) >= 0)
                    sum += std::max(0.0f, math::distance(planPos(endRoot(e)), endGoal(e).position) - endReach(e));
            return sum;
        };
        constexpr int kRounds = 16;
        constexpr float kProgress = 1e-3f;  // m per round: less is no progress
        float last = shortOf();
        for (int round = 0; round < kRounds && last > kProgress; ++round) {
            for (int e = 0; e < kEnds; ++e)
                if (trunkFirst_[e] >= 0) planTrunk(e, endGoal(e).position, trunkFirst_[e], trunkEnd_[e], out);
            planLegs(goals, s, out);
            const float now = shortOf();
            if (last - now < kProgress) break;
            last = now;
        }
    }
    // The hips travel to their new place (any goal's: a step's balance, a
    // reach's lean) on a minimum-jerk path in Fitts' time, from where the
    // body has them; a new place mid-way re-aims the rest of the path from
    // where the hips are (no jump). A moving foot's target already carries
    // the hips smoothly (planLegs): they are where it puts them. The legs follow them to the
    // planted feet.
    const bool footMoving = moving(0) || moving(1);
    if (planning && pelvisPlanned_ && goals.dt > 0.0f && !footMoving) {
        const math::Vec3 want = planPos(joints_.pelvis);
        constexpr float kRetarget = 1e-3f;  // m: a new place
        constexpr float kReaim = 0.95f;     // path share after which the rest is not re-aimed
        auto timeFor = [&](float distance) {
            return movementTime(distance, s.fittsWidth, s.fittsA, s.fittsB);
        };
        auto share = [](float time, float duration) {
            const float u = std::clamp(time / std::max(duration, 1e-3f), 0.0f, 1.0f);
            return u * u * u * (10.0f + u * (6.0f * u - 15.0f));  // minimum jerk
        };
        if (!hips_.on) {
            hips_.from = hips_.at = pos(joints_.pelvis);
            hips_.to = want;
            hips_.time = 0.0f;
            hips_.duration = timeFor(math::distance(hips_.from, want));
            hips_.on = true;
        } else {
            const bool moved = math::distance(want, hips_.to) > kRetarget;
            if (hips_.time >= hips_.duration) {
                if (moved) {
                    hips_.from = hips_.at;
                    hips_.time = 0.0f;
                    hips_.duration = timeFor(math::distance(hips_.at, want));
                    hips_.to = want;
                }
            } else if (moved) {
                hips_.to = want;
                const float w = share(hips_.time, hips_.duration);
                if (w < kReaim) hips_.from = (hips_.at - hips_.to * w) / (1.0f - w);
            }
        }
        hips_.time += goals.dt;
        hips_.at = math::lerp(hips_.from, hips_.to, share(hips_.time, hips_.duration));
        if (math::distance(hips_.at, want) > 1e-4f) {
            movePlannedPelvis(hips_.at - want);
            planLegs(goals, s, out, false);
        }
    } else {
        hips_.on = false;
    }
    if (planning && !goals.chest.active && !goals.hand[0].active && !goals.hand[1].active) {
        // The chest's own target (standing upright, moving with the root):
        // its passive share of the way back from where the plan bent it
        // (with a hand's goal the trunk serves that goal fully).
        const float k = std::clamp(s.passiveChest, 0.0f, 1.0f);
        const rig::Pose& rest = rig_->restPose();
        bool turned = false;
        for (int j : joints_.spine)
            if (j >= 0 && k > 0.0f) {
                math::Quat& q = scratch_.locals[static_cast<size_t>(j)].rotation;
                q = math::normalize(math::slerp(q, rest.locals[static_cast<size_t>(j)].rotation, k));
                turned = true;
            }
        if (turned && joints_.spine[0] >= 0) refreshPlan(joints_.spine[0]);
    }
    if (planning) {
        freeArms(goals, s, out);
        if (!goals.head.active && joints_.head >= 0) {
            // A head without a goal keeps the gaze level: upright, facing the
            // chest's heading (head stabilisation in walking, Pozzo et al.
            // 1990), not left where the body swung it.
            const int parent = rig_->parent(joints_.head);
            const int below = joints_.spine[2] >= 0 ? joints_.spine[2] : parent;
            if (parent >= 0 && below >= 0) {
                if (joints_.neck >= 0)
                    scratch_.locals[static_cast<size_t>(joints_.neck)].rotation =
                        rig_->restPose().locals[static_cast<size_t>(joints_.neck)].rotation;
                refreshPlan(joints_.neck >= 0 ? joints_.neck : joints_.head);
                const math::Vec3 f = planRot(below) * math::kForward;
                // Level (gaze), pulled by the passive share toward its own
                // target: standing, facing the root's way.
                const math::Quat level = math::slerp(math::fromAxisAngle(math::kUp, std::atan2(f.x, f.z)) * restHead_, restHead_,
                                                     std::clamp(s.passiveHead, 0.0f, 1.0f));
                math::Quat& head = scratch_.locals[static_cast<size_t>(joints_.head)].rotation;
                head = math::normalize(math::inverse(planRot(parent)) * level);
                if (limits_) head = ik::clampRotation(head, limits_->limitIn(joints_.head, scratch_));
                refreshPlan(joints_.head);
                markPlanned(joints_.head, out);
            }
        }
    }
    else lastValid_ = false;
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
        if (moving(i)) footOrders(i, goals, blocked, s, out);
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

    const math::Vec3 shoulder = planPos(arm.start);

    // Out of the arm's reach every part of the queue moves from the start,
    // shared by its place (shareQueue). The queue is led by where the target
    // is (BIOMECHANICS 9.1, 9.3):
    //  - below the hips: the legs lower the body first (a pickup mixes squat
    //    and stoop; fielders bend the legs rather than the back, section 8),
    //    then a bend mostly at the hips (Esola et al. 1996): hips lower,
    //    pelvis tilt, spine_01, spine_02, chest, clavicle;
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

void BodyDecider::trunkOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& /*blocked*/,
                              const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    const Goal which = h == 0 ? Goal::HandL : Goal::HandR;
    const math::Vec3 target = goal.position, shoulder = planPos(arm.start);
    const float reach = armLength_[h];
    const math::Vec3 hips = planPos(joints_.pelvis) + planRot(joints_.pelvis) * hipCentre_;
    const math::Vec3 forward = planRot(joints_.pelvis) * math::kForward;
    enum Stage { Clavicle, Chest, Spine2, Spine1, Tilt, Move };
    static constexpr Stage kLow[6] = {Move, Tilt, Spine1, Spine2, Chest, Clavicle};
    static constexpr Stage kBehind[6] = {Chest, Spine2, Spine1, Clavicle, Tilt, Move};
    static constexpr Stage kAhead[6] = {Clavicle, Chest, Spine2, Spine1, Tilt, Move};
    const Stage* order = target.y < hips.y ? kLow : math::dot(target - shoulder, forward) < 0.0f ? kBehind : kAhead;

    const int first = out.count;
    int lumbar = -1, tilt = -1;  // their orders, for the measured split
    for (int k = 0; k < 6; ++k) {
        if (order[k] == Move) {
            // The legs move the hips toward the hand (the planted feet stay:
            // the legs bend); they cannot once both knees are blocked.
            add(out, joints_.pelvis, PartMove::Move, math::normalize(target - shoulder), goal.velocity, which, false);
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
        add(out, j, PartMove::Turn, sw.axis, goal.velocity, which, false);
    }
    // Shares by place; a part at its range in the plan leaves its part to the
    // others (planTrunk's passes), the joint limits being the plan's own.
    shareQueue(first, s, out);

    // A bend (not a turn): the lumbar spine and the hips split their combined
    // share by the measured lumbopelvic rhythm at the trunk's flexion now.
    if (order != kBehind && lumbar >= 0 && tilt >= 0 && joints_.spine[2] >= 0) {
        PartOrder& l = out.orders[static_cast<size_t>(lumbar)];
        PartOrder& p = out.orders[static_cast<size_t>(tilt)];
        if (l.share > 0.0f && p.share > 0.0f) {
            const math::Vec3 trunk = math::safeNormalize(planPos(joints_.spine[2]) - planPos(joints_.pelvis), math::kUp);
            const float flexion = std::acos(math::clamp(math::dot(trunk, math::kUp), -1.0f, 1.0f));
            int band = 0;
            while (band < 2 && flexion > s.lumbarHipBand[band]) ++band;
            const float ratio = s.lumbarHipRatio[band], both = l.share + p.share;
            l.share = both * ratio / (1.0f + ratio);
            p.share = both / (1.0f + ratio);
        }
    }
    trunkFirst_[h] = first;
    trunkEnd_[h] = out.count;
    planTrunk(h, target, first, out.count, out);
}

int BodyDecider::endRoot(int e) const { return e < 2 ? joints_.arms[e].start : joints_.legs[e - 2].start; }

float BodyDecider::endReach(int e) const { return e < 2 ? armLength_[e] : legLength_[e - 2]; }

math::Vec3 BodyDecider::tiltPivot(int e) const {
    // A hand's pelvis tilt turns at the hips; a lifted foot's at the hip of
    // the leg it stands on (the pelvis on the stance femur).
    const int stance = e >= 2 ? joints_.legs[1 - (e - 2)].start : -1;
    return stance >= 0 ? planPos(stance) : planPos(joints_.pelvis) + planRot(joints_.pelvis) * hipCentre_;
}

void BodyDecider::planTrunk(int e, math::Vec3 target, int first, int end, BodyQueue& out) {
    // In passes: each part of the queue turns by its share of the swing that
    // would bring the end's root (shoulder, hip) within reach on its own (the
    // hips lower by their share of the drop), within its range; the next pass
    // takes what is left (a part at its range leaves its part to the others).
    const int root = endRoot(e);
    const float reach = endReach(e);
    constexpr int kPasses = 4;
    constexpr float kClose = 1e-3f;  // m: within reach
    for (int pass = 0; pass < kPasses; ++pass) {
        for (int k = first; k < end; ++k) {
            const PartOrder& o = out.orders[static_cast<size_t>(k)];
            if (o.share <= 0.0f) continue;
            const math::Vec3 shoulder = planPos(root);  // or the lifted leg's hip
            const float gap = math::distance(shoulder, target) - reach;
            if (gap <= kClose) return;
            if (o.joint == joints_.pelvis && o.kind == PartMove::Move) {
                // The legs lower the hips (they bend; the planted feet stay)
                // by their share of the drop that alone would bring the
                // shoulder within reach. Shifting them over the feet is
                // balance's, not the reach's.
                movePlannedPelvis(-math::kUp * (dropToward(shoulder, target, reach) * o.share));
            } else if (o.joint == joints_.pelvis) {
                // The pelvis tilts at the hips (the hips stay).
                const math::Vec3 hips = tiltPivot(e);
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
        if (mass_[j] > 0.0f) sum += mass_[j] * math::Vec3(solved_[j] * math::Vec4(centre_[j], 1.0f));
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

void BodyDecider::rebase(const math::Transform& toNew) {
    for (int i = 0; i < 2; ++i) {
        anchor_[i] = toNew.transformPoint(anchor_[i]);
        anchorRot_[i] = toNew.rotation * anchorRot_[i];
    }
    for (math::Vec3* p : {&hips_.from, &hips_.to, &hips_.at}) *p = toNew.transformPoint(*p);
    for (math::Vec3& p : lastCentre_) p = toNew.transformPoint(p);
}

void BodyDecider::freeArms(const BodyGoals& goals, const BodyDeciderSettings& s, BodyQueue& out) {
    // The arm swing of walking: the arms are not held; they hang from the
    // shoulders and swing opposite the legs because they cancel part of the
    // body's turning about the vertical (Collins et al. 2009; Bruijn et al.
    // 2008). Each free arm here: its swing rate takes its share (by mass and
    // lever) of cancelling the rest of the plan's vertical angular momentum
    // about the centre of mass; gravity takes it back toward hanging at its
    // pendulum rate (sqrt(m g d / I) about the shoulder).
    bool free[2];
    for (int h = 0; h < 2; ++h) free[h] = !goals.hand[h].active && joints_.arms[h].start >= 0;
    const rig::Pose& rest = rig_->restPose();
    auto place = [&](int h) {  // hanging, swung by armAngle_ (forward +) within the shoulder's range
        const ik::TwoBoneChain& c = joints_.arms[h];
        for (int j : {joints_.clavicles[h], c.mid, c.end})
            if (j >= 0) scratch_.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
        math::Quat& shoulder = scratch_.locals[static_cast<size_t>(c.start)].rotation;
        shoulder = rest.locals[static_cast<size_t>(c.start)].rotation * math::fromAxisAngle(math::kRight, -armAngle_[h]);
        if (limits_) shoulder = ik::clampRotation(math::normalize(shoulder), limits_->limitIn(c.start, scratch_));
        refreshPlan(joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : c.start);
        markPlanned(c.start, out);
        markPlanned(c.mid, out);
    };
    for (int h = 0; h < 2; ++h)
        if (free[h]) place(h);
    if (totalMass_ <= 0.0f || goals.dt <= 0.0f) return;
    // The plan's turning about the vertical through its centre of mass (all
    // but the free arms), and each free arm's mass, lever and inertia about
    // its shoulder.
    const math::Vec3 com = planCentreOfMass();
    float spin = 0.0f, mass[2] = {0.0f, 0.0f}, lever[2] = {0.0f, 0.0f}, inertia[2] = {0.0f, 0.0f};
    for (size_t j = 0; j < mass_.size() && j < solved_.size() && j < centre_.size(); ++j) {
        if (mass_[j] <= 0.0f) continue;
        const math::Vec3 p(solved_[j] * math::Vec4(centre_[j], 1.0f));
        const int h = armOf_[j];
        if (h >= 0 && free[h]) {
            const float d = math::distance(p, planPos(joints_.arms[h].start));
            mass[h] += mass_[j];
            lever[h] += mass_[j] * d;
            inertia[h] += mass_[j] * d * d;
        } else if (lastValid_) {
            const math::Vec3 v = (p - lastCentre_[j]) / goals.dt;
            spin += mass_[j] * ((p.z - com.z) * v.x - (p.x - com.x) * v.z);
        }
        lastCentre_[j] = p;
    }
    const bool measured = lastValid_;
    lastValid_ = true;
    // An arm swinging forward at rate w turns the body by k w about the
    // vertical (k = -its side offset x its mass x lever); the free arms
    // share cancelling `spin` at the least swing (w = -spin k / sum k^2).
    float k[2] = {0.0f, 0.0f}, kk = 0.0f;
    for (int h = 0; h < 2; ++h)
        if (free[h] && mass[h] > 0.0f) {
            k[h] = -(planPos(joints_.arms[h].start).x - com.x) * lever[h];
            kk += k[h] * k[h];
        }
    for (int h = 0; h < 2; ++h) {
        if (!free[h] || mass[h] <= 0.0f) continue;
        const float cancel = measured && kk > 1e-9f ? -s.armCancel * spin * k[h] / kk : 0.0f;
        const float pendulum = inertia[h] > 0.0f ? std::sqrt(std::max(0.0f, s.gravity) * lever[h] / inertia[h]) : 0.0f;
        armAngle_[h] += (cancel - pendulum * armAngle_[h]) * goals.dt;
        place(h);
    }
    passiveHands(free, s, out);
}

void BodyDecider::passiveHands(const bool free[2], const BodyDeciderSettings& s, BodyQueue& out) {
    // A hand without a goal still has its target (hanging, as standing,
    // moving with the root): the swing goes on, and the hand goes the
    // passive share of the way from where the swing has it to that target
    // (the elbow bending back, as it hangs).
    const float k = std::clamp(s.passiveHands, 0.0f, 1.0f);
    if (k <= 0.0f) return;
    const math::Quat chest = joints_.spine[2] >= 0 ? planRot(joints_.spine[2]) : math::identityQuat();
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (!free[h] || arm.end < 0) continue;
        const math::Vec3 at = planPos(arm.end);
        const math::Vec3 want = at + k * (restHand_[h] - at);
        if (ik::solveTwoBone(arm, solved_, want, chest * hangBend_[h], 1.0f, scratch_)) {
            if (limits_)
                for (int j : {arm.start, arm.mid})
                    scratch_.locals[static_cast<size_t>(j)].rotation =
                        ik::clampRotation(math::normalize(scratch_.locals[static_cast<size_t>(j)].rotation), limits_->limitIn(j, scratch_));
            refreshPlan(arm.start);
            markPlanned(arm.start, out);
            markPlanned(arm.mid, out);
        }
    }
}

void BodyDecider::planLegs(const BodyGoals& goals, const BodyDeciderSettings& s, BodyQueue& out, bool balance) {
    if (!pelvisPlanned_) return;
    // The planted feet stay where they are; the legs reach them from the
    // planned hips, and the hips go where the plan's centre of mass stays over
    // the feet (a bend takes the hips back: anticipatory posture, Massion
    // 1992). The hips cannot go higher than the straight legs allow.
    // The centre of mass goes between the feet's targets (planted or on
    // their way: a moving foot's progress carries the hips with it, half its
    // travel, as the body's centre moves one step while the foot moves a
    // stride), toward the standing foot and back through a timed swing
    // (hipsSway). The same for a step, both feet (a jump) or none.
    math::Vec3 at[2];
    bool has[2];
    int count = 0;
    for (int i = 0; i < 2; ++i) {
        // A foot held up without timing bears no weight and is not on its
        // way: the body stands on the other (all of it over that foot).
        has[i] = anchored_[i] || (goals.foot[i].active && goals.foot[i].swing >= 0.0f);
        at[i] = anchored_[i] ? anchor_[i] : goals.foot[i].position;
        count += has[i] ? 1 : 0;
    }
    if (count == 0) return;
    math::Vec3 support = count == 2 ? 0.5f * (at[0] + at[1]) : (has[0] ? at[0] : at[1]);
    if (count == 2 && anchored_[0] != anchored_[1]) {
        const int up = anchored_[0] ? 1 : 0, down = 1 - up;
        const float share = std::sin(math::kPi * std::clamp(goals.foot[up].swing, 0.0f, 1.0f));
        math::Vec3 side = at[down] - at[up];
        side.y = 0.0f;
        const float width = math::length(side);
        if (width > 1e-4f) support += side * (std::min(s.hipsSway, 0.5f * width) * share / width);
    }

    // The hips' own target (standing, moving with the root) has its passive
    // share when it is not the goal (a hips goal is followed exactly).
    if (!goals.hips.active) {
        const float k = std::clamp(s.passiveHips, 0.0f, 1.0f);
        support.x += k * (restPelvis_.position.x - support.x);
        support.z += k * (restPelvis_.position.z - support.z);
    }

    const rig::Pose& rest = rig_->restPose();
    auto solveLegs = [&] {
        for (int pass = 0; pass < 2; ++pass)
            for (int i = 0; i < 2; ++i) {
                const ik::TwoBoneChain& leg = joints_.legs[i];
                if (leg.start < 0 || !anchored_[i]) continue;
                const math::Vec3 hip = planPos(leg.start), ankle = anchor_[i];
                const float over = math::distance(hip, ankle) - legLength_[i];
                if (over <= 0.0f) continue;
                movePlannedPelvis(math::normalize(ankle - hip) * over);
            }
        for (int i = 0; i < 2; ++i) {
            const ik::TwoBoneChain& leg = joints_.legs[i];
            if (leg.start < 0 || !anchored_[i]) continue;
            for (int j : {leg.start, leg.mid})
                scratch_.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            refreshPlan(leg.start);
            // The knee tracks the planted foot (over the toes, as in a
            // squat): its pole turns with the foot from standing, the hip's
            // rotation taking the difference.
            const math::Vec3 pole = anchorRot_[i] * footToPelvis_[i] * s.kneePole;
            if (ik::solveTwoBone(leg, solved_, anchor_[i], pole, 1.0f, scratch_))
                refreshPlan(leg.start);
        }
    };
    // How far the planted legs are past their real ranges (rad): hip, knee,
    // and the ankle under a foot that stays as it is on the ground.
    auto pastRange = [&] {
        if (!limits_) return 0.0f;
        float worst = 0.0f;
        auto check = [&](int joint, math::Quat local) {
            const math::Quat q = math::normalize(local);
            const float past = math::angleBetween(q, ik::clampRotation(q, limits_->limitIn(joint, scratch_)));
            worst = std::max(worst, past);
        };
        for (int i = 0; i < 2; ++i) {
            const ik::TwoBoneChain& leg = joints_.legs[i];
            if (leg.start < 0 || !anchored_[i]) continue;
            check(leg.start, scratch_.locals[static_cast<size_t>(leg.start)].rotation);
            check(leg.mid, scratch_.locals[static_cast<size_t>(leg.mid)].rotation);
            check(leg.end, math::inverse(planRot(leg.mid)) * anchorRot_[i]);
        }
        return worst;
    };
    // Legs within range: the hips rise the least that lets the legs hold
    // them (found by halving, so the height changes smoothly from tick to
    // tick, never in fixed steps).
    constexpr float kMaxRaise = 0.48f;  // m
    constexpr int kHalvings = 8;        // ~2 mm
    constexpr float kInRange = 0.01f;   // rad
    auto solveInRange = [&] {
        solveLegs();
        if (pastRange() <= kInRange) return;
        float low = 0.0f, high = kMaxRaise, at = 0.0f;
        auto raiseTo = [&](float h) {
            movePlannedPelvis(math::kUp * (h - at));
            at = h;
            solveLegs();
        };
        raiseTo(high);
        if (pastRange() > kInRange) return;  // no height holds them: as high as it goes
        for (int k = 0; k < kHalvings; ++k) {
            const float mid = 0.5f * (low + high);
            raiseTo(mid);
            if (pastRange() > kInRange) low = mid;
            else high = mid;
        }
        raiseTo(high);
    };
    constexpr int kPasses = 4;
    constexpr float kOver = 1e-3f;  // m: the centre of mass over the feet
    // A foot on its way (a step, a jump): the hips go between the feet at
    // once, carried by the feet's targets; the centre of mass is kept over
    // the feet only standing (walking is not statically balanced: the body
    // vaults over the stance leg on its own momentum).
    const bool feetMoving = (goals.foot[0].active && !goals.down[0]) || (goals.foot[1].active && !goals.down[1]);
    if (balance && feetMoving) {
        const math::Vec3 hips = planPos(joints_.pelvis);
        movePlannedPelvis(math::Vec3(support.x - hips.x, 0.0f, support.z - hips.z));
    }
    if (!balance || feetMoving) solveInRange();
    for (int pass = 0; balance && !feetMoving && pass < kPasses; ++pass) {
        solveInRange();
        if (totalMass_ <= 0.0f) break;
        const math::Vec3 com = planCentreOfMass();
        const math::Vec3 off(support.x - com.x, 0.0f, support.z - com.z);
        if (math::length(off) <= kOver) break;
        movePlannedPelvis(off);  // the upper body moves with the hips; the next pass measures again
        if (pass == kPasses - 1) solveInRange();  // the legs under the last placement
    }
    for (int i = 0; i < 2; ++i)
        if (joints_.legs[i].start >= 0 && anchored_[i]) {
            markPlanned(joints_.legs[i].start, out);
            markPlanned(joints_.legs[i].mid, out);
        }
}

void BodyDecider::footTrunk(int i, const BodyGoals::Point& goal, const BodyDeciderSettings& s, BodyQueue& out) {
    // A lifted foot out of its leg's reach from the planned hip: the leg it
    // stands on bends (its hip and knee lower the hips: the reach distance of
    // a one-leg foot reach goes with the stance leg's hip and knee flexion,
    // Robinson & Gribble 2008, BIOMECHANICS 9.7), then the pelvis tilts on
    // the stance hip; the trunk's counter-lean comes from the plan's centre
    // of mass kept over the stance foot (planLegs). Shares by place.
    const ik::TwoBoneChain& leg = joints_.legs[i];
    if (leg.start < 0) return;
    const Goal which = i == 0 ? Goal::FootL : Goal::FootR;
    const math::Vec3 hip = planPos(leg.start), target = goal.position;
    if (math::distance(hip, target) <= legLength_[i]) return;
    const int e = 2 + i;
    const int first = out.count;
    if (dropToward(hip, target, legLength_[i]) > 0.0f)
        add(out, joints_.pelvis, PartMove::Move, math::normalize(target - hip), goal.velocity, which, false);
    const Swing sw = swingToward(tiltPivot(e), hip, target, legLength_[i]);
    if (sw.angle > kMinAngle) add(out, joints_.pelvis, PartMove::Turn, sw.axis, goal.velocity, which, false);
    shareQueue(first, s, out);
    trunkFirst_[e] = first;
    trunkEnd_[e] = out.count;
    planTrunk(e, target, first, out.count, out);
}

void BodyDecider::footOrders(int i, const BodyGoals& goals, const std::vector<uint8_t>& /*blocked*/,
                             const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& leg = joints_.legs[i];
    if (leg.start < 0) return;
    const Goal which = i == 0 ? Goal::FootL : Goal::FootR;
    const BodyGoals::Point& goal = goals.foot[i];

    // Hip and knee from the planned hip (footTrunk placed the pelvis), knee
    // forward from the planned pelvis.
    limbOrders(leg, goal.position, planRot(joints_.pelvis) * s.kneePole, goal.velocity, which, out);
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
    if (refreshPlan(chain.start) && ik::solveTwoBone(chain, solved_, target, pole, 1.0f, scratch_) &&
        refreshPlan(chain.start)) {
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
        refreshPlan(chain.start);  // the plan stays whole (the limb at rest)
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
