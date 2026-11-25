#include "motion/body_decider.h"

#include "core/log.h"
#include "motion/body_chain.h"
#include "ik/joint_limits.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace motion {

namespace {


// --- Limb reach math (law of cosines; BIOMECHANICS 9). Pure, no allocations. ---

constexpr float kReachEpsilon = 1e-4f;  // m; closer than this counts as reached

// `target` moved toward `from` so it is at most `reach` away.
math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach) {
    const math::Vec3 d = target - from;
    const float len = math::length(d);
    return len <= reach ? target : from + d * (reach / len);
}

// Turns smaller than this (rad) are no direction at all.
constexpr float kMinAngle = 1e-4f;

// A hand closer than this (m) to its goal is inside the arm's free room; the
// chain's least turn that brings it there is found in this many halvings.
constexpr float kFreeRoomEpsilon = 2e-3f;
constexpr int kFreeRoomSteps = 8;

void add(BodyQueue& out, int joint, PartMove kind, math::Vec3 direction, math::Vec3 velocity, Goal goal, bool active,
         float share = 1.0f) {
    assert(out.count < BodyQueue::kCapacity);
    if (out.count >= BodyQueue::kCapacity) return;
    out.orders[static_cast<size_t>(out.count++)] = {joint, kind, direction, velocity, goal, active, share};
}

math::Vec3 level(math::Vec3 v) { return math::Vec3(v.x, 0.0f, v.z); }

math::Vec3 clampLength(math::Vec3 v, float max) {
    const float len = math::length(v);
    return len > max && len > 0.0f ? v * (max / len) : v;
}

// A two-bone limb's span (root to end) with its middle joint bent `bend`.
float bentSpan(float a, float b, float bend) {
    return std::sqrt(std::max(a * a + b * b + 2.0f * a * b * std::cos(bend), 0.0f));
}

math::Vec3 shiftRate(math::Vec3 a, math::Vec3 v0, math::Vec3 b, float T, float u) {
    const float u2 = u * u, u3 = u2 * u, u4 = u3 * u;
    const float dm = 30.0f * u2 - 60.0f * u3 + 30.0f * u4;
    const float dh = 1.0f - 18.0f * u2 + 32.0f * u3 - 15.0f * u4;
    return T > 1e-6f ? (b - a) * (dm / T) + v0 * dh : v0;
}

// `x` moved toward `want` by at most `step` (a ramp, never a switch).
float ramp(float x, float want, float step) { return x + std::clamp(want - x, -step, step); }

} // namespace

bool BodyDecider::init(const rig::Rig& rig, const BodyDeciderJoints& joints, const ik::JointLimits* limits) {
    rig_ = &rig;
    limits_ = limits;
    joints_ = joints;
    if (joints.pelvis < 0 || rig.numJoints() > BodyQueue::kMaxJoints) return false;
    const size_t n = static_cast<size_t>(rig.numJoints());
    output_.pose = rig.restPose();
    output_.models.resize(n);
    desired_.pose = rig.restPose();
    desired_.models.resize(n);
    armOf_.assign(n, -1);
    for (int j = 0; j < static_cast<int>(n); ++j)
        for (int a = j; a >= 0; a = rig.parent(a))
            if (a == joints.arms[0].start || a == joints.arms[1].start) {
                armOf_[static_cast<size_t>(j)] = static_cast<int8_t>(a == joints.arms[0].start ? 0 : 1);
                break;
            }
    if (!refresh(output_)) return false;
    const Plan& r = output_;  // the rest pose
    restPelvis_ = pelvisOf(r);
    const math::Quat pelvisInv = math::inverse(restPelvis_.rotation);
    const math::Quat chestInv = joints.spine[2] >= 0 ? math::inverse(planRot(r, joints.spine[2])) : math::identityQuat();
    const math::Vec3 chestPos = joints.spine[2] >= 0 ? planPos(r, joints.spine[2]) : math::Vec3(0.0f);
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints.legs[i];
        if (leg.start >= 0) {
            thighLength_[i] = math::distance(planPos(r, leg.start), planPos(r, leg.mid));
            shinLength_[i] = math::distance(planPos(r, leg.mid), planPos(r, leg.end));
            footToPelvis_[i] = math::inverse(planRot(r, leg.end)) * restPelvis_.rotation;
            restFoot_[i] = planRot(r, leg.end);
            anchor_[i] = planPos(r, leg.end);
            anchorRot_[i] = restFoot_[i];
        }
        const ik::TwoBoneChain& arm = joints.arms[i];
        if (arm.start >= 0) {
            upperArmLength_[i] = math::distance(planPos(r, arm.start), planPos(r, arm.mid));
            forearmLength_[i] = math::distance(planPos(r, arm.mid), planPos(r, arm.end));
            const math::Vec3 hang = math::safeNormalize(planPos(r, arm.end) - planPos(r, arm.start), -math::kUp);
            const math::Vec3 back = -math::kForward - hang * math::dot(-math::kForward, hang);
            hangDir_[i] = chestInv * hang;
            hangBend_[i] = chestInv * math::safeNormalize(back, -math::kForward);
            restHandChest_[i] = chestInv * (planPos(r, arm.end) - chestPos);
        }
    }
    if (joints.legs[0].start >= 0 && joints.legs[1].start >= 0) {
        const math::Vec3 mid = 0.5f * (planPos(r, joints.legs[0].start) + planPos(r, joints.legs[1].start));
    }
    seed(rig.restPose());
    return true;
}

void BodyDecider::setMasses(const std::vector<float>& mass, const std::vector<math::Vec3>& centre) {
    mass_ = mass;
    centre_ = centre;
    totalMass_ = 0.0f;
    for (float m : mass_) totalMass_ += m;
}

void BodyDecider::seed(const rig::Pose& pose) {
    if (!rig_ || pose.locals.size() != output_.pose.locals.size()) return;
    output_.pose.locals = pose.locals;  // same size: no allocation
    refresh(output_);
    desired_.pose.locals = pose.locals;
    refresh(desired_);
    for (int i = 0; i < 2; ++i) {
        const int ankle = joints_.legs[i].end;
        if (ankle >= 0) anchor_[i] = planPos(output_, ankle);
        reachFrom_[i] = anchor_[i];
        anchorNormal_[i] = math::kUp;
        anchorRot_[i] = restFoot_[i];
        grounded_[i] = true;
        support_[i] = 1.0f;
    }
    for (float& a : act_) a = 0.0f;
    hipsRef_ = pelvisOf(output_).position;
    hipsRefVel_ = hipsVel_ = math::Vec3(0.0f);
    anchorYaw_[0] = anchorYaw_[1] = stanceYaw_ = 0.0f;
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (arm.start < 0) continue;
        freeArm_[h][0] = pose.locals[static_cast<size_t>(arm.start)].rotation;
        freeArm_[h][1] = pose.locals[static_cast<size_t>(arm.mid)].rotation;
        armHeld_[h] = false;
    }
    for (float& c : orderTravel_) c = 0.0f;
    orderCount_ = 0;
    orderEnd_ = orderEndWas_ = -1;
    limbOpen_ = true;
    farEnd_ = farNext_ = -1;
    for (ChainCache& c : chainCache_) c = {};
    seeded_ = true;
}

void BodyDecider::rebase(const math::Transform& toNew) {
    for (int i = 0; i < 2; ++i) {
        anchor_[i] = toNew.transformPoint(anchor_[i]);
        reachFrom_[i] = toNew.transformPoint(reachFrom_[i]);
        anchorNormal_[i] = toNew.rotation * anchorNormal_[i];
        anchorRot_[i] = toNew.rotation * anchorRot_[i];
    }
    for (Target& l : last_) {
        l.position = toNew.transformPoint(l.position);
        l.rotation = toNew.rotation * l.rotation;
        l.velocity = toNew.rotation * l.velocity;
        l.pathEnd = toNew.transformPoint(l.pathEnd);
        for (math::Vec3& d : l.directions) d = toNew.rotation * d;
    }
    for (math::Vec3& a : chainAim_) a = toNew.transformPoint(a);
    for (math::Vec3& a : footAim_) a = toNew.transformPoint(a);
    trialGoal_ = toNew.transformPoint(trialGoal_);
    hipsRef_ = toNew.transformPoint(hipsRef_);
    hipsRefVel_ = toNew.rotation * hipsRefVel_;
    hipsVel_ = toNew.rotation * hipsVel_;
    // A foot keeps its turn on the ground too.
    const math::Vec3 ahead = toNew.rotation * math::kForward;
    const float turned = std::atan2(ahead.x, ahead.z);
    for (float* yaw : {&anchorYaw_[0], &anchorYaw_[1], &stanceYaw_})
        *yaw += turned;
    if (!seeded_) return;
    placePelvis(output_, toNew * pelvisOf(output_));
    refresh(output_, joints_.pelvis);
}

// ---- plan access -----------------------------------------------------------

bool BodyDecider::refresh(Plan& p, int from) { return rig_->localToModel(p.pose, p.models, from); }

math::Vec3 BodyDecider::planPos(const Plan& p, int joint) const { return math::Vec3(p.models[static_cast<size_t>(joint)][3]); }

math::Quat BodyDecider::planRot(const Plan& p, int joint) const {
    return math::toTransform(p.models[static_cast<size_t>(joint)]).rotation;
}

math::Transform BodyDecider::pelvisOf(const Plan& p) const {
    return math::toTransform(p.models[static_cast<size_t>(joints_.pelvis)]);
}

void BodyDecider::mark(int joint, std::array<uint8_t, BodyQueue::kMaxJoints>& set) {
    if (joint >= 0 && joint < BodyQueue::kMaxJoints) set[static_cast<size_t>(joint)] = 1;
}

void BodyDecider::markPlanned(int joint, BodyQueue& out) const {
    if (joint >= 0 && joint < BodyQueue::kMaxJoints) out.planned[static_cast<size_t>(joint)] = 1;
}

void BodyDecider::turnPlanned(Plan& p, int j, math::Quat turn) {
    const int parent = rig_->parent(j);
    const math::Quat up = parent >= 0 ? planRot(p, parent) : math::identityQuat();
    math::Quat local = math::normalize(math::inverse(up) * turn * planRot(p, j));
    if (limits_) local = ik::clampRotation(local, limits_->limitIn(j, p.pose));
    p.pose.locals[static_cast<size_t>(j)].rotation = local;
    refresh(p, j);
}

void BodyDecider::placePelvis(Plan& p, const math::Transform& t) {
    const int parent = rig_->parent(joints_.pelvis);
    math::Transform& local = p.pose.locals[static_cast<size_t>(joints_.pelvis)];
    const math::Transform placed =
        parent >= 0 ? math::inverse(math::toTransform(p.models[static_cast<size_t>(parent)])) * t : t;
    local.position = placed.position;
    local.rotation = math::normalize(placed.rotation);
    refresh(p, joints_.pelvis);
}

math::Quat BodyDecider::limitTurn(math::Quat from, math::Quat to, float rate, float dt) {
    const float angle = math::angleBetween(from, to);
    const float most = std::max(rate, 0.0f) * dt;
    if (angle <= most || angle <= kMinAngle) return to;
    return math::normalize(math::slerp(from, to, most / angle));
}

void BodyDecider::limitJoint(int j, float rate, float dt) {
    if (j < 0) return;
    math::Quat& o = output_.pose.locals[static_cast<size_t>(j)].rotation;
    o = limitTurn(o, desired_.pose.locals[static_cast<size_t>(j)].rotation, rate, dt);
    clampJoint(output_, j);
}

bool BodyDecider::clampJoint(Plan& p, int j) const {
    if (!limits_ || j < 0) return false;
    math::Quat& local = p.pose.locals[static_cast<size_t>(j)].rotation;
    const math::Quat held = ik::clampRotation(math::normalize(local), limits_->limitIn(j, p.pose));
    const bool moved = math::angleBetween(held, local) > kMinAngle;
    local = held;
    return moved;
}

void BodyDecider::clampLimb(Plan& p, int e, const int* joints, int count, math::Vec3 aim) {
    bool moved = false;
    for (int k = 0; k < count; ++k) moved = clampJoint(p, joints[k]) || moved;
    refresh(p, joints[0]);
    if (&p != &output_) return;
    const int end = e < 2 ? joints_.arms[e].end : joints_.legs[e - 2].end;
    rangeMiss_[e] = moved && end >= 0 ? math::distance(planPos(p, end), aim) : 0.0f;
}

math::Vec3 BodyDecider::planCentreOfMass() const { return centreOfMass(desired_); }

math::Vec3 BodyDecider::centreOfMass(const Plan& p) const {
    math::Vec3 sum(0.0f);
    for (size_t j = 0; j < mass_.size() && j < p.models.size() && j < centre_.size(); ++j)
        if (mass_[j] > 0.0f) sum += mass_[j] * math::Vec3(p.models[j] * math::Vec4(centre_[j], 1.0f));
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

math::Quat BodyDecider::restFootLevel(int i) const {
    // Flat on its ground, heading as it stands.
    return math::normalize(math::fromTo(math::kUp, anchorNormal_[i]) * math::fromAxisAngle(math::kUp, anchorYaw_[i]) * restFoot_[i]);
}

// ---- the tick -----------------------------------------------------------------

void BodyDecider::decide(const TargetSet& given, const BodyDeciderSettings& s, BodyQueue& out) {
    out.count = 0;
    out.plan = nullptr;
    out.planned.fill(0);
    out.active.fill(0);
    out.holding.fill(0);
    out.heightClamped = false;
    out.landed[0] = out.landed[1] = false;
    for (ReachReport& r : out.reach) r = {};
    if (!rig_) return;
    if (!seeded_) seed(rig_->restPose());
    out.plan = &output_.models;
    for (int e = 0; e < kEnds; ++e) {
        rangeMiss_[e] = 0.0f;
    }

    work_ = given;  // no allocation (a plain struct)
    // The chain's far end (last decide): a driven end, its target where the
    // chain's turn put it.
    for (int i = 0; i < 2; ++i) {
        handGiven_[i] = given.hand[i].mode == TargetMode::Driven;
        footGiven_[i] = given.foot[i].mode == TargetMode::Driven;
    }
    if (farEnd_ >= 0 && !(farEnd_ < 2 ? handGiven_[farEnd_] : footGiven_[farEnd_ - 2])) {
        Target& f = farEnd_ < 2 ? work_.hand[farEnd_] : work_.foot[farEnd_ - 2];
        f = Target{};
        f.mode = TargetMode::Driven;
        f.position = farTarget_;
        f.rotation = farRot_;
    }
    const TargetSet& t = work_;
    blendWeights(t, s);
    pinFeet(t, s);
    rangePelvis(s);
    supportAndHips(t, ranged_, out);
    desiredPose(t, ranged_, out);
    limitOutput(t, ranged_, out);
    report(t, ranged_, out);
    farEnd_ = farNext_;
}

void BodyDecider::blendWeights(const TargetSet& t, const BodyDeciderSettings& s) {
    // Every activation ramps toward its mode (a Driven target 1, else 0) at
    // 1 / blendTime: no target switches on or off at once.
    const float step = t.dt / std::max(s.blendTime, 1e-3f);
    const Target* targets[kTargets] = {&t.hand[0], &t.hand[1], &t.foot[0], &t.foot[1], &t.hips, &t.chest, &t.head};
    for (int k = 0; k < kTargets; ++k) {
        const bool driven = targets[k]->mode == TargetMode::Driven;
        if (driven) last_[k] = *targets[k];
        act_[k] = ramp(act_[k], driven ? 1.0f : 0.0f, step);
    }
}

const Target& BodyDecider::given(const TargetSet& t, int k) const {
    const Target* targets[kTargets] = {&t.hand[0], &t.hand[1], &t.foot[0], &t.foot[1], &t.hips, &t.chest, &t.head};
    return targets[k]->mode == TargetMode::Driven ? *targets[k] : last_[k];
}

void BodyDecider::pinFeet(const TargetSet& t, const BodyDeciderSettings& s) {
    for (int i = 0; i < 2; ++i) {
        const int ankle = joints_.legs[i].end;
        if (ankle < 0) continue;
        const Target& f = t.foot[i];
        if (f.mode == TargetMode::Pinned) {
            // On its given spot, flat to the ground there; the planner says
            // where it landed.
            anchor_[i] = f.position;
            anchorNormal_[i] = math::safeNormalize(f.contactNormal, math::kUp);
            anchorRot_[i] = restFootLevel(i);
            grounded_[i] = true;
            reachFrom_[i] = anchor_[i];
        } else if (f.mode == TargetMode::Driven) {
            // On its way: the anchor follows the foot, so a foot let go is
            // pinned where it is. On the ground until it lifts clear of
            // where it was pinned (the weight shift before a step).
            const float groundY = anchor_[i].y;
            const math::Vec3 at = planPos(output_, ankle);
            grounded_[i] = grounded_[i] && at.y <= groundY + s.onGround;
            anchor_[i] = math::Vec3(at.x, groundY, at.z);  // the ground under it
            // In the air on a step it turns as its target does, and lands so.
            if (!grounded_[i] && f.swing >= 0.0f) {
                const math::Vec3 toes = f.rotation * math::kForward;
                anchorYaw_[i] = std::atan2(toes.x, toes.z);
            }
            // The leg reaches where it is solved to (solveLegs), the heel risen.
            const float k = act_[FootLW + i] * std::clamp(f.weightPosition, 0.0f, 1.0f);
            reachFrom_[i] = math::lerp(anchor_[i], f.position, k);
        } else {
            // Free: pinned where it last was. A foot lifted and let go lands
            // on its anchor (the planner pins it once it is down).
            grounded_[i] = true;
            reachFrom_[i] = anchor_[i];
        }
    }
}

void BodyDecider::supportAndHips(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Support weights: a pinned foot carries weight, a foot on its way (even
    // still on the ground: the weight shift) or free of the ground none;
    // each ramps at 1 / supportBlendTime (Winter 1995's load/unload).
    const float step = t.dt / std::max(s.supportBlendTime, 1e-3f);
    // The hips stay where they are (the user, 2026-11-23: only turns); a
    // driven hips target moves them (below).
    for (int i = 0; i < 2; ++i) {
        const bool pinned = joints_.legs[i].end >= 0 && t.foot[i].mode == TargetMode::Pinned;
        support_[i] = ramp(support_[i], pinned ? 1.0f : 0.0f, step);
    }
    math::Vec3 ref = hipsRef_;
    hipsRef_ = ref;
    // Driven hips: the given place along its active directions, by the
    // blended weight; the other directions stay the support's.
    const Target& hips = given(t, HipsW);
    const float k = act_[HipsW] * std::clamp(hips.weightPosition, 0.0f, 1.0f);
    if (k > 0.0f) {
        ref += drivenPart(hips, hips.position - ref) * k;
        const math::Vec3 dir = math::safeNormalize(ref - pelvisOf(output_).position);
        if (math::length(dir) > 0.0f) add(out, joints_.pelvis, PartMove::Move, dir, hips.velocity, Goal::Hips, true);
    }
    out.supportWeight[0] = support_[0];
    out.supportWeight[1] = support_[1];
    out.hipsReference = ref;
}

void BodyDecider::desiredPose(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // The desired pose comes from the targets alone, never from the lagging
    // body: the trunk starts upright over the hips' reference (so it also
    // straightens after a reach), the limbs from the output (warm start).
    Plan& d = desired_;
    legChained_ = false;
    heldLeg_ = -1;
    farNext_ = -1;
    orderEnd_ = -1;  // the chains decide below which leads (solveLegs, solveArms)
    orderCount_ = 0;
    hipsDown_ = math::Vec3(0.0f);
    d.pose.locals = output_.pose.locals;  // same size: no allocation
    const rig::Pose& rest = rig_->restPose();
    for (int j : {joints_.spine[0], joints_.spine[1], joints_.spine[2], joints_.neck, joints_.head, joints_.clavicles[0],
                  joints_.clavicles[1]})
        if (j >= 0) {
            d.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            markPlanned(j, out);
        }
    // A free arm stands as it was let go (a foot's chain turns it from there).
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (arm.start < 0) continue;
        // The planner's own (a chain's far end stands from the let-go pose
        // too: the chain turns it from there each tick).
        const bool driven = handGiven_[h];
        if (!driven) {
            if (armHeld_[h]) {
                freeArm_[h][0] = output_.pose.locals[static_cast<size_t>(arm.start)].rotation;
                freeArm_[h][1] = output_.pose.locals[static_cast<size_t>(arm.mid)].rotation;
            }
            d.pose.locals[static_cast<size_t>(arm.start)].rotation = freeArm_[h][0];
            d.pose.locals[static_cast<size_t>(arm.mid)].rotation = freeArm_[h][1];
        }
        armHeld_[h] = driven;
    }
    refresh(d);
    stanceYaw_ = stanceNow();
    float pelvisYaw = stanceYaw_;
    math::Quat pelvisRot = math::fromAxisAngle(math::kUp, pelvisYaw) * restPelvis_.rotation;
    {
        // A driven hips target turns the pelvis by its rotation weight, as
        // every driven target turns its part; within the pelvis's range from
        // standing (pelvisMin / pelvisMax, rotation vector, character axes).
        const Target& hips = given(t, HipsW);
        const float kr = act_[HipsW] * std::clamp(hips.weightRotation, 0.0f, 1.0f);
        if (kr > 0.0f) {
            const math::Quat want = math::normalize(math::slerp(pelvisRot, hips.rotation, kr));
            math::Vec3 v = math::rotationVector(want * math::inverse(pelvisBase()));
            for (int a = 0; a < 3; ++a) v[a] = std::clamp(v[a], s.pelvisMin[a], s.pelvisMax[a]);
            const float angle = math::length(v);
            pelvisRot = (angle > 1e-6f ? math::fromAxisAngle(v / angle, angle) : math::identityQuat()) * pelvisBase();
        }
    }
    placePelvis(d, math::Transform{out.hipsReference, pelvisRot});

    solveLegs(d, t, s, out);
    // A driven chest: by its blended weight, the chest first, the rest down
    // the chain to the pelvis (passUp).
    if (act_[ChestW] > 0.0f && joints_.spine[2] >= 0) {
        const Target& chest = given(t, ChestW);
        const float k = act_[ChestW] * std::clamp(chest.weightRotation, 0.0f, 1.0f);
        math::Quat delta = math::slerp(math::identityQuat(), chest.rotation * math::inverse(planRot(d, joints_.spine[2])), k);
        // A chest place: the trunk bends from the lumbar spine so the chest
        // goes toward it (the same chain, each joint within its range).
        const float kp = act_[ChestW] * std::clamp(chest.weightPosition, 0.0f, 1.0f);
        if (kp > 0.0f && joints_.spine[0] >= 0) {
            const math::Vec3 base = planPos(d, joints_.spine[0]);
            const math::Vec3 from = planPos(d, joints_.spine[2]) - base, to = drivenPart(chest, chest.position - base);
            if (math::length(from) > 1e-4f && math::length(to) > 1e-4f)
                delta = math::slerp(math::identityQuat(), math::fromTo(math::normalize(from), math::normalize(to)), kp) * delta;
        }
        // The chain rule (passUp, the foot's rule): the chest turns within
        // its own limits, what it cannot take spine_02, spine_01 and then
        // the pelvis take, the same way about the same axis.
        const int chain[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
        bool pelvisTurned = false;
        passUp(d, chain, 4, math::rotationVector(math::normalize(delta)), s, pelvisTurned);
        for (int j : chain) mark(j, out.active);
        solveLegs(d, t, s, out);  // the legs under the turned pelvis
    }
    levelHead(t, s, out);
    solveArms(d, t, s, out);
    // The far leg (the user, 2026-11-26): a leg the hips' turn let go stands
    // on its spot as far as its hip's range allows; where its foot then is,
    // is its target from the next decide (a driven foot, its IK).
    if (heldLeg_ >= 0) {
        const int i = heldLeg_;
        if (farEnd_ != 2 + i) farSpot_ = anchor_[i];
        trialLeg_ = i;
        trialGoal_ = farSpot_;
        solveLegs(d, t, s, out);
        trialLeg_ = -1;
        farNext_ = 2 + i;
        farTarget_ = planPos(d, joints_.legs[i].end);
        farRot_ = planRot(d, joints_.legs[i].end);
        solveLegs(d, t, s, out);
    }
}

float BodyDecider::stanceNow() const {
    float yaw = 0.0f;
    for (int i = 0; i < 2; ++i)
        yaw += 0.5f * anchorYaw_[i];
    return yaw;
}

void BodyDecider::rangePelvis(const BodyDeciderSettings& s) {
    // A grounded foot holds its leg's turn: the pelvis turns on it only as far
    // as that hip twists (a hip turns out further than in). The left foot's
    // outward turn is the pelvis turning right, the right one's left. Only
    // the leg the body turns toward holds it (its weight): the other one's
    // hip turns out and its foot pivots with it (passUp).
    ranged_ = s;
    if (!limits_ || joints_.legs[0].start < 0) return;
    const ik::JointLimit& hip = limits_->limit(joints_.legs[0].start);  // left: + external
    const float stance = stanceNow();
    float lo = s.pelvisMin.y, hi = s.pelvisMax.y;
    for (int i = 0; i < 2; ++i) {
        if (!grounded_[i] || joints_.legs[i].start < 0) continue;
        const float foot = std::remainder(anchorYaw_[i] - stance, math::kTwoPi);
        if (i == 0) hi = std::min(hi, foot - hip.twistMin);  // turning left, on the left leg
        else lo = std::max(lo, foot + hip.twistMin);        // turning right, on the right leg
    }
    if (lo > hi) lo = hi = 0.5f * (lo + hi);  // feet turned past what the hips span: between
    ranged_.pelvisMin.y = lo;
    ranged_.pelvisMax.y = hi;
}

void BodyDecider::saveChain(Plan& p, const int* chain, ChainCache& c) {
    for (int k = 0; k < c.depth && k < kOrderLinks; ++k) {
        const int j = chain[k];
        if (j < 0) continue;
        if (j == joints_.pelvis) c.pelvis = pelvisOf(p).rotation;
        else c.local[k] = p.pose.locals[static_cast<size_t>(j)].rotation;
    }
    c.held = heldLeg_;
}

bool BodyDecider::loadChain(Plan& p, const int* chain, const ChainCache& c) {
    // Each link its saved number; true if the pelvis turned (legs to solve).
    bool pelvis = false;
    for (int k = 0; k < c.depth && k < kOrderLinks; ++k) {
        const int j = chain[k];
        if (j < 0) continue;
        if (j == joints_.pelvis) {
            placePelvis(p, math::Transform{pelvisOf(p).position, c.pelvis});
            pelvis = true;
        } else {
            p.pose.locals[static_cast<size_t>(j)].rotation = c.local[k];
        }
    }
    refresh(p, joints_.pelvis);
    heldLeg_ = c.held;
    return pelvis;
}

template <class Line>
math::Vec3 BodyDecider::turnChain(Plan& p, const int* chain, int n, float f, Line&& line, const BodyDeciderSettings& s,
                                  bool& pelvisTurned) {
    // The deepest link turns toward the target by share f (only what is
    // needed); the links above it follow its turn in full, each up to its
    // own limits (the user, 2026-11-26: the chain reaches a link only once
    // those above it have turned in full).
    math::Vec3 nearestLeft(0.0f);
    math::Vec3 full(0.0f);
    bool first = true;
    for (int c = n - 1; c >= 0 && f > 0.0f; --c) {
        const int j = chain[c];
        if (j < 0) continue;
        math::Vec3 turn = full;
        if (first) {
            first = false;
            math::Vec3 from(0.0f), to(0.0f);
            line(planPos(p, j), from, to);
            if (math::length(from) < 1e-4f || math::length(to) < 1e-4f) break;
            full = math::rotationVector(math::fromTo(math::normalize(from), math::normalize(to)));
            turn = full * f;
        }
        if (math::length(turn) <= kMinAngle) continue;
        const math::Vec3 rest = passUp(p, &chain[c], 1, turn, s, pelvisTurned);
        if (c == 0) nearestLeft = rest;
    }
    return nearestLeft;
}

template <class Restore, class Line, class Miss>
int BodyDecider::reachChain(Plan& p, const int* chain, int links, Restore&& restore, Line&& line, Miss&& miss,
                            const BodyDeciderSettings& s, bool& pelvisTurned, float& share, math::Vec3* left) {
    // Every depth alike: the chain's last link (the deepest) asks the turn
    // toward the goal (share f of the line's from onto its to, about that
    // link); every link turns that whole turn, the same way, each up to its
    // own limits, and stops there (passUp): a link at its limit does not
    // stop the others. The limb's miss after it.
    bool tried = false;
    math::Vec3 nearestLeft(0.0f);
    auto missAt = [&](int n, float f, bool& turned) {
        restore();
        nearestLeft = turnChain(p, chain, n, f, line, s, turned);
        return miss();
    };
    // As deep as it has to go and as little as does: the shallowest chain
    // with a share that brings the goal into the free room, the least such
    // share (none: the whole chain, its whole turn).
    int depth = links;
    share = 1.0f;
    bool fits = missAt(0, 0.0f, tried) <= kFreeRoomEpsilon;
    if (fits) {
        depth = 0;
        share = 0.0f;
    }
    for (int n = 1; n <= links && !fits; ++n) {
        if (chain[n - 1] < 0) continue;
        for (int i = 1; i <= kFreeRoomSteps && !fits; ++i) {
            if (missAt(n, static_cast<float>(i) / kFreeRoomSteps, tried) > kFreeRoomEpsilon) continue;
            fits = true;
            depth = n;
            float lo = static_cast<float>(i - 1) / kFreeRoomSteps, hi = static_cast<float>(i) / kFreeRoomSteps;
            for (int halving = 0; halving < kFreeRoomSteps; ++halving) {
                const float mid = 0.5f * (lo + hi);
                (missAt(n, mid, tried) <= kFreeRoomEpsilon ? hi : lo) = mid;
            }
            share = hi;
        }
    }
    missAt(depth, share, pelvisTurned);
    if (left) *left = nearestLeft;
    return depth;
}

void BodyDecider::solveLegs(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Grounded feet: each leg reaches its anchor from the hip (two-bone,
    // the knee tracking the foot: its pole turns with the foot from
    // standing), the foot flat on its ground. Lifted feet: the
    // leg reaches its target, clamped to its reach (the knee at limbMinBend: it
    // keeps its bend), the knee forward of the pelvis, the foot turned as
    // given by the blended rotation weight.
    const rig::Pose& rest = rig_->restPose();
    const bool isOutput = &p == &output_;
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints_.legs[i];
        if (leg.start < 0) continue;
        if (isOutput && orderEnd_ == 2 + i && !limbOpen_) {
            // The foot's chain in time: the leg keeps its own turns (riding
            // the pelvis) until the pelvis is chainOnset of its way along
            // (limitOutput).
            refresh(p, leg.start);
            for (int j : {leg.start, leg.mid, leg.end}) markPlanned(j, out);
            continue;
        }
        const math::Quat prevThigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
        const math::Quat prevShin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
        for (int j : {leg.start, leg.mid})
            p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
        refresh(p, leg.start);
        const math::Vec3 hip = planPos(p, leg.start);
        const float reach = limbReach(2 + i, s);
        math::Vec3 target;
        math::Vec3 pole;
        math::Quat footRot;
        const Target& foot = given(t, FootLW + i);
        const bool driven = t.foot[i].mode == TargetMode::Driven;
        const float k = driven ? act_[FootLW + i] * std::clamp(foot.weightPosition, 0.0f, 1.0f) : 0.0f;
        const float kr = driven ? act_[FootLW + i] * std::clamp(foot.weightRotation, 0.0f, 1.0f) : 0.0f;
        // On its anchor (exactly: a pinned foot never slides; the hips'
        // reference keeps the leg within its clamp), or on its way from it
        // (a driven foot leaves the ground from its anchor), clamped.
        target = k > 0.0f ? clampToReach(hip, math::lerp(anchor_[i], foot.position, k), reach) : anchor_[i];
        if (i == trialLeg_) target = clampToReach(hip, trialGoal_, reach);  // the foot chain's try: its B
        if (grounded_[i]) {
            pole = anchorRot_[i] * footToPelvis_[i] * s.kneePole;
            footRot = anchorRot_[i];
        } else {
            // A lifted foot: its knee points where its toes point (a hip turns
            // the whole leg, the knee a hinge over the foot: "knee over toes"),
            // the hip's twist within its range; the foot turned as given by
            // the blended rotation weight. A foot further out than the leg's
            // ranges reach is the chain's (passUp), short of it reported.
            footRot = math::normalize(math::slerp(anchorRot_[i], foot.rotation, kr));
            const math::Quat pelvisRot = planRot(p, joints_.pelvis);
            const math::Quat turned = pelvisRot * math::inverse(restPelvis_.rotation);  // from standing
            const float side = i == 0 ? 1.0f : -1.0f;  // +X is the left leg's outside
            const math::Vec3 toes = math::inverse(turned) * (footRot * (math::inverse(restFoot_[i]) * math::kForward));
            float turn = side * std::atan2(toes.x, toes.z);  // + out (external)
            if (limits_ && joints_.legs[0].start >= 0) {
                const ik::JointLimit& hipLimit = limits_->limit(joints_.legs[0].start);  // left: + external
                turn = std::clamp(turn, hipLimit.twistMin, hipLimit.twistMax);
            }
            pole = math::fromAxisAngle(turned * math::kUp, side * turn) * (pelvisRot * s.kneePole);
        }
        // A foot let go: its pose blends from where it was (the last target,
        // solved) to its anchor as its activation fades.
        math::Quat fadeThigh = math::identityQuat(), fadeShin = math::identityQuat();
        const bool fading = !driven && act_[FootLW + i] > 0.0f;
        if (fading && ik::solveTwoBone(leg, p.models, clampToReach(hip, foot.position, reach),
                                       planRot(p, joints_.pelvis) * s.kneePole, 1.0f, p.pose)) {
            fadeThigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
            fadeShin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
            for (int j : {leg.start, leg.mid})
                p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            refresh(p, leg.start);
        }
        if (ik::solveTwoBone(leg, p.models, target, pole, 1.0f, p.pose)) {
            if (fading) {
                math::Quat& thigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
                math::Quat& shin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
                thigh = math::normalize(math::slerp(thigh, fadeThigh, act_[FootLW + i]));
                shin = math::normalize(math::slerp(shin, fadeShin, act_[FootLW + i]));
            }
            if (isOutput && !grounded_[i]) {
                // A lifted leg's joints turn no faster than a leg does.
                math::Quat& thigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
                math::Quat& shin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
                thigh = limitTurn(prevThigh, thigh, s.legMaxRate, t.dt);
                shin = limitTurn(prevShin, shin, s.legMaxRate, t.dt);
            }
            // A lifted leg within its ranges (short of its target if they
            // stop it). A grounded one keeps its foot pinned (a foot never
            // slides): its hip and knee are the pelvis's and the anchor's,
            // which the hips' bounds keep within reach and the knee's bend.
            const int chain[2] = {leg.start, leg.mid};
            // A leg the hips let go (heldLeg_): within its ranges like a
            // lifted one, so it stands only as far as its hip allows.
            if (grounded_[i] && i != heldLeg_) {
                refresh(p, leg.start);
                if (!isOutput && trialLeg_ < 0 && i != heldLeg_ && t.foot[i].mode == TargetMode::Pinned) {
                    // The chain down to the ground: what the thigh asks past
                    // its range its foot takes, pivoting on the ground (passUp);
                    // past the range itself (onset 1: the hips' turn keeps the
                    // grounded hips within it, legsOver).
                    const math::Vec3 beyond = beyondRange(p, leg.start, s, 1.0f);
                    anchorYaw_[i] += math::dot(beyond, math::kUp);  // about the vertical
                }
            } else {
                clampLimb(p, 2 + i, chain, 2, target);
            }
        }
        if (!isOutput)
            for (int j : {leg.start, leg.mid, leg.end}) {
                if (grounded_[i] && support_[i] > 0.0f) mark(j, out.holding);
                if (driven) mark(j, out.active);
            }
        // The foot: flat on its ground, or as given, within the ankle's range.
        math::Quat footLocal = math::normalize(math::inverse(planRot(p, leg.mid)) * footRot);
        if (limits_) footLocal = ik::clampRotation(footLocal, limits_->limitIn(leg.end, p.pose));
        p.pose.locals[static_cast<size_t>(leg.end)].rotation = footLocal;
        refresh(p, leg.end);
        for (int j : {leg.start, leg.mid, leg.end}) markPlanned(j, out);
    }
    // The foot's chain (the user, 2026-11-25/26), once a tick: the hand's one
    // formula (reachChain), the other way: the leg alone first (its shin and
    // thigh within their ranges, the IK, toward its B); outside its free room
    // the hips, only as far as needed, up to their own range. The pelvis's
    // line: from it to where the leg alone puts the foot, onto the foot's
    // goal. The legs once more toward their targets after it.
    if (!isOutput && !legChained_) {
        legChained_ = true;
        bool turned = false;
        for (int i = 0; i < 2; ++i) {
            const ik::TwoBoneChain& leg = joints_.legs[i];
            // A chain's far end is not a chain's start.
            ChainCache& cache = chainCache_[2 + i];
            if (leg.start < 0 || grounded_[i] || t.foot[i].mode != TargetMode::Driven || farEnd_ == 2 + i) {
                cache.valid = false;
                continue;
            }
            const Target& foot = given(t, FootLW + i);
            // The whole of it (its path's end, B; its given weight).
            const math::Vec3 aim = foot.pathLeft >= 0.0f ? foot.pathEnd : foot.position;
            const bool newAim = math::distance(aim, footAim_[i]) > kReachEpsilon;
            if (newAim) newWay_ = true;
            footAim_[i] = aim;
            const math::Vec3 goal = math::lerp(anchor_[i], aim, std::clamp(foot.weightPosition, 0.0f, 1.0f));
            // The foot's chain is the hips alone (the user, 2026-11-26): the
            // body above rides them, and only the leg and the hips move the
            // foot.
            const int chain[kFootLinks] = {joints_.pelvis};
            math::Transform pelvisSaved = p.pose.locals[static_cast<size_t>(joints_.pelvis)];
            const int heldWas = heldLeg_;
            auto restore = [&] {
                p.pose.locals[static_cast<size_t>(joints_.pelvis)] = pelvisSaved;
                refresh(p, joints_.pelvis);
                heldLeg_ = heldWas;
            };
            // How far the leg alone leaves the foot from its goal.
            auto legMiss = [&] {
                trialLeg_ = i;
                trialGoal_ = goal;
                solveLegs(p, t, s, out);
                trialLeg_ = -1;
                return math::distance(planPos(p, leg.end), goal);
            };
            restore();
            legMiss();
            math::Vec3 alone = planPos(p, leg.end);
            const bool search = newAim || !cache.valid;
            // First the hips' place (the user, 2026-11-26): outside the leg's
            // free room they move the way the foot still has to go, only as
            // far as it needs, while the body's centre of mass stays over the
            // standing foot and the standing leg still reaches its foot; then
            // the hips' turn (reachChain) from there.
            math::Vec3 shift = cache.shift;
            math::Vec3 down = cache.down;
            if (search) {
                const float outsideAtRest = comOutside(p, i, s);
                math::Vec3 base(0.0f), way(0.0f);
                auto moved = [&](float amount, bool& ok) {
                    restore();
                    const math::Transform at = pelvisOf(p);
                    placePelvis(p, math::Transform{at.position + base + way * amount, at.rotation});
                    refresh(p, joints_.pelvis);
                    const float miss = legMiss();
                    const float outside = comOutside(p, i, s);
                    ok = outside >= 0.0f && outside <= std::max(outsideAtRest, 0.0f) + kReachEpsilon &&
                         standingReaches(p, i, s);
                    return miss;
                };
                // The least share of `way` (from `base`) that brings the foot
                // in, or the most the body allows.
                auto leastMove = [&]() {
                    float lo = 0.0f, f = 0.0f;
                    bool allowed = true;
                    if (math::length(way) > kFreeRoomEpsilon && moved(0.0f, allowed) > kFreeRoomEpsilon) {
                        for (int k = 1; k <= kFreeRoomSteps; ++k) {
                            const float hi = static_cast<float>(k) / kFreeRoomSteps;
                            const float miss = moved(hi, allowed);
                            if (!allowed || miss <= kFreeRoomEpsilon) {
                                // Between the last share that was short and this
                                // one: the most the body allows, then within it
                                // the least that brings the foot in.
                                float top = hi;
                                if (!allowed) {
                                    float a = lo, b = hi;
                                    for (int halving = 0; halving < kFreeRoomSteps; ++halving) {
                                        const float mid = 0.5f * (a + b);
                                        moved(mid, allowed);
                                        (allowed ? a : b) = mid;
                                    }
                                    top = a;
                                }
                                f = top;
                                if (top > lo && moved(top, allowed) <= kFreeRoomEpsilon) {
                                    float a = lo, b = top;
                                    for (int halving = 0; halving < kFreeRoomSteps; ++halving) {
                                        const float mid = 0.5f * (a + b);
                                        (moved(mid, allowed) <= kFreeRoomEpsilon ? b : a) = mid;
                                    }
                                    f = b;
                                }
                                break;
                            }
                            lo = f = hi;
                        }
                    }
                    return f;
                };
                // First the way the foot still has to go, as far as the
                // centre of mass allows; then what is still missing of it up
                // or down alone (a drop moves the centre of mass little), as
                // far as the standing leg allows.
                way = goal - alone;
                shift = way * leastMove();
                bool ok = true;
                base = shift;
                way = math::Vec3(0.0f);
                moved(0.0f, ok);
                // Up or down the way the foot is still off, as far as the
                // standing leg can take the hips (its reach: the search finds
                // the least of it that brings the foot in).
                const float off = (goal - planPos(p, leg.end)).y;
                way = std::abs(off) > kFreeRoomEpsilon
                          ? math::Vec3(0.0f, std::copysign(limbReach(2 + (1 - i), s), off), 0.0f)
                          : math::Vec3(0.0f);
                down = way * leastMove();
                shift += down;
                cache.shift = shift;
                cache.down = down;
            }
            restore();
            if (math::length(shift) > kReachEpsilon) {
                const math::Transform at = pelvisOf(p);
                placePelvis(p, math::Transform{at.position + shift, at.rotation});
                refresh(p, joints_.pelvis);
                pelvisSaved = p.pose.locals[static_cast<size_t>(joints_.pelvis)];
                legMiss();
                alone = planPos(p, leg.end);
            }
            // The one formula (reachChain), the hand's, the other way.
            float share = 1.0f;
            auto line = [&](math::Vec3 about, math::Vec3& from, math::Vec3& to) {
                from = alone - about;
                to = goal - about;
            };
            // The search only for a new B or a changed chain; else the same
            // numbers once more.
            int depth = 0;
            if (search) {
                depth = reachChain(p, chain, kFootLinks, restore, line, legMiss, s, turned, share);
                cache.valid = true;
                cache.depth = depth;
                cache.share = share;
                saveChain(p, chain, cache);
            } else {
                depth = cache.depth;
                share = cache.share;
                if (depth > 0 && share > 0.0f && loadChain(p, chain, cache)) turned = true;
                heldLeg_ = heldWas;
            }
            const bool moves = math::length(shift) > kReachEpsilon;
            if (share <= 0.0f || depth <= 0) depth = 0;
            if (depth <= 0 && !moves) continue;
            for (int c = 0; c < depth; ++c) mark(chain[c], out.active);
            if (moves) mark(joints_.pelvis, out.active);
            // In time: from the link the calculation reached down to the
            // pelvis (its move first, then its turn), the leg last
            // (limitOutput).
            if (orderEnd_ < 0) {
                orderEnd_ = 2 + i;
                orderCount_ = 0;
                for (int c = depth - 1; c >= 0; --c)
                    if (chain[c] >= 0) order_[orderCount_++] = chain[c];
                if (moves && !inOrder(joints_.pelvis)) order_[orderCount_++] = joints_.pelvis;
                hipsDown_ = down;
            }
        }
        (void)turned;  // the legs are solved again either way
        solveLegs(p, t, s, out);
    }
}

void BodyDecider::solveArms(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Driven hands: shoulder and elbow from the shoulder, the reach clamped
    // (the elbow keeps its bend). The elbow's side (the solve's pole): the
    // arm turns from hanging relaxed off the chest to the target by the
    // shortest swing, adding no twist, and the elbow's bend turns with it
    // (no fixed direction, so none the arm can line up with); out of the
    // shoulder's range, the elbow goes round as little as brings it in
    // (fitPole). A free hand stays where it is.
    const rig::Pose& rest = rig_->restPose();
    const bool isOutput = &p == &output_;
    bool pelvisTurned = false;
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (arm.start < 0) continue;
        const math::Quat prevUpper = p.pose.locals[static_cast<size_t>(arm.start)].rotation;
        const math::Quat prevFore = p.pose.locals[static_cast<size_t>(arm.mid)].rotation;
        const bool driven = t.hand[h].mode == TargetMode::Driven;
        if (!driven) {
            if (isOutput) {
                // The free arm follows the desired pose (it stays where it
                // is), no faster than a limb turns.
                for (int j : {joints_.clavicles[h], arm.start, arm.mid, arm.end}) {
                    if (j < 0) continue;
                    if (inOrder(j)) continue;  // the foot's chain moved it already (limitOutput)
                    math::Quat& o = p.pose.locals[static_cast<size_t>(j)].rotation;
                    o = limitTurn(o, desired_.pose.locals[static_cast<size_t>(j)].rotation, s.limbMaxRate, t.dt);
                    clampJoint(p, j);
                }
                refresh(p, joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : arm.start);
                rangeMiss_[h] = 0.0f;
            }
            continue;
        }
        if (isOutput && orderEnd_ == h && !limbOpen_) {
            // The hand's chain in time: the arm keeps its own turns (riding
            // the trunk) until its clavicle is chainOnset of its way along
            // (limitOutput).
            rangeMiss_[h] = 0.0f;
            for (int j : {arm.start, arm.mid, arm.end}) markPlanned(j, out);
            continue;
        }
        if (!isOutput)
            for (int j : {arm.start, arm.mid, arm.end}) mark(j, out.active);
        const Target& hand = given(t, h);
        // Toward the target by its blended weight.
        const float k = act_[h] * std::clamp(hand.weightPosition, 0.0f, 1.0f);
        const float reach = limbReach(h, s);
        auto armTarget = [&](math::Vec3 goal, float k) {
            const math::Quat chest = joints_.spine[2] >= 0 ? planRot(p, joints_.spine[2]) : math::identityQuat();
            const math::Vec3 chestPos = joints_.spine[2] >= 0 ? planPos(p, joints_.spine[2]) : math::Vec3(0.0f);
            const math::Vec3 restWorld = chestPos + chest * restHandChest_[h];
            return clampToReach(planPos(p, arm.start), math::lerp(restWorld, goal, k), reach);
        };
        auto armAtRest = [&] {
            for (int j : {arm.start, arm.mid})
                p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            refresh(p, arm.start);
        };
        armAtRest();
        if (!isOutput && !(driven && limits_ && farEnd_ != h)) chainCache_[h].valid = false;
        if (!isOutput && driven && limits_ && farEnd_ != h) {  // a chain's far end is not a chain's start
            // The chain rule (body_chain.h) from the upper arm up: the arm
            // to the target, its elbow placed as the solve below does; what the shoulder asks
            // past chainOnset of its range on each of its range axes the
            // clavicle, the chest, spine_02, spine_01 and the pelvis take in
            // turn, the same way about the same axis, each its own part
            // within its own limits (passUp). Worked out up front for the
            // whole move: from where the hand's path ends (B, the sync
            // formula), so the trunk can go first (limitOutput).
            // The whole of it (its given weight, not the blend's ramp in).
            const math::Vec3 aim = hand.pathLeft >= 0.0f ? hand.pathEnd : hand.position;
            // A new B (a new target, e.g. each arrow press): the chain goes
            // from the hips again (limitOutput).
            const bool newAim = math::distance(aim, chainAim_[h]) > kReachEpsilon;
            if (newAim) newWay_ = true;
            chainAim_[h] = aim;
            const float kB = std::clamp(hand.weightPosition, 0.0f, 1.0f);
            // One formula (the user's design, 2026-11-24). The arm's free
            // room: where the arm alone, within its range and length, puts
            // the hand. A target inside it: the arm alone. Outside it: the
            // chain goes as deep as it has to (clavicle, chest, spine_02,
            // spine_01, the pelvis), its deepest link turns toward the
            // target and each next one follows the one before it; the arm
            // reaches last. The chain stops at the hips; below them the legs
            // are the IK's (the user, 2026-11-26). A leg the hips' turn takes
            // past its hip's range lets go and goes with them by what its hip
            // cannot take (passUp, the far end).
            constexpr int kLinks = 5;
            const int chain[kLinks] = {joints_.clavicles[h], joints_.spine[2], joints_.spine[1], joints_.spine[0],
                                       joints_.pelvis};
            constexpr int kKept = 7;
            math::Quat saved[kKept];
            const math::Transform pelvisSaved = p.pose.locals[static_cast<size_t>(joints_.pelvis)];
            const int keep[kKept] = {chain[0], chain[1], chain[2], chain[3], arm.start, arm.mid, arm.end};
            for (int i = 0; i < kKept; ++i)
                if (keep[i] >= 0) saved[i] = p.pose.locals[static_cast<size_t>(keep[i])].rotation;
            const int heldWas = heldLeg_;
            auto restore = [&] {
                for (int i = 0; i < kKept; ++i)
                    if (keep[i] >= 0) p.pose.locals[static_cast<size_t>(keep[i])].rotation = saved[i];
                p.pose.locals[static_cast<size_t>(joints_.pelvis)] = pelvisSaved;
                refresh(p, joints_.pelvis);
                heldLeg_ = heldWas;
            };
            auto goalNow = [&] {
                const math::Quat chest = joints_.spine[2] >= 0 ? planRot(p, joints_.spine[2]) : math::identityQuat();
                const math::Vec3 chestPos = joints_.spine[2] >= 0 ? planPos(p, joints_.spine[2]) : math::Vec3(0.0f);
                return math::lerp(chestPos + chest * restHandChest_[h], aim, kB);
            };
            // How far the arm alone leaves the hand from its goal (0: inside
            // its free room).
            auto armMiss = [&] {
                armAtRest();
                const math::Vec3 goal = goalNow(), shoulder = planPos(p, arm.start);
                const math::Vec3 at = clampToReach(shoulder, goal, reach);
                float unused = 0.0f;
                const math::Vec3 armPole = fitPole(armShape(h, p, s), armHang(h, p), armFrame(h, p), shoulder, at, unused);
                if (!ik::solveTwoBone(arm, p.models, at, armPole, 1.0f, p.pose)) return math::distance(shoulder, goal);
                clampJoint(p, arm.start);
                clampJoint(p, arm.mid);
                refresh(p, arm.start);
                return math::distance(planPos(p, arm.end), goal);
            };
            // The one formula (reachChain): the deepest link's line is from it
            // to the shoulder, onto the target.
            float share = 1.0f;
            auto line = [&](math::Vec3 about, math::Vec3& from, math::Vec3& to) {
                from = planPos(p, arm.start) - about;
                to = goalNow() - about;
            };
            // The search only for a new B or a changed chain; else the same
            // numbers once more.
            ChainCache& cache = chainCache_[h];
            int depth = 0;
            if (newAim || !cache.valid) {
                depth = reachChain(p, chain, kLinks, restore, line, armMiss, s, pelvisTurned, share);
                cache.valid = true;
                cache.depth = depth;
                cache.share = share;
                saveChain(p, chain, cache);
            } else {
                depth = cache.depth;
                share = cache.share;
                restore();
                if (depth > 0 && share > 0.0f && loadChain(p, chain, cache)) pelvisTurned = true;
            }
            if (share > 0.0f)
                for (int c = 0; c < depth; ++c) mark(chain[c], out.active);
            // In time: from the link the calculation reached (down to the
            // hips) up, the arm last (limitOutput); a hand's chain leads
            // before a foot's.
            if (share > 0.0f && depth > 0 && (orderEnd_ < 0 || orderEnd_ >= 2)) {
                orderEnd_ = h;
                orderCount_ = 0;
                for (int c = depth - 1; c >= 0; --c)
                    if (chain[c] >= 0) order_[orderCount_++] = chain[c];
            }
            armAtRest();
        }
        const math::Vec3 from = planPos(p, arm.start);
        const math::Vec3 target = armTarget(hand.position, k);
        const ArmHang hang = armHang(h, p);
        math::Vec3 pole = relaxedPole(hang, math::safeNormalize(target - from, hang.down));
        float miss = 0.0f;
        if (limits_) pole = fitPole(armShape(h, p, s), hang, armFrame(h, p), from, target, miss);
        if (ik::solveTwoBone(arm, p.models, target, pole, 1.0f, p.pose)) {
            math::Quat& upper = p.pose.locals[static_cast<size_t>(arm.start)].rotation;
            math::Quat& fore = p.pose.locals[static_cast<size_t>(arm.mid)].rotation;
            if (isOutput) {
                upper = limitTurn(prevUpper, upper, s.limbMaxRate, t.dt);
                fore = limitTurn(prevFore, fore, s.limbMaxRate, t.dt);
            }
            const int chain[2] = {arm.start, arm.mid};
            clampLimb(p, h, chain, 2, target);
            // The hand's roll (about the forearm): the forearm twists its own
            // part within its range (chainOwnPart); the rest turns the elbow
            // round the shoulder-hand line (the shoulder's turn), the same
            // way, and the arm is solved again (the chain rule, as passUp).
            const float krr = act_[h] * std::clamp(hand.weightRotation, 0.0f, 1.0f);
            if (limits_ && arm.end >= 0 && krr > 0.0f) {
                const math::Quat restHandNow = planRot(p, arm.mid) * rest.locals[static_cast<size_t>(arm.end)].rotation;
                const math::Quat wantHand = math::normalize(math::slerp(restHandNow, hand.rotation, krr));
                const math::Vec3 along = math::safeNormalize(planPos(p, arm.end) - planPos(p, arm.mid), hang.down);
                const math::Quat rel = math::normalize(wantHand * math::inverse(planRot(p, arm.end)));
                const float roll = std::remainder(2.0f * std::atan2(math::dot(math::Vec3(rel.x, rel.y, rel.z), along), rel.w),
                                                  math::kTwoPi);
                const ik::JointLimit foreLimit = limits_->limitIn(arm.mid, p.pose);
                const ik::LimitFrame ff = ik::limitFrame(foreLimit);
                const float now = ik::limitAngles(fore, foreLimit).x;
                const float total = now + roll;
                const float bound = total >= 0.0f ? ff.max.x : -ff.min.x;
                float part = roll;
                if (bound > 0.0f)
                    part = std::copysign(chainOwnPart(std::abs(total), bound, s.chainOnset), total) - now;
                const float over = roll - part;
                if (std::abs(over) > kMinAngle) {
                    pole = math::fromAxisAngle(math::safeNormalize(target - from, hang.down), over) * pole;
                    armAtRest();
                    if (ik::solveTwoBone(arm, p.models, target, pole, 1.0f, p.pose)) {
                        if (isOutput) {
                            upper = limitTurn(prevUpper, upper, s.limbMaxRate, t.dt);
                            fore = limitTurn(prevFore, fore, s.limbMaxRate, t.dt);
                        }
                        clampLimb(p, h, chain, 2, target);
                    }
                }
                if (std::abs(part) > kMinAngle) {
                    const math::Vec3 alongNow = math::safeNormalize(planPos(p, arm.end) - planPos(p, arm.mid), hang.down);
                    turnPlanned(p, arm.mid, math::fromAxisAngle(alongNow, part));
                }
            }
        }
        // The hand's rotation, as given, within the wrist's range.
        if (arm.end >= 0) {
            const float kr = act_[h] * std::clamp(hand.weightRotation, 0.0f, 1.0f);
            const math::Quat restHand = planRot(p, arm.mid) * rest.locals[static_cast<size_t>(arm.end)].rotation;
            const math::Quat want = math::normalize(math::slerp(restHand, hand.rotation, kr));
            math::Quat handLocal = math::normalize(math::inverse(planRot(p, arm.mid)) * want);
            if (limits_) handLocal = ik::clampRotation(handLocal, limits_->limitIn(arm.end, p.pose));
            p.pose.locals[static_cast<size_t>(arm.end)].rotation = handLocal;
            refresh(p, arm.end);
        }
        for (int j : {arm.start, arm.mid, arm.end}) markPlanned(j, out);
    }
    if (pelvisTurned) solveLegs(p, t, s, out);
}

math::Vec3 BodyDecider::beyondRange(const Plan& p, int j, const BodyDeciderSettings& s, float onset) const {
    // What joint j asks past `onset` of its range on each of its range
    // axes (before its clamp), as one rotation (world rotation vector).
    (void)s;
    math::Vec3 beyond(0.0f);
    if (!limits_ || j < 0) return beyond;
    const ik::JointLimit lim = limits_->limitIn(j, p.pose);
    const ik::LimitFrame f = ik::limitFrame(lim);
    const math::Vec3 need = ik::limitAngles(p.pose.locals[static_cast<size_t>(j)].rotation, lim);
    const math::Quat parent = planRot(p, rig_->parent(j));
    const math::Vec3 axes[3] = {f.x, f.y, f.z};
    for (int a = 0; a < 3; ++a) {
        const float bound = need[a] >= 0.0f ? f.max[a] : -f.min[a];
        if (bound <= 0.0f) continue;
        const float over = std::abs(need[a]) - chainOwnPart(std::abs(need[a]), bound, onset);
        beyond += math::normalize(parent * axes[a]) * std::copysign(over, need[a]);
    }
    return beyond;
}

math::Vec3 BodyDecider::passUp(Plan& p, const int* chain, int count, math::Vec3 rest, const BodyDeciderSettings& s,
                               bool& pelvisTurned) {
    // The chain rule up any chain: each joint takes its own
    // part of what comes to it on each of its range axes (chainOwnPart,
    // within its own limits), turning the same way about the same axis, and
    // hands the rest on.
    const float onset = s.chainOnset;
    for (int k = 0; k < count && math::length(rest) > kMinAngle; ++k) {
        const int j = chain[k];
        if (j < 0) continue;
        // The legs that hold the hips: standing and not let go (heldLeg_).
        // None: the hips' range from standing (pelvisMin / pelvisMax).
        bool legsHold = false;
        for (int f = 0; f < 2; ++f) legsHold = legsHold || (legStands(f) && f != heldLeg_);
        // The joint's angles now and its range, on its range axes (world).
        math::Vec3 axis[3], now, lo, hi;
        if (j == joints_.pelvis) {
            // Its range comes from the legs (the user, 2026-11-24): on the
            // ground, as far as the grounded hips allow (legsOver, below);
            // in the air, its range from standing (pelvisMin / pelvisMax,
            // character axes). Driven hips are not the chain's to turn.
            if (act_[HipsW] >= 1.0f) break;
            const math::Transform pel = pelvisOf(p);
            for (int a = 0; a < 3; ++a) axis[a] = math::Vec3(a == 0, a == 1, a == 2);
            now = math::rotationVector(math::normalize(pel.rotation * math::inverse(pelvisBase())));
            lo = legsHold ? math::Vec3(-math::kPi) : s.pelvisMin;
            hi = legsHold ? math::Vec3(math::kPi) : s.pelvisMax;
        } else {
            if (!limits_) break;
            const ik::JointLimit lim = limits_->limitIn(j, p.pose);
            const ik::LimitFrame f = ik::limitFrame(lim);
            const math::Quat parent = planRot(p, rig_->parent(j));
            axis[0] = math::normalize(parent * f.x);
            axis[1] = math::normalize(parent * f.y);
            axis[2] = math::normalize(parent * f.z);
            now = ik::limitAngles(p.pose.locals[static_cast<size_t>(j)].rotation, lim);
            lo = f.min;
            hi = f.max;
        }
        // Each axis: the joint keeps its own part of all it is asked (what it
        // has plus the rest that comes to it), never more than comes.
        math::Vec3 take(0.0f);
        for (int a = 0; a < 3; ++a) {
            const float comes = math::dot(rest, axis[a]);
            const float total = now[a] + comes;
            const float bound = total >= 0.0f ? hi[a] : -lo[a];
            if (bound <= 0.0f || std::abs(comes) <= 1e-6f) continue;
            const float own = std::copysign(chainOwnPart(std::abs(total), bound, onset), total) - now[a];
            const float part = comes >= 0.0f ? std::clamp(own, 0.0f, comes) : std::clamp(own, comes, 0.0f);
            take += axis[a] * part;
        }
        const float angle = math::length(take);
        if (angle <= kMinAngle) continue;
        const math::Quat turn = math::fromAxisAngle(take / angle, angle);
        if (j == joints_.pelvis) {
            // The hips turn in place, only as far as the holding legs' hips
            // stay within their ranges (the other hip follows up to its own
            // limit), the most of the turn that does (halving).
            const math::Transform before = pelvisOf(p);
            auto place = [&](float f) {
                const math::Quat q = math::fromAxisAngle(take / angle, angle * f);
                math::Transform pel = before;
                pel.rotation = math::normalize(q * before.rotation);
                placePelvis(p, pel);
            };
            float share = 1.0f;
            if (legsHold) {
                float legBefore[2] = {0.0f, 0.0f};
                for (int f = 0; f < 2; ++f) legBefore[f] = legOver(p, s, f);
                place(1.0f);
                // A standing leg this turn takes past its hip's range (on any
                // axis: the turn's direction decides, no side is fixed) lets
                // go: it goes with the hips by what its hip cannot take
                // (solveLegs, the far end). Never the last standing leg nor a
                // foot the planner drives; both past: the one further past.
                if (heldLeg_ < 0 && legStands(0) && legStands(1)) {
                    float past[2] = {-1.0f, -1.0f};
                    for (int f = 0; f < 2; ++f)
                        if (!footGiven_[f]) past[f] = legOver(p, s, f) - legBefore[f];
                    const int go = past[0] > past[1] ? 0 : 1;
                    if (past[go] > kMinAngle) heldLeg_ = go;
                }
                // The hips' bound (the user, 2026-11-26): the legs' hips while
                // both stand and hold; one let go or lifted (a foot's case
                // too), the hips' own range (pelvisMin / pelvisMax) and they
                // stop there (the standing foot pivots on the ground past
                // its hip's range, solveLegs).
                auto ownOver = [&](math::Quat r) {
                    const math::Vec3 v = math::rotationVector(math::normalize(r * math::inverse(pelvisBase())));
                    float o = 0.0f;
                    for (int a = 0; a < 3; ++a)
                        o += std::max(v[a] - s.pelvisMax[a], 0.0f) + std::max(s.pelvisMin[a] - v[a], 0.0f);
                    return o;
                };
                const bool own = heldLeg_ >= 0 || !(legStands(0) && legStands(1));
                float overBefore = 0.0f;
                if (own) overBefore = ownOver(before.rotation);
                else
                    for (int f = 0; f < 2; ++f) overBefore += legBefore[f];
                auto overNow = [&] { return own ? ownOver(pelvisOf(p).rotation) : legsOver(p, s); };
                if (overNow() > overBefore + kMinAngle) {
                    float held = 0.0f, past = 1.0f;
                    for (int i = 0; i < kFreeRoomSteps; ++i) {
                        const float mid = 0.5f * (held + past);
                        place(mid);
                        (overNow() > overBefore + kMinAngle ? past : held) = mid;
                    }
                    share = held;
                }
            }
            place(share);
            take *= share;
            if (share > 0.0f) pelvisTurned = true;
        } else {
            turnPlanned(p, j, turn);
        }
        rest -= take;
    }
    return rest;
}

bool BodyDecider::legStands(int i) const {
    // On the ground, or the chain's far leg (it stands on its spot for the
    // chain's sums; it is up only by what its hip cannot take).
    return joints_.legs[i].start >= 0 && (grounded_[i] || farEnd_ == 2 + i);
}

math::Vec3 BodyDecider::legSpot(int i) const { return farEnd_ == 2 + i ? farSpot_ : anchor_[i]; }

float BodyDecider::legOver(Plan& p, const BodyDeciderSettings& s, int i) {
    // How far leg i's hip is past its range (rad, summed over its range
    // axes), the leg reaching its spot under the plan's pelvis; the leg's
    // pose is left as it was.
    float over = 0.0f;
    if (!limits_ || !legStands(i)) return over;
    const rig::Pose& rest = rig_->restPose();
    const ik::TwoBoneChain& leg = joints_.legs[i];
    math::Quat& thigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
    math::Quat& shin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
    const math::Quat thighWas = thigh, shinWas = shin;
    thigh = rest.locals[static_cast<size_t>(leg.start)].rotation;
    shin = rest.locals[static_cast<size_t>(leg.mid)].rotation;
    refresh(p, leg.start);
    // As solveLegs reaches a grounded foot: the knee tracking the foot.
    const math::Vec3 pole = anchorRot_[i] * footToPelvis_[i] * s.kneePole;
    if (ik::solveTwoBone(leg, p.models, legSpot(i), pole, 1.0f, p.pose)) {
        const ik::JointLimit lim = limits_->limitIn(leg.start, p.pose);
        const ik::LimitFrame f = ik::limitFrame(lim);
        const math::Vec3 a = ik::limitAngles(thigh, lim);
        for (int k = 0; k < 3; ++k) over += std::max(a[k] - f.max[k], 0.0f) + std::max(f.min[k] - a[k], 0.0f);
    }
    thigh = thighWas;
    shin = shinWas;
    refresh(p, leg.start);
    return over;
}

float BodyDecider::legsOver(Plan& p, const BodyDeciderSettings& s) {
    float over = 0.0f;
    for (int i = 0; i < 2; ++i)
        if (i != heldLeg_) over += legOver(p, s, i);
    return over;
}

float BodyDecider::comOutside(const Plan& p, int i, const BodyDeciderSettings& s) const {
    // The other foot is the one the body stands on: its outline (the foot's
    // size, its centre ahead of the ankle, turned as it stands), less the
    // margin; how far the plan's centre of mass is outside it, level.
    const int k = 1 - i;
    if (joints_.legs[k].start < 0 || !grounded_[k]) return -1.0f;
    const float yaw = anchorYaw_[k];
    const math::Vec3 ahead(std::sin(yaw), 0.0f, std::cos(yaw)), side(std::cos(yaw), 0.0f, -std::sin(yaw));
    const math::Vec3 d = centreOfMass(p) - (anchor_[k] + ahead * s.footCentreAhead);
    const float along = std::max(std::abs(math::dot(d, ahead)) - std::max(s.footHalfLength - s.comMargin, 0.0f), 0.0f);
    const float across = std::max(std::abs(math::dot(d, side)) - std::max(s.footHalfWidth - s.comMargin, 0.0f), 0.0f);
    return std::sqrt(along * along + across * across);
}

bool BodyDecider::standingReaches(const Plan& p, int i, const BodyDeciderSettings& s) const {
    // The standing leg (the other one) still reaches its foot from the hips:
    // no further than its reach, no nearer than its knee bent oneLegKneeMax.
    const int k = 1 - i;
    const ik::TwoBoneChain& leg = joints_.legs[k];
    if (leg.start < 0) return false;
    const float d = math::distance(planPos(p, leg.start), anchor_[k]);
    return d <= limbReach(2 + k, s) + kReachEpsilon && d >= bentSpan(thighLength_[k], shinLength_[k], s.oneLegKneeMax);
}

bool BodyDecider::inOrder(int j) const {
    if (j < 0) return false;
    for (int k = 0; k < orderCount_; ++k)
        if (order_[k] == j) return true;
    return false;
}

void BodyDecider::limitOutput(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // The output pose follows the desired one with bounded motion: the hips
    // by speed and acceleration (a stopping law: toward the target at most
    // sqrt(2 a d), so they arrive without overshoot), the hips' turn, the
    // trunk's and the head's joints by rate. Commanded motion is bounded by
    // muscle activation, so no target change can make the plan jump.
    const float dt = std::max(t.dt, 0.0f);
    const math::Transform want = pelvisOf(desired_);
    math::Transform have = pelvisOf(output_);
    const math::Vec3 wantPosition = want.position;
    const float onset = std::clamp(s.chainOnset, 0.0f, 1.0f);
    // A new target or another chain leading: every link's way starts here.
    if (newWay_ || orderEnd_ != orderEndWas_) {
        for (float& c : orderTravel_) c = 0.0f;
        moveTravel_[0] = moveTravel_[1] = 0.0f;
        downOpen_ = false;
    }
    newWay_ = false;
    orderEndWas_ = orderEnd_;
    auto link = [&](float& travel, float moved, float left, float least = kMinAngle) {
        travel = moved > least ? travel + moved : 0.0f;
        return left <= least || left <= (1.0f - onset) * (travel + left);
    };
    // A foot's chain moves the hips in two parts (solveLegs): toward the
    // target first, their drop (hipsDown_) once that is chainOnset along.
    if (math::length(hipsDown_) <= kReachEpsilon) downOpen_ = true;
    const math::Vec3 aimPosition = downOpen_ ? wantPosition : wantPosition - hipsDown_;
    if (dt > 0.0f) {
        const math::Vec3 d = aimPosition - have.position;
        const float dist = math::length(d);
        // Never faster than lands on the target this tick: the continuous
        // law alone overshoots within a dt^2 of it and the standing plan
        // chattered (pelvis +-0.17 mm in a 6-tick cycle, knee 4.5..5.5 deg).
        const float speed = std::min({s.hipsMaxSpeed, std::sqrt(2.0f * std::max(s.hipsMaxAccel, 0.0f) * dist), dist / dt});
        const math::Vec3 vWant = dist > 1e-6f ? d * (speed / dist) : math::Vec3(0.0f);
        hipsVel_ += clampLength(vWant - hipsVel_, std::max(s.hipsMaxAccel, 0.0f) * dt);
        have.position += hipsVel_ * dt;
    }
    if (!downOpen_)
        downOpen_ = link(moveTravel_[0], math::length(hipsVel_) * dt, math::distance(have.position, aimPosition), kReachEpsilon);
    // The chain in time (the user, 2026-11-25), one rule for the hand's and
    // the foot's: from the link the calculation reached (order_, deepest
    // first: the hand's from the opposite shin up, the foot's from the
    // crossed hand down), each toward the desired pose by rate, each starting
    // only once the one before it is chainOnset of its way along (or has
    // none to go); the limb last (limbOpen_: solveArms, solveLegs). What
    // each link turns is worked out up front (the desired pose).
    auto rateOf = [&](int j) {
        for (int e = 0; e < 2; ++e) {
            if (j == joints_.arms[e].start || j == joints_.arms[e].mid || j == joints_.arms[e].end) return s.limbMaxRate;
            if (j == joints_.legs[e].start || j == joints_.legs[e].mid || j == joints_.legs[e].end) return s.legMaxRate;
        }
        return s.trunkMaxRate;
    };
    auto turnPelvis = [&] {
        if (dt > 0.0f) have.rotation = limitTurn(have.rotation, want.rotation, s.hipsMaxTurnRate, dt);
    };
    if (dt <= 0.0f) {
        have = math::Transform{wantPosition, want.rotation};
        hipsVel_ = math::Vec3(0.0f);
    }
    bool open = true;
    for (int k = 0; k < orderCount_; ++k) {
        const int j = order_[k];
        if (j == joints_.pelvis) {
            // The hips' move first (it goes on above: toward the target, then
            // down), their turn once the whole move is chainOnset of its way
            // along (the user, 2026-11-26: first the place, then the turn).
            open = open && downOpen_ &&
                   link(moveTravel_[1], math::length(hipsVel_) * dt, math::distance(have.position, wantPosition),
                        kReachEpsilon);
            const math::Quat was = have.rotation;
            if (open) turnPelvis();
            open = open && link(orderTravel_[k], math::angleBetween(was, have.rotation),
                                math::angleBetween(have.rotation, want.rotation));
            continue;
        }
        const math::Quat was = output_.pose.locals[static_cast<size_t>(j)].rotation;
        if (open) limitJoint(j, rateOf(j), dt);
        const math::Quat now = output_.pose.locals[static_cast<size_t>(j)].rotation;
        open = open && link(orderTravel_[k], math::angleBetween(was, now),
                            math::angleBetween(now, desired_.pose.locals[static_cast<size_t>(j)].rotation));
    }
    limbOpen_ = open;
    // The links no chain leads follow freely.
    if (!inOrder(joints_.pelvis)) turnPelvis();
    // The height under every grounded leg's reach is the desired pose's
    // (solveLegs), so the hips get there within the legs' bounds.
    placePelvis(output_, have);
    for (int j : {joints_.spine[0], joints_.spine[1], joints_.spine[2], joints_.clavicles[0], joints_.clavicles[1]})
        if (j >= 0 && !inOrder(j)) limitJoint(j, s.trunkMaxRate, dt);
    for (int j : {joints_.neck, joints_.head}) limitJoint(j, s.headMaxRate, dt);
    refresh(output_, joints_.pelvis);
    // The legs on the output pelvis (grounded feet exactly on their anchors),
    // then the arms on the output chest.
    solveLegs(output_, t, s, out);
    solveArms(output_, t, s, out);
    refresh(output_);
}

void BodyDecider::report(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    for (int e = 0; e < kEnds; ++e) {
        ReachReport& r = out.reach[e];
        const int root = endRoot(e);
        const int end = e < 2 ? joints_.arms[e].end : joints_.legs[e - 2].end;
        if (root < 0 || end < 0) continue;
        math::Vec3 target;
        if (e < 2) {
            r.driven = t.hand[e].mode == TargetMode::Driven;
            if (!r.driven) continue;
            target = t.hand[e].position;
        } else {
            const int i = e - 2;
            r.driven = t.foot[i].mode == TargetMode::Driven && !grounded_[i];
            target = r.driven ? t.foot[i].position : anchor_[i];
        }
        const math::Vec3 from = planPos(output_, root);
        const float reach = limbReach(e, s);
        r.shortfall = std::max({0.0f, math::distance(from, target) - reach, rangeMiss_[e]});
        r.reached = r.shortfall <= 1e-3f;
        r.direction = math::safeNormalize(target - planPos(output_, end));
    }
}

// ---- the balance step (the capture point) ----------------------------------------------

// ---- reach queues (the desired pose) ----------------------------------------------------

ArmShape BodyDecider::armShape(int h, const Plan& p, const BodyDeciderSettings& s) const {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    ArmShape a;
    a.upper = upperArmLength_[h];
    a.fore = forearmLength_[h];
    a.reach = limbReach(h, s);
    if (limits_) {
        a.least = bentSpan(a.upper, a.fore, std::max(0.0f, -limits_->limit(arm.mid).swing1Min));
        a.shoulder = limits_->limitIn(arm.start, p.pose);
    }
    return a;
}

ArmHang BodyDecider::armHang(int h, const Plan& p) const {
    const math::Quat chest = joints_.spine[2] >= 0 ? planRot(p, joints_.spine[2]) : math::identityQuat();
    return ArmHang{chest * hangDir_[h], chest * hangBend_[h]};
}

math::Quat BodyDecider::armFrame(int h, const Plan& p) const {
    const int parent = rig_->parent(joints_.arms[h].start);
    return parent >= 0 ? planRot(p, parent) : math::identityQuat();
}

int BodyDecider::endRoot(int e) const { return e < 2 ? joints_.arms[e].start : joints_.legs[e - 2].start; }

float BodyDecider::limbReach(int e, const BodyDeciderSettings& s) const {
    const float bend = std::clamp(s.limbMinBend, 0.0f, 1.5f);
    return e < 2 ? bentSpan(upperArmLength_[e], forearmLength_[e], bend)
                 : bentSpan(thighLength_[e - 2], shinLength_[e - 2], bend);
}

// ---- posture (the desired pose) ------------------------------------------------------------

void BodyDecider::levelHead(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // A driven head: by its blended weight, the head first, the rest down
    // the chain (passUp): neck, chest, spine_02, spine_01, the pelvis.
    // Without a target the head rides the neck.
    Plan& d = desired_;
    if (joints_.head < 0 || act_[HeadW] <= 0.0f) return;
    const Target& head = given(t, HeadW);
    const float k = act_[HeadW] * std::clamp(head.weightRotation, 0.0f, 1.0f);
    const math::Quat want = math::slerp(planRot(d, joints_.head), head.rotation, k);
    const int chain[6] = {joints_.head, joints_.neck, joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
    bool pelvisTurned = false;
    passUp(d, chain, 6, math::rotationVector(math::normalize(want * math::inverse(planRot(d, joints_.head)))), s,
           pelvisTurned);
    for (int j : chain) mark(j, out.active);
    if (pelvisTurned) solveLegs(d, t, s, out);
    markPlanned(joints_.head, out);
}

} // namespace motion
