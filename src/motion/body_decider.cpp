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
// A Driven foot this close (m) above where it was pinned still stands on the
// ground (it has not lifted yet: the weight shift before a step).
constexpr float kOnGround = 0.02f;

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

math::Vec3 level(math::Vec3 v) { return math::Vec3(v.x, 0.0f, v.z); }

math::Vec3 clampLength(math::Vec3 v, float max) {
    const float len = math::length(v);
    return len > max && len > 0.0f ? v * (max / len) : v;
}

// A two-bone limb's span (root to end) with its middle joint bent `bend`.
float bentSpan(float a, float b, float bend) {
    return std::sqrt(std::max(a * a + b * b + 2.0f * a * b * std::cos(bend), 0.0f));
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
    lastCentre_.assign(n, math::Vec3(0.0f));
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
            legLength_[i] = thighLength_[i] + shinLength_[i];
            footToPelvis_[i] = math::inverse(planRot(r, leg.end)) * restPelvis_.rotation;
            restFoot_[i] = planRot(r, leg.end);
            restFootY_[i] = planPos(r, leg.end).y;
            hipOffset_[i] = pelvisInv * (planPos(r, leg.start) - restPelvis_.position);
            anchor_[i] = planPos(r, leg.end);
            anchorRot_[i] = restFoot_[i];
        }
        const ik::TwoBoneChain& arm = joints.arms[i];
        if (arm.start >= 0) {
            upperArmLength_[i] = math::distance(planPos(r, arm.start), planPos(r, arm.mid));
            forearmLength_[i] = math::distance(planPos(r, arm.mid), planPos(r, arm.end));
            armLength_[i] = upperArmLength_[i] + forearmLength_[i];
            const math::Vec3 hang = math::safeNormalize(planPos(r, arm.end) - planPos(r, arm.start), -math::kUp);
            const math::Vec3 back = -math::kForward - hang * math::dot(-math::kForward, hang);
            hangDir_[i] = chestInv * hang;
            hangBend_[i] = chestInv * math::safeNormalize(back, -math::kForward);
            restHandChest_[i] = chestInv * (planPos(r, arm.end) - chestPos);
        }
    }
    if (joints.head >= 0) restHead_ = planRot(r, joints.head);
    if (joints.legs[0].start >= 0 && joints.legs[1].start >= 0) {
        const math::Vec3 mid = 0.5f * (planPos(r, joints.legs[0].start) + planPos(r, joints.legs[1].start));
        hipCentre_ = pelvisInv * (mid - restPelvis_.position);
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
        anchorNormal_[i] = math::kUp;
        anchorRot_[i] = restFoot_[i];
        grounded_[i] = true;
        support_[i] = 1.0f;
    }
    for (float& a : act_) a = 0.0f;
    hipsRef_ = pelvisOf(output_).position;
    hipsRefVel_ = hipsVel_ = math::Vec3(0.0f);
    for (int e = 0; e < kEnds; ++e) trunkFirst_[e] = trunkEnd_[e] = -1;
    lastValid_ = false;
    armAngle_[0] = armAngle_[1] = 0.0f;
    seeded_ = true;
}

void BodyDecider::rebase(const math::Transform& toNew) {
    for (int i = 0; i < 2; ++i) {
        anchor_[i] = toNew.transformPoint(anchor_[i]);
        anchorNormal_[i] = toNew.rotation * anchorNormal_[i];
        anchorRot_[i] = toNew.rotation * anchorRot_[i];
    }
    for (Target& l : last_) {
        l.position = toNew.transformPoint(l.position);
        l.rotation = toNew.rotation * l.rotation;
        l.velocity = toNew.rotation * l.velocity;
    }
    hipsRef_ = toNew.transformPoint(hipsRef_);
    hipsRefVel_ = toNew.rotation * hipsRefVel_;
    hipsVel_ = toNew.rotation * hipsVel_;
    for (math::Vec3& p : lastCentre_) p = toNew.transformPoint(p);
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

void BodyDecider::movePelvis(Plan& p, math::Vec3 delta) {
    // A move without a turn shifts the pelvis and everything below it by
    // `delta` (no skeleton pass needed: the same result).
    const int pelvis = joints_.pelvis;
    const int parent = rig_->parent(pelvis);
    const math::Vec3 local =
        parent >= 0 ? math::inverse(math::toTransform(p.models[static_cast<size_t>(parent)]).rotation) * delta : delta;
    p.pose.locals[static_cast<size_t>(pelvis)].position += local;
    for (int j = pelvis; j < rig_->subtreeEnd(pelvis); ++j) p.models[static_cast<size_t>(j)][3] += math::Vec4(delta, 0.0f);
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
}

math::Vec3 BodyDecider::planCentreOfMass() const {
    const Plan& p = desired_;
    math::Vec3 sum(0.0f);
    for (size_t j = 0; j < mass_.size() && j < p.models.size() && j < centre_.size(); ++j)
        if (mass_[j] > 0.0f) sum += mass_[j] * math::Vec3(p.models[j] * math::Vec4(centre_[j], 1.0f));
    return totalMass_ > 0.0f ? sum / totalMass_ : sum;
}

math::Quat BodyDecider::restFootLevel(int i) const {
    // Flat on its ground, heading as it stands.
    return math::normalize(math::fromTo(math::kUp, anchorNormal_[i]) * restFoot_[i]);
}

float BodyDecider::reachableHeight(const Plan& p, int i, const BodyDeciderSettings& s, bool& valid) const {
    // The pelvis can be no higher than puts the hip at the leg's reach from
    // the anchor (the knee keeps limbMinBend); level distance beyond that
    // reach: no height works.
    const math::Transform pelvis = pelvisOf(p);
    const math::Vec3 hip = pelvis.position + pelvis.rotation * hipOffset_[i];
    const float reach = limbReach(2 + i, s);
    const math::Vec3 d = anchor_[i] - hip;
    const float flat = d.x * d.x + d.z * d.z;
    valid = flat < reach * reach;
    if (!valid) return pelvis.position.y;
    return anchor_[i].y + std::sqrt(reach * reach - flat) - (hip.y - pelvis.position.y);
}

float BodyDecider::lowestHeight(const Plan& p, const BodyDeciderSettings& s, bool& any) const {
    // A leg with its knee bent by q spans sqrt(t^2 + s^2 + 2 t s cos q) from
    // hip to ankle: the pelvis no lower than puts every grounded leg's hip
    // that far from its anchor at the knee's bound (two feet or one,
    // BIOMECHANICS 9.4, 9.7). A leg whose level distance alone keeps it
    // longer bounds nothing.
    int grounded = 0;
    for (int i = 0; i < 2; ++i)
        if (joints_.legs[i].start >= 0 && grounded_[i]) ++grounded;
    const float knee = grounded >= 2 ? s.twoLegKneeMax : s.oneLegKneeMax;
    const math::Transform pelvis = pelvisOf(p);
    float lowest = 0.0f;
    any = false;
    for (int i = 0; i < 2; ++i) {
        if (joints_.legs[i].start < 0 || !grounded_[i]) continue;
        const float t = thighLength_[i], sh = shinLength_[i];
        const float span = bentSpan(t, sh, knee), span2 = span * span;
        const math::Vec3 hip = pelvis.position + pelvis.rotation * hipOffset_[i];
        const math::Vec3 d = anchor_[i] - hip;
        const float flat = d.x * d.x + d.z * d.z;
        if (flat >= span2) continue;
        const float y = anchor_[i].y + std::sqrt(span2 - flat) - (hip.y - pelvis.position.y);
        lowest = any ? std::max(lowest, y) : y;
        any = true;
    }
    return lowest;
}

// ---- the tick -----------------------------------------------------------------

void BodyDecider::decide(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    out.count = 0;
    out.plan = nullptr;
    out.planned.fill(0);
    out.offsetClamped = out.heightClamped = false;
    for (ReachReport& r : out.reach) r = {};
    if (!rig_) return;
    if (!seeded_) seed(rig_->restPose());
    out.plan = &output_.models;
    for (int e = 0; e < kEnds; ++e) trunkFirst_[e] = trunkEnd_[e] = -1;

    blendWeights(t, s);
    pinFeet(t);
    supportAndHips(t, s, out);
    desiredPose(t, s, out);
    limitOutput(t, s, out);
    report(t, s, out);
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

void BodyDecider::pinFeet(const TargetSet& t) {
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
        } else if (f.mode == TargetMode::Driven) {
            // On its way: the anchor follows the foot, so a foot let go is
            // pinned where it is. On the ground until it lifts clear of
            // where it was pinned (the weight shift before a step).
            const float groundY = anchor_[i].y;
            const math::Vec3 at = planPos(output_, ankle);
            grounded_[i] = grounded_[i] && at.y <= groundY + kOnGround;
            anchor_[i] = math::Vec3(at.x, groundY, at.z);  // the ground under it
        } else {
            // Free: pinned where it last was. A foot lifted and let go lands
            // on its anchor (the planner pins it once it is down).
            grounded_[i] = true;
        }
    }
}

void BodyDecider::supportAndHips(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Support weights: a pinned foot carries weight, a foot on its way (even
    // still on the ground: the weight shift) or free of the ground none;
    // each ramps at 1 / supportBlendTime (Winter 1995's load/unload).
    const float step = t.dt / std::max(s.supportBlendTime, 1e-3f);
    // The hips' place: a foot on a timed swing (a step: the planner gives its
    // swing share) still counts where its target is, so the hips go between
    // the feet and travel half the foot's way as it moves (walking is not
    // statically balanced: the body vaults over the stance leg on its own
    // momentum, its centre of mass between the feet); a foot held up
    // without timing counts by its load (all weight goes to the other).
    float load = 0.0f, sum = 0.0f;
    math::Vec3 centre(0.0f);
    int timed = -1;
    for (int i = 0; i < 2; ++i) {
        const int ankle = joints_.legs[i].end;
        const bool pinned = ankle >= 0 && t.foot[i].mode == TargetMode::Pinned;
        support_[i] = ramp(support_[i], pinned ? 1.0f : 0.0f, step);
        if (ankle < 0) continue;
        load += support_[i];
        const bool swinging = t.foot[i].mode == TargetMode::Driven && t.foot[i].swing >= 0.0f;
        if (swinging) timed = i;
        const float w = swinging ? 1.0f : support_[i];
        const math::Vec3 at = pinned ? anchor_[i] : swinging ? t.foot[i].position : planPos(output_, ankle);
        centre += w * at;
        sum += w;
    }
    const float restAnkleY = 0.5f * (restFootY_[0] + restFootY_[1]);
    math::Vec3 ref;
    if (load > 1e-3f && sum > 1e-3f) {
        centre /= sum;
        // Through a timed swing the hips go toward the standing foot and
        // back (hipsSway at mid-swing, a sine from toe-off to heel strike;
        // Orendurff et al. 2004), never past half the feet's width.
        const int down = timed >= 0 ? 1 - timed : -1;
        if (down >= 0 && t.foot[down].mode == TargetMode::Pinned) {
            math::Vec3 side = anchor_[down] - t.foot[timed].position;
            side.y = 0.0f;
            const float width = math::length(side);
            const float share = std::sin(math::kPi * std::clamp(t.foot[timed].swing, 0.0f, 1.0f));
            if (width > 1e-4f) centre += side * (std::min(s.hipsSway, 0.5f * width) * share / width);
        }
        // Over the weighted support feet, the hips' own target (standing on
        // the root) pulling by its passive share; as high as every grounded
        // leg reaches at the clamp (the knees keep their bend; a slope or a
        // stair comes in through the feet's heights).
        const float k = std::clamp(s.passiveHips, 0.0f, 1.0f);
        float height = restPelvis_.position.y + (centre.y - restAnkleY);
        bool any = false;
        for (int i = 0; i < 2; ++i) {
            if (joints_.legs[i].start < 0 || !grounded_[i]) continue;
            bool valid = false;
            const float most = reachableHeight(output_, i, s, valid);
            if (!valid) continue;
            height = any ? std::min(height, most) : most;
            any = true;
        }
        ref = math::Vec3(math::lerp(centre.x, restPelvis_.position.x, k), height,
                         math::lerp(centre.z, restPelvis_.position.z, k));
        hipsRefVel_ = t.dt > 0.0f ? (ref - hipsRef_) / t.dt : math::Vec3(0.0f);
    } else {
        // No load (flight): the reference carries on as it was going; the
        // physics body is ballistic meanwhile (the pin fades with support).
        ref = hipsRef_ + hipsRefVel_ * t.dt;
    }
    hipsRef_ = ref;
    // Driven hips: the given place, by the blended weight, split across and up.
    const Target& hips = given(t, HipsW);
    const float k = act_[HipsW] * std::clamp(hips.weightPosition, 0.0f, 1.0f);
    const float kh = k * std::clamp(t.hipsWeightHorizontal, 0.0f, 1.0f), kv = k * std::clamp(t.hipsWeightVertical, 0.0f, 1.0f);
    if (k > 0.0f) {
        ref.x = math::lerp(ref.x, hips.position.x, kh);
        ref.z = math::lerp(ref.z, hips.position.z, kh);
        ref.y = math::lerp(ref.y, hips.position.y, kv);
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
    d.pose.locals = output_.pose.locals;  // same size: no allocation
    const rig::Pose& rest = rig_->restPose();
    for (int j : {joints_.spine[0], joints_.spine[1], joints_.spine[2], joints_.neck, joints_.head, joints_.clavicles[0],
                  joints_.clavicles[1]})
        if (j >= 0) {
            d.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            markPlanned(j, out);
        }
    refresh(d);
    placePelvis(d, math::Transform{out.hipsReference, restPelvis_.rotation});

    // Reach: a hand or lifted foot out of its limb's reach brings the trunk
    // in (its queue and plan), then the standing balance and the grounded
    // legs move the hips, and the trunk goes again until every end is within
    // reach or nothing gives any more.
    math::Vec3 endTarget[kEnds];
    bool endOn[kEnds] = {false, false, false, false};
    for (int h = 0; h < 2; ++h) {
        if (joints_.arms[h].end < 0 || act_[h] <= 0.0f || t.hand[h].mode != TargetMode::Driven) continue;
        const Target& hand = given(t, h);
        const float k = act_[h] * std::clamp(hand.weightPosition, 0.0f, 1.0f);
        const math::Vec3 restWorld = planPos(d, joints_.spine[2]) + planRot(d, joints_.spine[2]) * restHandChest_[h];
        endTarget[h] = math::lerp(restWorld, hand.position, k);
        endOn[h] = true;
        handTrunk(h, endTarget[h], s, out);
    }
    for (int i = 0; i < 2; ++i) {
        if (joints_.legs[i].end < 0 || grounded_[i] || act_[FootLW + i] <= 0.0f || t.foot[i].mode != TargetMode::Driven)
            continue;
        const Target& foot = given(t, FootLW + i);
        const float k = act_[FootLW + i] * std::clamp(foot.weightPosition, 0.0f, 1.0f);
        endTarget[2 + i] = math::lerp(anchor_[i], foot.position, k);
        endOn[2 + i] = true;
        footTrunk(i, endTarget[2 + i], foot.velocity, s, out);
    }
    auto shortOf = [&] {
        float sum = 0.0f;
        for (int e = 0; e < kEnds; ++e)
            if (endOn[e] && endRoot(e) >= 0)
                sum += std::max(0.0f, math::distance(planPos(d, endRoot(e)), endTarget[e]) - limbReach(e, s));
        return sum;
    };
    balanceStanding(t, s);
    solveLegs(d, t, s, out);
    constexpr int kRounds = 8;
    constexpr float kProgress = 1e-3f;  // m per round: less is no progress
    float last = shortOf();
    for (int round = 0; round < kRounds && last > kProgress; ++round) {
        for (int e = 0; e < kEnds; ++e)
            if (trunkFirst_[e] >= 0) planTrunk(e, endTarget[e], trunkFirst_[e], trunkEnd_[e], s, out);
        balanceStanding(t, s);
        solveLegs(d, t, s, out);
        const float now = shortOf();
        if (last - now < kProgress) break;
        last = now;
    }

    // Driven chest and head: the chain turns by its blended weight, shared by
    // place along it (chest: chest, spine_02, spine_01, pelvis; head: head,
    // neck).
    if (act_[ChestW] > 0.0f && joints_.spine[2] >= 0) {
        const Target& chest = given(t, ChestW);
        const float k = act_[ChestW] * std::clamp(chest.weightRotation, 0.0f, 1.0f);
        const math::Quat delta = math::slerp(math::identityQuat(), chest.rotation * math::inverse(planRot(d, joints_.spine[2])), k);
        const int chain[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
        const int first = out.count;
        turnChain(chain, 4, delta, chest.angularVelocity, Goal::Chest, s, out);
        for (int q = first; q < out.count; ++q) {
            const PartOrder& o = out.orders[static_cast<size_t>(q)];
            if (o.share <= 0.0f) continue;
            if (o.joint == joints_.pelvis) {
                math::Transform p = pelvisOf(d);
                p.rotation = math::normalize(math::slerp(math::identityQuat(), delta, o.share) * p.rotation);
                placePelvis(d, p);
            } else {
                turnPlanned(d, o.joint, math::slerp(math::identityQuat(), delta, o.share));
            }
        }
        solveLegs(d, t, s, out);  // the legs under the turned pelvis
    }
    // Posture: the chest back toward upright by its passive share when no
    // hand drives the trunk (a hand's trunk serves the hand fully).
    if (act_[HandLW] <= 0.0f && act_[HandRW] <= 0.0f && act_[ChestW] <= 0.0f) {
        const float k = std::clamp(s.passiveChest, 0.0f, 1.0f);
        bool turned = false;
        for (int j : joints_.spine)
            if (j >= 0 && k > 0.0f) {
                math::Quat& q = d.pose.locals[static_cast<size_t>(j)].rotation;
                q = math::normalize(math::slerp(q, rest.locals[static_cast<size_t>(j)].rotation, k));
                turned = true;
            }
        if (turned && joints_.spine[0] >= 0) refresh(d, joints_.spine[0]);
    }
    freeArms(t, s, out);
    levelHead(t, s, out);
    solveArms(d, t, s, out);
}

void BodyDecider::balanceStanding(const TargetSet& t, const BodyDeciderSettings& s) {
    // Standing (both feet grounded, none on its way, the hips not driven):
    // the hips go where the plan's centre of mass is over the support (a
    // bend takes the hips back: anticipatory posture, Massion 1992). On the
    // way through a step the hips follow the support reference itself
    // (walking is not statically balanced: the body vaults over the stance
    // leg on its own momentum).
    Plan& d = desired_;
    const bool standing = grounded_[0] && grounded_[1] && act_[FootLW] <= 0.0f && act_[FootRW] <= 0.0f &&
                          act_[HipsW] <= 0.0f && totalMass_ > 0.0f;
    if (!standing) return;
    constexpr int kPasses = 3;
    constexpr float kOver = 1e-3f;  // m: the centre of mass over the support
    for (int pass = 0; pass < kPasses; ++pass) {
        const math::Vec3 com = planCentreOfMass();
        const math::Vec3 off(hipsRef_.x - com.x, 0.0f, hipsRef_.z - com.z);
        if (math::length(off) <= kOver) break;
        movePelvis(d, off);  // the upper body moves with the hips; the next pass measures again
    }
    (void)t;
    (void)s;
}

void BodyDecider::solveLegs(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Grounded feet: each leg reaches its anchor from the hip (two-bone,
    // the knee tracking the foot: its pole turns with the foot from
    // standing), the foot flat on its ground; the pelvis is already within
    // every grounded leg's reach (the height constraint). Lifted feet: the
    // leg reaches its target, clamped to its reach (the knee at limbMinBend: it
    // keeps its bend), the knee forward of the pelvis, the foot turned as
    // given by the blended rotation weight.
    const rig::Pose& rest = rig_->restPose();
    const bool isOutput = &p == &output_;
    if (!isOutput) {
        // The desired pelvis no lower than the grounded knees allow, and no
        // higher than every grounded leg reaches (that wins: the output
        // follows it within its bounds; meanwhile a leg short of its anchor
        // is reported, never stretched past its clamp).
        bool bounded = false;
        const float lowest = lowestHeight(p, s, bounded);
        if (bounded && pelvisOf(p).position.y < lowest) movePelvis(p, math::kUp * (lowest - pelvisOf(p).position.y));
        for (int i = 0; i < 2; ++i) {
            if (joints_.legs[i].start < 0 || !grounded_[i]) continue;
            bool valid = false;
            const float most = reachableHeight(p, i, s, valid);
            const float y = pelvisOf(p).position.y;
            if (valid && y > most) {
                movePelvis(p, math::kUp * (most - y));
                out.heightClamped = true;
            }
        }
    }
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& leg = joints_.legs[i];
        if (leg.start < 0) continue;
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
        if (grounded_[i]) {
            pole = anchorRot_[i] * footToPelvis_[i] * s.kneePole;
            footRot = anchorRot_[i];
        } else {
            pole = planRot(p, joints_.pelvis) * s.kneePole;
            footRot = math::normalize(math::slerp(anchorRot_[i], foot.rotation, kr));
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
                // A lifted leg's joints turn no faster than a limb does.
                math::Quat& thigh = p.pose.locals[static_cast<size_t>(leg.start)].rotation;
                math::Quat& shin = p.pose.locals[static_cast<size_t>(leg.mid)].rotation;
                thigh = limitTurn(prevThigh, thigh, s.limbMaxRate, t.dt);
                shin = limitTurn(prevShin, shin, s.limbMaxRate, t.dt);
            }
            refresh(p, leg.start);
        }
        // The foot: flat on its ground, or as given, within the ankle's range.
        math::Quat footLocal = math::normalize(math::inverse(planRot(p, leg.mid)) * footRot);
        if (limits_) footLocal = ik::clampRotation(footLocal, limits_->limitIn(leg.end, p.pose));
        p.pose.locals[static_cast<size_t>(leg.end)].rotation = footLocal;
        refresh(p, leg.end);
        for (int j : {leg.start, leg.mid, leg.end}) markPlanned(j, out);
    }
}

void BodyDecider::solveArms(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Driven hands: shoulder and elbow from the shoulder, the reach clamped
    // (the elbow keeps its bend). The elbow's side (the solve's pole): the
    // arm turns from hanging relaxed off the chest to the target by the
    // shortest swing, adding no twist, and the elbow's bend turns with it
    // (no fixed direction, so none the arm can line up with). Free hands
    // keep what freeArms placed on the desired pose.
    const rig::Pose& rest = rig_->restPose();
    const bool isOutput = &p == &output_;
    const math::Quat chest = joints_.spine[2] >= 0 ? planRot(p, joints_.spine[2]) : math::identityQuat();
    const math::Vec3 chestPos = joints_.spine[2] >= 0 ? planPos(p, joints_.spine[2]) : math::Vec3(0.0f);
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (arm.start < 0) continue;
        const math::Quat prevUpper = p.pose.locals[static_cast<size_t>(arm.start)].rotation;
        const math::Quat prevFore = p.pose.locals[static_cast<size_t>(arm.mid)].rotation;
        const bool driven = t.hand[h].mode == TargetMode::Driven;
        const bool fading = !driven && act_[h] > 0.0f && isOutput;
        if (!driven && !fading) {
            if (isOutput) {
                // The free arm follows the desired pose (its swing), no faster
                // than a limb turns.
                for (int j : {joints_.clavicles[h], arm.start, arm.mid, arm.end}) {
                    if (j < 0) continue;
                    math::Quat& o = p.pose.locals[static_cast<size_t>(j)].rotation;
                    o = limitTurn(o, desired_.pose.locals[static_cast<size_t>(j)].rotation, s.limbMaxRate, t.dt);
                }
                refresh(p, joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : arm.start);
            }
            continue;
        }
        const Target& hand = given(t, h);
        // Driven: toward the target by its blended weight. Fading: the
        // last target's full solve, blended below with the free arm.
        const float k = fading ? 1.0f : act_[h] * std::clamp(hand.weightPosition, 0.0f, 1.0f);
        const math::Vec3 restWorld = chestPos + chest * restHandChest_[h];
        for (int j : {arm.start, arm.mid})
            p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
        refresh(p, arm.start);
        const math::Vec3 from = planPos(p, arm.start);
        const float reach = limbReach(h, s);
        const math::Vec3 target = clampToReach(from, math::lerp(restWorld, hand.position, k), reach);
        const math::Vec3 hang = chest * hangDir_[h], bend = chest * hangBend_[h];
        const math::Vec3 dir = math::safeNormalize(target - from, hang);
        const math::Quat swing = math::dot(hang, dir) < -0.9999f
                                     ? math::fromAxisAngle(math::safeNormalize(math::cross(hang, -bend), math::kRight), math::kPi)
                                     : math::fromTo(hang, dir);
        if (ik::solveTwoBone(arm, p.models, target, swing * bend, 1.0f, p.pose)) {
            math::Quat& upper = p.pose.locals[static_cast<size_t>(arm.start)].rotation;
            math::Quat& fore = p.pose.locals[static_cast<size_t>(arm.mid)].rotation;
            if (fading) {
                // Toward the free arm (the desired pose's) as the activation fades.
                const rig::Pose& freeArm = isOutput ? desired_.pose : p.pose;
                upper = math::normalize(math::slerp(freeArm.locals[static_cast<size_t>(arm.start)].rotation, upper, act_[h]));
                fore = math::normalize(math::slerp(freeArm.locals[static_cast<size_t>(arm.mid)].rotation, fore, act_[h]));
            }
            if (isOutput) {
                upper = limitTurn(prevUpper, upper, s.limbMaxRate, t.dt);
                fore = limitTurn(prevFore, fore, s.limbMaxRate, t.dt);
            }
            refresh(p, arm.start);
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
    if (dt > 0.0f) {
        const math::Vec3 d = want.position - have.position;
        const float dist = math::length(d);
        const float speed = std::min(s.hipsMaxSpeed, std::sqrt(2.0f * std::max(s.hipsMaxAccel, 0.0f) * dist));
        const math::Vec3 vWant = dist > 1e-6f ? d * (speed / dist) : math::Vec3(0.0f);
        hipsVel_ += clampLength(vWant - hipsVel_, std::max(s.hipsMaxAccel, 0.0f) * dt);
        have.position += hipsVel_ * dt;
        have.rotation = limitTurn(have.rotation, want.rotation, s.hipsMaxTurnRate, dt);
    } else {
        have = want;
        hipsVel_ = math::Vec3(0.0f);
    }
    // Hard constraint: around the root (the capsule leads). The height under
    // every grounded leg's reach is the desired pose's (solveLegs), so the
    // hips get there within their bounds.
    {
        const math::Vec3 flat = level(have.position);
        const float r = math::length(flat);
        if (r > s.hipsOffsetMax && r > 0.0f) {
            const math::Vec3 n = flat / r;
            have.position -= n * (r - s.hipsOffsetMax);
            hipsVel_ -= n * std::max(0.0f, math::dot(hipsVel_, n));
            out.offsetClamped = true;
        }
    }
    placePelvis(output_, have);
    out.pelvisOffset = level(have.position);

    // The trunk and head joints toward the desired ones by rate.
    for (int j : {joints_.spine[0], joints_.spine[1], joints_.spine[2], joints_.clavicles[0], joints_.clavicles[1]})
        limitJoint(j, s.trunkMaxRate, dt);
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
        r.shortfall = std::max(0.0f, math::distance(from, target) - reach);
        r.reached = r.shortfall <= 1e-3f;
        r.direction = math::safeNormalize(target - planPos(output_, end));
    }
}

// ---- reach queues (the desired pose) ----------------------------------------------------

void BodyDecider::handTrunk(int h, math::Vec3 target, const BodyDeciderSettings& s, BodyQueue& out) {
    const ik::TwoBoneChain& arm = joints_.arms[h];
    if (arm.start < 0) return;
    // Out of the arm's reach every part of the queue moves from the start,
    // shared by its place (shareQueue). The queue is led by where the target
    // is (BIOMECHANICS 9.1, 9.3):
    //  - below the hips: a forward bend, lumbar spine and hips together
    //    (Esola et al. 1996), then the upper trunk, the legs last (lowering
    //    the hips within the knees' bound, 9.4; people default to the stoop
    //    and change mostly the knees): spine_01, pelvis tilt, spine_02,
    //    chest, clavicle, hips lower;
    //  - behind the shoulders: a turn is mostly thoracic: chest, spine_02,
    //    spine_01, clavicle, pelvis, hips;
    //  - ahead or to the side: the shoulder girdle, then the trunk leans, then
    //    the whole body (Levin et al. 2001): clavicle, chest, spine_02,
    //    spine_01, pelvis tilt, hips.
    if (math::distance(planPos(desired_, arm.start), target) > limbReach(h, s))
        trunkOrders(h, target, math::Vec3(0.0f), s, out);
}

void BodyDecider::trunkOrders(int h, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s,
                              BodyQueue& out) {
    const Plan& d = desired_;
    const ik::TwoBoneChain& arm = joints_.arms[h];
    const Goal which = h == 0 ? Goal::HandL : Goal::HandR;
    const math::Vec3 shoulder = planPos(d, arm.start);
    const float reach = limbReach(h, s);
    const math::Transform pelvis = pelvisOf(d);
    const math::Vec3 hips = pelvis.position + pelvis.rotation * hipCentre_;
    const math::Vec3 forward = pelvis.rotation * math::kForward;
    enum Stage { Clavicle, Chest, Spine2, Spine1, Tilt, Move };
    static constexpr Stage kLow[6] = {Spine1, Tilt, Spine2, Chest, Clavicle, Move};
    static constexpr Stage kBehind[6] = {Chest, Spine2, Spine1, Clavicle, Tilt, Move};
    static constexpr Stage kAhead[6] = {Clavicle, Chest, Spine2, Spine1, Tilt, Move};
    const Stage* order = target.y < hips.y ? kLow : math::dot(target - shoulder, forward) < 0.0f ? kBehind : kAhead;

    const int first = out.count;
    int lumbar = -1, tilt = -1;  // their orders, for the measured split
    for (int k = 0; k < 6; ++k) {
        if (order[k] == Move) {
            // The legs move the hips toward the hand (the planted feet stay:
            // the legs bend). Driven hips are not the reach's to move.
            add(out, joints_.pelvis, PartMove::Move, math::normalize(target - shoulder), velocity, which, false,
                1.0f - act_[HipsW]);
            continue;
        }
        const int j = order[k] == Clavicle ? joints_.clavicles[h]
                    : order[k] == Chest    ? joints_.spine[2]
                    : order[k] == Spine2   ? joints_.spine[1]
                    : order[k] == Spine1   ? joints_.spine[0]
                                           : joints_.pelvis;  // Tilt: the pelvis tilts at the hips
        if (j < 0) continue;
        const Swing sw = swingToward(order[k] == Tilt ? hips : planPos(d, j), shoulder, target, reach);
        if (sw.angle <= kMinAngle) continue;
        if (order[k] == Spine1) lumbar = out.count;
        if (order[k] == Tilt) tilt = out.count;
        add(out, j, PartMove::Turn, sw.axis, velocity, which, false, order[k] == Tilt ? 1.0f - act_[HipsW] : 1.0f);
    }
    shareQueue(first, s, out);

    // A bend (not a turn): the lumbar spine and the hips split their combined
    // share by the measured lumbopelvic rhythm at the trunk's flexion now.
    if (order != kBehind && lumbar >= 0 && tilt >= 0 && joints_.spine[2] >= 0) {
        PartOrder& l = out.orders[static_cast<size_t>(lumbar)];
        PartOrder& p = out.orders[static_cast<size_t>(tilt)];
        if (l.share > 0.0f && p.share > 0.0f) {
            const math::Vec3 trunk = math::safeNormalize(planPos(d, joints_.spine[2]) - pelvis.position, math::kUp);
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
    planTrunk(h, target, first, out.count, s, out);
}

void BodyDecider::footTrunk(int i, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s,
                            BodyQueue& out) {
    // A lifted foot out of its leg's reach from the planned hip: the leg it
    // stands on bends (its hip and knee lower the hips: the reach distance of
    // a one-leg foot reach goes with the stance leg's hip and knee flexion,
    // Robinson & Gribble 2008, BIOMECHANICS 9.7), then the pelvis tilts on
    // the stance hip. Shares by place.
    const ik::TwoBoneChain& leg = joints_.legs[i];
    if (leg.start < 0) return;
    const Goal which = i == 0 ? Goal::FootL : Goal::FootR;
    const math::Vec3 hip = planPos(desired_, leg.start);
    const float reach = limbReach(2 + i, s);
    if (math::distance(hip, target) <= reach) return;
    const int e = 2 + i;
    const int first = out.count;
    if (dropToward(hip, target, reach) > 0.0f)
        add(out, joints_.pelvis, PartMove::Move, math::normalize(target - hip), velocity, which, false, 1.0f - act_[HipsW]);
    const Swing sw = swingToward(tiltPivot(e), hip, target, reach);
    if (sw.angle > kMinAngle) add(out, joints_.pelvis, PartMove::Turn, sw.axis, velocity, which, false, 1.0f - act_[HipsW]);
    shareQueue(first, s, out);
    trunkFirst_[e] = first;
    trunkEnd_[e] = out.count;
    planTrunk(e, target, first, out.count, s, out);
}

int BodyDecider::endRoot(int e) const { return e < 2 ? joints_.arms[e].start : joints_.legs[e - 2].start; }

float BodyDecider::endReach(int e) const { return e < 2 ? armLength_[e] : legLength_[e - 2]; }

float BodyDecider::limbReach(int e, const BodyDeciderSettings& s) const {
    const float bend = std::clamp(s.limbMinBend, 0.0f, 1.5f);
    return e < 2 ? bentSpan(upperArmLength_[e], forearmLength_[e], bend)
                 : bentSpan(thighLength_[e - 2], shinLength_[e - 2], bend);
}

math::Vec3 BodyDecider::tiltPivot(int e) const {
    // A hand's pelvis tilt turns at the hips; a lifted foot's at the hip of
    // the leg it stands on (the pelvis on the stance femur).
    const Plan& d = desired_;
    const int stance = e >= 2 ? joints_.legs[1 - (e - 2)].start : -1;
    const math::Transform pelvis = pelvisOf(d);
    return stance >= 0 ? planPos(d, stance) : pelvis.position + pelvis.rotation * hipCentre_;
}

void BodyDecider::planTrunk(int e, math::Vec3 target, int first, int end, const BodyDeciderSettings& s, BodyQueue& out) {
    // In passes: each part of the queue turns by its share of the swing that
    // would bring the end's root (shoulder, hip) within reach on its own (the
    // hips lower by their share of the drop), within its range; the next pass
    // takes what is left (a part at its range leaves its part to the others).
    Plan& d = desired_;
    const int root = endRoot(e);
    constexpr int kPasses = 4;
    constexpr float kClose = 1e-3f;  // m: within reach
    const float reach = limbReach(e, s);
    const math::Quat restInv = math::inverse(restPelvis_.rotation);
    for (int pass = 0; pass < kPasses; ++pass) {
        for (int k = first; k < end; ++k) {
            const PartOrder& o = out.orders[static_cast<size_t>(k)];
            if (o.share <= 0.0f) continue;
            const math::Vec3 shoulder = planPos(d, root);  // or the lifted leg's hip
            const float gap = math::distance(shoulder, target) - reach;
            if (gap <= kClose) return;
            if (o.joint == joints_.pelvis && o.kind == PartMove::Move) {
                // The legs lower the hips (they bend; the planted feet stay)
                // by their share of the drop that alone would bring the
                // shoulder within reach, no lower than the grounded knees
                // allow (BIOMECHANICS 9.4, 9.7). Shifting them over the feet
                // is balance's, not the reach's.
                float drop = dropToward(shoulder, target, reach) * o.share;
                bool bounded = false;
                const float lowest = lowestHeight(d, s, bounded);
                if (bounded) drop = std::min(drop, std::max(0.0f, pelvisOf(d).position.y - lowest));
                if (drop > 0.0f) movePelvis(d, -math::kUp * drop);
            } else if (o.joint == joints_.pelvis) {
                // The pelvis tilts at the hips (the hips stay), within its
                // range from standing (hip flexion, BIOMECHANICS 9.3).
                const math::Vec3 hips = tiltPivot(e);
                const Swing sw = swingToward(hips, shoulder, target, reach);
                math::Transform p = pelvisOf(d);
                const math::Quat want = math::normalize(math::fromAxisAngle(sw.axis, sw.angle * o.share) * p.rotation);
                const math::Quat rel = clampRotationVector(math::normalize(want * restInv), s.pelvisMin, s.pelvisMax);
                const math::Quat turn = math::normalize(rel * restPelvis_.rotation * math::inverse(p.rotation));
                p.rotation = math::normalize(turn * p.rotation);
                p.position = hips + turn * (p.position - hips);
                placePelvis(d, p);
            } else if (o.joint == joints_.spine[0] || o.joint == joints_.spine[1] || o.joint == joints_.spine[2]) {
                // A spine joint (the clavicle has no body yet: it rides the
                // chest as built, so it is not planned).
                const Swing sw = swingToward(planPos(d, o.joint), shoulder, target, reach);
                turnPlanned(d, o.joint, math::fromAxisAngle(sw.axis, sw.angle * o.share));
            }
        }
    }
}

void BodyDecider::turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                            const BodyDeciderSettings& s, BodyQueue& out) {
    const math::Vec3 axis = axisOf(delta);
    if (math::length(axis) <= 0.0f) return;
    const int first = out.count;
    for (int k = 0; k < count; ++k) {
        const int j = joints[k];
        if (j < 0) continue;
        add(out, j, PartMove::Turn, axis, velocity, goal, false, 1.0f);
    }
    shareQueue(first, s, out);
}

void BodyDecider::shareQueue(int first, const BodyDeciderSettings& s, BodyQueue& out) const {
    // Orders come in with share 1 (free) to 0 (held: a driven pelvis); the
    // k-th one by place carries queueShare^k, then all are normalised to 1.
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

// ---- posture (the desired pose) ------------------------------------------------------------

void BodyDecider::freeArms(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // The arm swing of walking: the arms are not held; they hang from the
    // shoulders and swing opposite the legs because they cancel part of the
    // body's turning about the vertical (Collins et al. 2009; Bruijn et al.
    // 2008). Each free arm here: its swing rate takes its share (by mass and
    // lever) of cancelling the rest of the plan's vertical angular momentum
    // about the centre of mass; gravity takes it back toward hanging at its
    // pendulum rate (sqrt(m g d / I) about the shoulder).
    Plan& d = desired_;
    bool free[2];
    for (int h = 0; h < 2; ++h) free[h] = t.hand[h].mode != TargetMode::Driven && joints_.arms[h].start >= 0;
    const rig::Pose& rest = rig_->restPose();
    auto place = [&](int h) {  // hanging, swung by armAngle_ (forward +) within the shoulder's range
        const ik::TwoBoneChain& c = joints_.arms[h];
        for (int j : {joints_.clavicles[h], c.mid, c.end})
            if (j >= 0) d.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
        math::Quat& shoulder = d.pose.locals[static_cast<size_t>(c.start)].rotation;
        shoulder = rest.locals[static_cast<size_t>(c.start)].rotation * math::fromAxisAngle(math::kRight, -armAngle_[h]);
        if (limits_) shoulder = ik::clampRotation(math::normalize(shoulder), limits_->limitIn(c.start, d.pose));
        refresh(d, joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : c.start);
        markPlanned(c.start, out);
        markPlanned(c.mid, out);
    };
    for (int h = 0; h < 2; ++h)
        if (free[h]) place(h);
    if (totalMass_ <= 0.0f || t.dt <= 0.0f) {
        passiveHands(free, s, out);
        return;
    }
    const math::Vec3 com = planCentreOfMass();
    float spin = 0.0f, mass[2] = {0.0f, 0.0f}, lever[2] = {0.0f, 0.0f}, inertia[2] = {0.0f, 0.0f};
    for (size_t j = 0; j < mass_.size() && j < d.models.size() && j < centre_.size(); ++j) {
        if (mass_[j] <= 0.0f) continue;
        const math::Vec3 p(d.models[j] * math::Vec4(centre_[j], 1.0f));
        const int h = armOf_[j];
        if (h >= 0 && free[h]) {
            const float dist = math::distance(p, planPos(d, joints_.arms[h].start));
            mass[h] += mass_[j];
            lever[h] += mass_[j] * dist;
            inertia[h] += mass_[j] * dist * dist;
        } else if (lastValid_) {
            const math::Vec3 v = (p - lastCentre_[j]) / t.dt;
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
            k[h] = -(planPos(d, joints_.arms[h].start).x - com.x) * lever[h];
            kk += k[h] * k[h];
        }
    for (int h = 0; h < 2; ++h) {
        if (!free[h] || mass[h] <= 0.0f) continue;
        const float cancel = measured && kk > 1e-9f ? -s.armCancel * spin * k[h] / kk : 0.0f;
        const float pendulum = inertia[h] > 0.0f ? std::sqrt(std::max(0.0f, s.gravity) * lever[h] / inertia[h]) : 0.0f;
        armAngle_[h] += (cancel - pendulum * armAngle_[h]) * t.dt;
        place(h);
    }
    passiveHands(free, s, out);
}

void BodyDecider::passiveHands(const bool free[2], const BodyDeciderSettings& s, BodyQueue& out) {
    // A hand without a goal still has its target (hanging at rest in the
    // chest's frame): the swing goes on, and the hand goes the passive share
    // of the way from where the swing has it to that target (the elbow
    // bending back, as it hangs).
    Plan& d = desired_;
    const float k = std::clamp(s.passiveHands, 0.0f, 1.0f);
    if (k <= 0.0f) return;
    const math::Quat chest = joints_.spine[2] >= 0 ? planRot(d, joints_.spine[2]) : math::identityQuat();
    const math::Vec3 chestPos = joints_.spine[2] >= 0 ? planPos(d, joints_.spine[2]) : math::Vec3(0.0f);
    for (int h = 0; h < 2; ++h) {
        const ik::TwoBoneChain& arm = joints_.arms[h];
        if (!free[h] || arm.end < 0) continue;
        const math::Vec3 at = planPos(d, arm.end);
        const math::Vec3 want = at + k * (chestPos + chest * restHandChest_[h] - at);
        if (ik::solveTwoBone(arm, d.models, want, chest * hangBend_[h], 1.0f, d.pose)) {
            if (limits_)
                for (int j : {arm.start, arm.mid})
                    d.pose.locals[static_cast<size_t>(j)].rotation =
                        ik::clampRotation(math::normalize(d.pose.locals[static_cast<size_t>(j)].rotation), limits_->limitIn(j, d.pose));
            refresh(d, arm.start);
            markPlanned(arm.start, out);
            markPlanned(arm.mid, out);
        }
    }
}

void BodyDecider::levelHead(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // The head keeps the gaze level in the world, facing the chest's heading
    // (head stabilisation, Pozzo et al. 1990), pulled by its passive share
    // toward its own target (standing, facing the root's way); a driven head
    // turns the given way by its blended weight (head, then neck).
    Plan& d = desired_;
    if (joints_.head < 0) return;
    const int parent = rig_->parent(joints_.head);
    const int below = joints_.spine[2] >= 0 ? joints_.spine[2] : parent;
    if (parent < 0 || below < 0) return;
    const math::Vec3 f = planRot(d, below) * math::kForward;
    math::Quat want = math::slerp(math::fromAxisAngle(math::kUp, std::atan2(f.x, f.z)) * restHead_, restHead_,
                                  std::clamp(s.passiveHead, 0.0f, 1.0f));
    if (act_[HeadW] > 0.0f) {
        const Target& head = given(t, HeadW);
        const float k = act_[HeadW] * std::clamp(head.weightRotation, 0.0f, 1.0f);
        want = math::slerp(want, head.rotation, k);
        const int chain[2] = {joints_.head, joints_.neck};
        turnChain(chain, 2, want * math::inverse(planRot(d, joints_.head)), head.angularVelocity, Goal::Head, s, out);
    }
    math::Quat& head = d.pose.locals[static_cast<size_t>(joints_.head)].rotation;
    head = math::normalize(math::inverse(planRot(d, parent)) * want);
    if (limits_) head = ik::clampRotation(head, limits_->limitIn(joints_.head, d.pose));
    refresh(d, joints_.head);
    markPlanned(joints_.head, out);
}

} // namespace motion
