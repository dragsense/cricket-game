#include "motion/body_decider.h"

#include "core/log.h"
#include "motion/body_chain.h"
#include "ik/joint_limits.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace motion {

namespace {

// A passive share: the action's own (TargetSet::passive) or, negative, the decider's.
float passiveShare(float given, float fallback) { return std::clamp(given >= 0.0f ? given : fallback, 0.0f, 1.0f); }

// --- Limb reach math (law of cosines; BIOMECHANICS 9). Pure, no allocations. ---

constexpr float kReachEpsilon = 1e-4f;  // m; closer than this counts as reached

// Clamps `q` per axis of its rotation vector (axis * angle, radians).
math::Quat clampRotationVector(math::Quat q, math::Vec3 min, math::Vec3 max) {
    const math::Vec3 c = math::clamp(math::rotationVector(q), min, max);
    const float angle = math::length(c);
    return angle < math::kEpsilon ? math::identityQuat() : math::fromAxisAngle(c / angle, angle);
}

// `target` moved toward `from` so it is at most `reach` away.
math::Vec3 clampToReach(math::Vec3 from, math::Vec3 target, float reach) {
    const math::Vec3 d = target - from;
    const float len = math::length(d);
    return len <= reach ? target : from + d * (reach / len);
}

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

math::Vec3 level(math::Vec3 v) { return math::Vec3(v.x, 0.0f, v.z); }

math::Vec3 clampLength(math::Vec3 v, float max) {
    const float len = math::length(v);
    return len > max && len > 0.0f ? v * (max / len) : v;
}

// A two-bone limb's span (root to end) with its middle joint bent `bend`.
float bentSpan(float a, float b, float bend) {
    return std::sqrt(std::max(a * a + b * b + 2.0f * a * b * std::cos(bend), 0.0f));
}

// The capture point's path through a weight shift (time T, share u): from `a`
// moving at `v0` to `b` at rest, no acceleration at either end (a quintic;
// v0 = 0: the minimum-jerk path), and its velocity.
math::Vec3 shiftPath(math::Vec3 a, math::Vec3 v0, math::Vec3 b, float T, float u) {
    const float h = u - 6.0f * u * u * u + 8.0f * u * u * u * u - 3.0f * u * u * u * u * u;
    return a + (b - a) * math::minimumJerk(u) + v0 * (T * h);
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
    steps_[0] = steps_[1] = StepPlanner{};
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
            restShoulder_[i] = planPos(r, arm.start);
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
    appliedList_ = 0.0f;
    appliedPivot_ = math::Vec3(0.0f);
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
        listLeft_[i] = 1.0f;
    }
    for (float& a : act_) a = 0.0f;
    hipsRef_ = pelvisOf(output_).position;
    hipsRefVel_ = hipsVel_ = math::Vec3(0.0f);
    dcmOn_ = false;
    dcmStartVel_ = math::Vec3(0.0f);
    lastValid_ = false;
    armAngle_[0] = armAngle_[1] = 0.0f;
    lastStepped_ = -1;
    wasSwinging_[0] = wasSwinging_[1] = false;
    steps_[0] = steps_[1] = StepPlanner{};
    lastStarted_ = -1;
    anchorYaw_[0] = anchorYaw_[1] = stanceYaw_ = 0.0f;
    footEvent_ = false;
    lastGoalVelocity_ = math::Vec3(0.0f);
    lastPelvisYaw_ = lastPelvisRate_ = trunkTwist_ = trunkTwistRate_ = 0.0f;
    trunkTicks_ = 0;
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
    lastPathEnd_ = toNew.transformPoint(lastPathEnd_);
    for (int i = 0; i < 2; ++i)
        for (math::Vec3* p : {&stepFrom_[i]})
            *p = toNew.transformPoint(*p);
    hipsRef_ = toNew.transformPoint(hipsRef_);
    hipsRefVel_ = toNew.rotation * hipsRefVel_;
    hipsVel_ = toNew.rotation * hipsVel_;
    for (math::Vec3* p : {&dcmHips_, &dcmXi_, &dcmStart_}) *p = toNew.transformPoint(*p);
    dcmStartVel_ = toNew.rotation * dcmStartVel_;
    // (The free arms cancel the body's turning against its heading: a turn of
    // the heading itself is meant.)
    for (math::Vec3& p : lastCentre_) p += toNew.position;
    for (StepPlanner& p : steps_) p.rebase(toNew);
    // A foot keeps its turn on the ground too.
    const math::Vec3 ahead = toNew.rotation * math::kForward;
    const float turned = std::atan2(ahead.x, ahead.z);
    for (float* yaw : {&anchorYaw_[0], &anchorYaw_[1], &stepYawFrom_[0], &stepYawFrom_[1], &stepYaw_[0], &stepYaw_[1],
                       &stanceYaw_})
        *yaw += turned;
    lastGoalVelocity_ = toNew.rotation * lastGoalVelocity_;
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
math::Vec3 BodyDecider::desiredPelvis() const { return pelvisOf(desired_).position; }

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

float BodyDecider::reachableHeight(const Plan& p, int i, const BodyDeciderSettings& s, bool& valid) const {
    // The pelvis can be no higher than puts the hip at the leg's reach from
    // where the leg reaches to (the anchor, or the ankle risen by the
    // toe-off in the weight shift: reachFrom_; the knee keeps limbMinBend);
    // level distance beyond that reach: no height works.
    const math::Transform pelvis = pelvisOf(p);
    const math::Vec3 hip = pelvis.position + pelvis.rotation * hipOffset_[i];
    const float reach = limbReach(2 + i, s);
    const math::Vec3 d = reachFrom_[i] - hip;
    const float flat = d.x * d.x + d.z * d.z;
    valid = flat < reach * reach;
    if (!valid) return pelvis.position.y;
    return reachFrom_[i].y + std::sqrt(reach * reach - flat) - (hip.y - pelvis.position.y);
}

// ---- the tick -----------------------------------------------------------------

void BodyDecider::decide(const TargetSet& given, const BodyDeciderSettings& s, BodyQueue& out) {
    out.count = 0;
    out.plan = nullptr;
    out.planned.fill(0);
    out.heightClamped = false;
    out.landed[0] = out.landed[1] = false;
    for (ReachReport& r : out.reach) r = {};
    if (!rig_) return;
    if (!seeded_) seed(rig_->restPose());
    out.plan = &output_.models;
    for (int e = 0; e < kEnds; ++e) {
        rangeMiss_[e] = 0.0f;
    }

    // The decider's own steps: each stepping foot's target over time (Start:
    // the weight shifts off it, Up: the swing, Land: pinned on its spot) in
    // place of the one given; the parts follow by the step's passive shares
    // unless the action gives its own.
    work_ = given;  // no allocation (a plain struct)
    const TargetSet& t = work_;
    bool stepping = false;
    // A foot never slides (the user's rule): given a spot on the ground
    // further than stepMargin from where it is (standing there, or held up
    // in the air), it steps there, in its path's time if it has one (the
    // sync formula), else the gait's, and is pinned there as it lands; on a
    // path it lands turned as given, and a turn alone takes a step too.
    constexpr float kTurned = 0.05f;  // rad
    for (int i = 0; i < 2; ++i) {
        const Target& f = given.foot[i];
        const int ankle = joints_.legs[i].end;
        if (ankle < 0 || steps_[i].running() || steps_[1 - i].running() || f.mode != TargetMode::Pinned) continue;
        const math::Vec3 from = grounded_[i] ? anchor_[i] : planPos(output_, ankle);
        math::Vec3 way = f.position - from;
        if (grounded_[i]) way.y = 0.0f;  // standing: only a new place on the ground
        float turn = anchorYaw_[i];
        if (f.pathLeft > 0.0f) {
            const math::Vec3 toes = f.rotation * math::kForward;
            turn += std::remainder(std::atan2(toes.x, toes.z) - turn, math::kTwoPi);
        }
        if (math::length(way) <= s.stepMargin && std::abs(turn - anchorYaw_[i]) <= kTurned) continue;
        timeStep(s);
        stepNow_[i] = nextStep_;
        if (f.pathLeft > 0.0f) stepNow_[i].duration = f.pathLeft;
        timeWeightShift(i, f.position, f.pathLeft > 0.0f, s);
        startStep(i, from, turn);
        steps_[i].start(i, from, f.position, f.contactNormal);
        lastStarted_ = i;
        footEvent_ = true;
    }
    for (int i = 0; i < 2; ++i) {
        StepPlanner& p = steps_[i];
        if (!p.running()) continue;
        p.tick(t.dt, stepNow_[i], work_);
        const float yaw = math::lerp(stepYawFrom_[i], stepYaw_[i], p.along());
        work_.foot[i].rotation = math::fromAxisAngle(math::kUp, yaw);
        stepping = true;
        if (p.landed()) {
            out.landed[i] = true;
            out.landedSpot[i] = p.landing();
            anchorYaw_[i] = stepYaw_[i];
            footEvent_ = true;
        }
    }
    if (stepping) {
        const PassiveShares& p = given.passive;
        if (p.head < 0.0f && p.hands < 0.0f && p.hips < 0.0f && p.chest < 0.0f) work_.passive = s.stepPassive;
    }

    if (s.gravity <= 0.0f && !warnedNoGravity_) {
        warnedNoGravity_ = true;
        LOG_WARN("body decider: gravity is {} m/s^2 (<= 0): no capture-point pendulum", s.gravity);
    }
    blendWeights(t, s);
    pinFeet(t, s);
    rangePelvis(s);
    supportAndHips(t, ranged_, out);
    desiredPose(t, ranged_, out);
    limitOutput(t, ranged_, out);
    report(t, ranged_, out);
    timeSwings(t, ranged_);
}

int BodyDecider::stepFoot() const {
    if (lastStarted_ >= 0 && steps_[lastStarted_].running()) return lastStarted_;
    for (int i = 0; i < 2; ++i)
        if (steps_[i].running()) return i;
    return -1;
}

float BodyDecider::stepShare() const {
    const int f = stepFoot() >= 0 ? stepFoot() : lastStarted_;
    return f >= 0 ? steps_[f].share() : 0.0f;
}

void BodyDecider::timeSwings(const TargetSet& t, const BodyDeciderSettings& s) {
    // Each swing lands with the body, planned (never chased each tick: a
    // guess re-made every tick fed on itself): on a foot's event or a change
    // of the hips target's motion, the rest of each swing from the body's
    // prediction of its hips: the other foot standing, the swing lands by the
    // end of that stance at the latest (its gait's time if sooner). A swing
    // keeps its progress.
    // On the hips target's path (the sync formula) a swing lands as the hips
    // get to its end; a new path or one whose end moved is an event too.
    const Target& hips = given(t, HipsW);
    const math::Vec3 goal = hips.velocity;
    const bool changed = math::length(goal - lastGoalVelocity_) > 1e-4f;
    lastGoalVelocity_ = goal;
    const bool onPath = act_[HipsW] > 0.0f && hips.pathLeft >= 0.0f;
    constexpr float kPathMoved = 0.01f;  // m
    const bool pathEvent = onPath && (lastPathLeft_ < 0.0f || math::distance(hips.pathEnd, lastPathEnd_) > kPathMoved);
    if (onPath && (pathEvent || lastPathLeft_ < 0.0f)) lastPathEnd_ = hips.pathEnd;
    lastPathLeft_ = onPath ? hips.pathLeft : -1.0f;
    if (!footEvent_ && !changed && !pathEvent) return;
    footEvent_ = false;
    for (int f = 0; f < 2; ++f) {
        const int o = 1 - f;
        if (!steps_[f].running()) continue;
        float left;
        if (onPath) {
            // The rest of the step from where the foot is, shared as when it
            // started (its weight shift keeps its share of the clock).
            left = pathStepTime(t, f, steps_[f].target(), steps_[f].landing(), pathSteps_[f], s);
            if (left < 0.0f) continue;  // the path has ended: the swing keeps its own time
        } else if (!steps_[o].running() && grounded_[o] && t.foot[o].mode == TargetMode::Pinned) {
            left = std::min(steps_[f].timeLeft(stepNow_[f]), stanceLeft(o, s));
        } else {
            continue;
        }
        steps_[f].setTimeLeft(left, stepNow_[f]);
    }
}

float BodyDecider::stanceLeft(int o, const BodyDeciderSettings& s) const {
    // The body's level velocity (the plan's hips).
    const math::Vec3 velocity = level(hipsVel_);
    const float speed = math::length(velocity);
    if (speed < 1e-3f) return 0.0f;
    const math::Vec3 dir = velocity / speed;
    const math::Transform pelvis = pelvisOf(output_);
    const math::Vec3 d = pelvis.position + pelvis.rotation * hipOffset_[o] - anchor_[o];
    const float along = d.x * dir.x + d.z * dir.z;
    const float side = std::abs(d.x * dir.z - d.z * dir.x);
    // The leg at its reach holds the hips reachDrop below standing that far
    // past it (as the balance step's foot left behind).
    const math::Vec3 restHip = restPelvis_.position + restPelvis_.rotation * hipOffset_[o];
    const float up = restHip.y - restFootY_[o] - s.reachDrop;
    const float reach = limbReach(2 + o, s);
    const float flat2 = up < reach ? reach * reach - up * up - side * side : 0.0f;
    const float most = flat2 > 0.0f ? std::sqrt(flat2) : 0.0f;
    return std::max(0.0f, (most - along) / speed);
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
    // The hips' place: over the weighted feet (by their load shares, even
    // unless the planner gives them: the COM over the foot that carries it,
    // the trailing knee bending by the legs' reach); a foot held up without timing
    // counts by its load (all weight goes to the other). Their height counts
    // a foot on a timed swing where its target is. Level, through a step of
    // the decider's own the hips follow the capture point's path instead
    // (stepHips), and settle from there onto the feet after it lands.
    float load = 0.0f, sum = 0.0f;
    math::Vec3 centre(0.0f);
    int timed = -1;
    const float share[2] = {loadShare(t, 0), loadShare(t, 1)};
    for (int i = 0; i < 2; ++i) {
        const int ankle = joints_.legs[i].end;
        const bool pinned = ankle >= 0 && t.foot[i].mode == TargetMode::Pinned;
        support_[i] = ramp(support_[i], pinned ? 1.0f : 0.0f, step);
        if (ankle < 0) continue;
        load += support_[i];
        const bool swinging = t.foot[i].mode == TargetMode::Driven && t.foot[i].swing >= 0.0f;
        if (swinging) timed = i;
        const float w = swinging ? 1.0f : support_[i] * share[i];
        const math::Vec3 at = pinned ? anchor_[i] : swinging ? t.foot[i].position : planPos(output_, ankle);
        centre += w * at;
        sum += w;
    }
    math::Vec3 ref;
    if (load > 1e-3f && sum > 1e-3f) {
        centre /= sum;
        // Level: a step of the decider's own with the other foot standing
        // moves the hips by the capture point's path; after it they settle
        // onto the feet the same way (x' = -w0 (x - xi), xi now the feet's
        // centre) until within a millimetre. Without gravity no pendulum:
        // the hips stay through a step.
        const int down = timed >= 0 ? 1 - timed : -1;
        const float w0 = pendulumRate(centre.y, s);
        if (down >= 0 && t.foot[down].mode == TargetMode::Pinned && steps_[timed].running() && w0 > 0.0f) {
            const math::Vec3 at = stepHips(timed, t.dt, s);
            centre.x = at.x;
            centre.z = at.z;
        } else if (timed >= 0) {
            dcmOn_ = true;
            centre.x = dcmHips_.x;
            centre.z = dcmHips_.z;
        } else if (dcmOn_) {
            constexpr float kSettled = 1e-3f;  // m
            dcmXi_ = centre;
            if (w0 > 0.0f) dcmHips_ += (centre - dcmHips_) * std::min(1.0f, w0 * t.dt);
            const math::Vec3 off = centre - dcmHips_;
            if (w0 <= 0.0f || off.x * off.x + off.z * off.z < kSettled * kSettled) {
                dcmOn_ = false;
            } else {
                centre.x = dcmHips_.x;
                centre.z = dcmHips_.z;
            }
        }
        if (!dcmOn_) dcmHips_ = dcmXi_ = centre;  // a step starts from here
        // The hips stay where they are (the user, 2026-11-23: no movement,
        // only turns); a driven hips target still moves them (below).
        ref = hipsRef_;
        hipsRefVel_ = math::Vec3(0.0f);
    } else {
        // No load (flight): the reference carries on as it was going; the
        // physics body is ballistic meanwhile (the pin fades with support).
        ref = hipsRef_ + hipsRefVel_ * t.dt;
    }
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
    d.pose.locals = output_.pose.locals;  // same size: no allocation
    const rig::Pose& rest = rig_->restPose();
    for (int j : {joints_.spine[0], joints_.spine[1], joints_.spine[2], joints_.neck, joints_.head, joints_.clavicles[0],
                  joints_.clavicles[1]})
        if (j >= 0) {
            d.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            markPlanned(j, out);
        }
    refresh(d);
    stanceYaw_ = stanceNow();
    float pelvisYaw = stanceYaw_ + pelvisTurn(t, s);
    math::Quat pelvisRot = math::fromAxisAngle(math::kUp, pelvisYaw) * restPelvis_.rotation;
    {
        // Pelvic list: the side whose leg carries less weight drops, by the
        // weight's share: it comes with the weight shift, so it leads a lift
        // (Massion 1992). Turned on the output pelvis about the hip that
        // carries the weight (limitOutput), so that hip never rises; here the
        // trunk turns back from it.
        // Eased (smoothstep of the weight's share) so it starts and stops
        // without a jerk although the weight shifts at an even rate.
        // The lifted side rises again with its foot's path: level when the
        // foot lands (pelvic obliquity peaks early in single support and is
        // level again at the other foot's initial contact, Perry 2010), so
        // the landing leg takes the weight under a level pelvis. Held after
        // the landing until that leg carries all its weight. A foot held up
        // without timing keeps its list.
        for (int i = 0; i < 2; ++i) {
            const Target& f = t.foot[i];
            if (f.mode == TargetMode::Driven && f.swing >= 0.0f) {
                const float u = std::clamp(f.swing, 0.0f, 1.0f);
                listLeft_[i] = std::min(listLeft_[i], 1.0f - u * u * (3.0f - 2.0f * u));
            } else if (support_[i] >= 1.0f) {
                listLeft_[i] = 1.0f;
            }
        }
        const float load = support_[0] + support_[1];
        const float x = std::clamp(support_[0] - support_[1], -1.0f, 1.0f), ax = std::abs(x);
        const float left = listLeft_[x >= 0.0f ? 1 : 0];  // the side that drops: the one with less weight
        listAngle_ = load > 1e-3f && joints_.legs[0].start >= 0 && joints_.legs[1].start >= 0
                         ? s.pelvisList * left * std::copysign(ax * ax * (3.0f - 2.0f * ax), x)  // +: the left side up
                         : 0.0f;
        listAxis_ = math::fromAxisAngle(math::kUp, pelvisYaw) * math::kForward;
    }
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
        solveLegs(d, t, s, out);  // the legs under the turned pelvis
    }
    // Posture: the chest back toward upright by its passive share when no
    // hand drives the trunk (a hand's trunk serves the hand fully).
    if (act_[HandLW] <= 0.0f && act_[HandRW] <= 0.0f && act_[ChestW] <= 0.0f) {
        const float k = passiveShare(t.passive.chest, s.passiveChest);
        bool turned = false;
        for (int j : joints_.spine)
            if (j >= 0 && k > 0.0f) {
                math::Quat& q = d.pose.locals[static_cast<size_t>(j)].rotation;
                q = math::normalize(math::slerp(q, rest.locals[static_cast<size_t>(j)].rotation, k));
                turned = true;
            }
        if (turned && joints_.spine[0] >= 0) refresh(d, joints_.spine[0]);
    }
    // The chest's turn through the trunk's spring-damper (Prins 2019), shared
    // evenly by the three spine joints; a hand or chest target driving the
    // trunk serves its goal instead (the spring keeps running).
    trunkSpring(pelvisYaw, t.dt, s);
    if (act_[HandLW] <= 0.0f && act_[HandRW] <= 0.0f && act_[ChestW] <= 0.0f && std::abs(trunkTwist_) > 1e-5f) {
        const math::Quat part = math::fromAxisAngle(math::kUp, trunkTwist_ / 3.0f);
        for (int j : joints_.spine)
            if (j >= 0) turnPlanned(d, j, part);
    }
    // The trunk upright over the pelvic list: the lumbar and lower thoracic
    // spine bend the other way (side flexion), shared evenly by the three
    // spine joints; a hand or chest target driving the trunk serves its goal.
    if (act_[HandLW] <= 0.0f && act_[HandRW] <= 0.0f && act_[ChestW] <= 0.0f && std::abs(listAngle_) > 1e-6f) {
        const math::Quat part = math::fromAxisAngle(listAxis_, -listAngle_ / 3.0f);
        for (int j : joints_.spine)
            if (j >= 0) turnPlanned(d, j, part);
    }
    freeArms(t, s, out);
    levelHead(t, s, out);
    solveArms(d, t, s, out);
}

float BodyDecider::stanceNow() const {
    float yaw = 0.0f;
    for (int i = 0; i < 2; ++i)
        yaw += 0.5f * (steps_[i].running() ? math::lerp(stepYawFrom_[i], stepYaw_[i], steps_[i].along()) : anchorYaw_[i]);
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

float BodyDecider::loadShare(const TargetSet& t, int i) {
    const float own = t.foot[i].load, other = t.foot[1 - i].load;
    return 2.0f * (own >= 0.0f ? std::min(own, 1.0f) : other >= 0.0f ? 1.0f - std::min(other, 1.0f) : 0.5f);
}

void BodyDecider::startStep(int i, math::Vec3 from, float yaw) {
    stepFrom_[i] = from;
    stepYawFrom_[i] = anchorYaw_[i];
    stepYaw_[i] = yaw;
}

void BodyDecider::trunkSpring(float pelvisYaw, float dt, const BodyDeciderSettings& s) {
    // The chest (its turn in the world c) on a rotational spring-damper to the
    // pelvis (p); its turn against the pelvis, q = c - p, is moved by the
    // pelvis's turning: q'' = -w^2 q - 2 z w q' - p''.
    if (dt <= 0.0f) return;
    const float rate = trunkTicks_ > 0 ? (pelvisYaw - lastPelvisYaw_) / dt : 0.0f;
    const float accel = trunkTicks_ > 1 ? (rate - lastPelvisRate_) / dt : 0.0f;
    lastPelvisYaw_ = pelvisYaw;
    lastPelvisRate_ = rate;
    trunkTicks_ = std::min(trunkTicks_ + 1, 2);
    const float w = std::max(s.trunkTurnFrequency, 0.0f);
    trunkTwistRate_ += dt * (-w * w * trunkTwist_ - 2.0f * s.trunkTurnDamping * w * trunkTwistRate_ - accel);
    trunkTwist_ += dt * trunkTwistRate_;
    const float maxTwist = std::max(s.trunkMaxTwist, 0.0f);  // the thoracic turn's range each way
    trunkTwist_ = std::clamp(trunkTwist_, -maxTwist, maxTwist);
}

float BodyDecider::pelvisTurn(const TargetSet& t, const BodyDeciderSettings& s) const {
    // Each foot where it stands, or where its target is on a timed swing (the
    // swinging leg carries its hip with it); the pelvis turns by
    // pelvisTurnPerStep per metre the feet are apart fore-aft (the step's
    // length). Left foot ahead: the left hip forward, a turn to the right
    // (negative about +Y).
    math::Vec3 at[2];
    for (int i = 0; i < 2; ++i) {
        if (joints_.legs[i].end < 0) return 0.0f;
        const Target& f = t.foot[i];
        const bool swinging = f.mode == TargetMode::Driven && f.swing >= 0.0f;
        at[i] = swinging ? f.position : grounded_[i] ? anchor_[i] : planPos(output_, joints_.legs[i].end);
    }
    const math::Vec3 apart = math::fromAxisAngle(math::kUp, -stanceYaw_) * (at[0] - at[1]);
    const float ahead = apart.z;  // left ahead of right, along the feet
    return -s.pelvisTurnPerStep * ahead;
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
    math::Vec3 legBeyond[2] = {math::Vec3(0.0f), math::Vec3(0.0f)};
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
            if (grounded_[i]) {
                refresh(p, leg.start);
                if (!isOutput && t.foot[i].mode == TargetMode::Pinned) {
                    // The chain down to the ground: what the thigh asks past
                    // its range its foot takes, pivoting on the ground (passUp).
                    const math::Vec3 beyond = beyondRange(p, leg.start, s);
                    bool unused = false;
                    if (math::length(beyond) > kMinAngle) passUp(p, &leg.end, 1, beyond, s, unused);
                }
            } else {
                // What the thigh asks past its range: the chain's, after both legs.
                if (!isOutput && !legChained_) legBeyond[i] = beyondRange(p, leg.start, s);
                clampLimb(p, 2 + i, chain, 2, target);
            }
        }
        // The foot: flat on its ground, or as given, within the ankle's range.
        math::Quat footLocal = math::normalize(math::inverse(planRot(p, leg.mid)) * footRot);
        if (limits_) footLocal = ik::clampRotation(footLocal, limits_->limitIn(leg.end, p.pose));
        p.pose.locals[static_cast<size_t>(leg.end)].rotation = footLocal;
        refresh(p, leg.end);
        for (int j : {leg.start, leg.mid, leg.end}) markPlanned(j, out);
    }
    // The chain (passUp), once a tick: what a lifted thigh asks past its
    // range the pelvis, spine_01, spine_02 and the chest take in turn; the
    // legs once more under the turned pelvis.
    if (!isOutput && !legChained_) {
        legChained_ = true;
        const int chain[4] = {joints_.pelvis, joints_.spine[0], joints_.spine[1], joints_.spine[2]};
        bool turned = false;
        for (int i = 0; i < 2; ++i)
            if (math::length(legBeyond[i]) > kMinAngle) passUp(p, chain, 4, legBeyond[i], s, turned);
        if (turned) solveLegs(p, t, s, out);
    }
}

void BodyDecider::solveArms(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // Driven hands: shoulder and elbow from the shoulder, the reach clamped
    // (the elbow keeps its bend). The elbow's side (the solve's pole): the
    // arm turns from hanging relaxed off the chest to the target by the
    // shortest swing, adding no twist, and the elbow's bend turns with it
    // (no fixed direction, so none the arm can line up with); out of the
    // shoulder's range, the elbow goes round as little as brings it in
    // (fitPole). Free hands
    // keep what freeArms placed on the desired pose.
    const rig::Pose& rest = rig_->restPose();
    const bool isOutput = &p == &output_;
    bool pelvisTurned = false;
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
                    clampJoint(p, j);
                }
                refresh(p, joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : arm.start);
                rangeMiss_[h] = 0.0f;
            }
            continue;
        }
        const Target& hand = given(t, h);
        // Driven: toward the target by its blended weight. Fading: the
        // last target's full solve, blended below with the free arm.
        const float k = fading ? 1.0f : act_[h] * std::clamp(hand.weightPosition, 0.0f, 1.0f);
        const float reach = limbReach(h, s);
        auto armTarget = [&] {
            const math::Quat chest = joints_.spine[2] >= 0 ? planRot(p, joints_.spine[2]) : math::identityQuat();
            const math::Vec3 chestPos = joints_.spine[2] >= 0 ? planPos(p, joints_.spine[2]) : math::Vec3(0.0f);
            const math::Vec3 restWorld = chestPos + chest * restHandChest_[h];
            return clampToReach(planPos(p, arm.start), math::lerp(restWorld, hand.position, k), reach);
        };
        auto armAtRest = [&] {
            for (int j : {arm.start, arm.mid})
                p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
            refresh(p, arm.start);
        };
        armAtRest();
        if (!isOutput && driven && limits_) {
            // The chain rule (body_chain.h) from the upper arm up: the arm
            // to the target, its elbow placed as the solve below does; what the shoulder asks
            // past chainOnset of its range on each of its range axes the
            // clavicle, the chest, spine_02, spine_01 and the pelvis take in
            // turn, the same way about the same axis, each its own part
            // within its own limits (passUp).
            const math::Vec3 at = armTarget(), shoulder = planPos(p, arm.start);
            // The arm's own first: its elbow round as little as keeps the
            // shoulder in range (fitPole), as the solve below.
            float unused = 0.0f;
            const math::Vec3 armPole = fitPole(armShape(h, p, s), armHang(h, p), armFrame(h, p), shoulder, at, unused);
            if (ik::solveTwoBone(arm, p.models, at, armPole, 1.0f, p.pose)) {
                const math::Vec3 beyond = beyondRange(p, arm.start, s);
                armAtRest();
                if (math::length(beyond) > kMinAngle) {
                    const int chain[5] = {joints_.clavicles[h], joints_.spine[2], joints_.spine[1], joints_.spine[0],
                                          joints_.pelvis};
                    passUp(p, chain, 5, beyond, s, pelvisTurned);
                    refresh(p, arm.start);
                }
            } else {
                armAtRest();
            }
        }
        const math::Vec3 from = planPos(p, arm.start);
        const math::Vec3 target = armTarget();
        const ArmHang hang = armHang(h, p);
        math::Vec3 pole = relaxedPole(hang, math::safeNormalize(target - from, hang.down));
        float miss = 0.0f;
        if (limits_) pole = fitPole(armShape(h, p, s), hang, armFrame(h, p), from, target, miss);
        if (ik::solveTwoBone(arm, p.models, target, pole, 1.0f, p.pose)) {
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
            const int chain[2] = {arm.start, arm.mid};
            clampLimb(p, h, chain, 2, target);
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

math::Vec3 BodyDecider::beyondRange(const Plan& p, int j, const BodyDeciderSettings& s) const {
    // What joint j asks past chainOnset of its range on each of its range
    // axes (before its clamp), as one rotation (world rotation vector).
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
        const float over = std::abs(need[a]) - chainOwnPart(std::abs(need[a]), bound, s.chainOnset);
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
        // A grounded foot: it pivots on the ground about the vertical by
        // what comes to it that way (its next turn, anchorYaw_).
        bool pivot = false;
        for (int f = 0; f < 2; ++f)
            if (j == joints_.legs[f].end && grounded_[f]) {
                const float yaw = math::dot(rest, math::kUp);
                anchorYaw_[f] += yaw;
                rest -= math::kUp * yaw;
                pivot = true;
            }
        if (pivot) continue;
        // The joint's angles now and its range, on its range axes (world).
        math::Vec3 axis[3], now, lo, hi;
        if (j == joints_.pelvis) {
            // Its range from standing (pelvisMin / pelvisMax, character axes);
            // driven hips are not the chain's to turn.
            if (act_[HipsW] >= 1.0f) break;
            const math::Transform pel = pelvisOf(p);
            for (int a = 0; a < 3; ++a) axis[a] = math::Vec3(a == 0, a == 1, a == 2);
            now = math::rotationVector(math::normalize(pel.rotation * math::inverse(pelvisBase())));
            lo = s.pelvisMin;
            hi = s.pelvisMax;
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
            math::Transform pel = pelvisOf(p);
            pel.rotation = math::normalize(turn * pel.rotation);
            placePelvis(p, pel);
            pelvisTurned = true;
        } else {
            turnPlanned(p, j, turn);
        }
        rest -= take;
    }
    return rest;
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
    // The pelvic list is the output's own: turned exactly about the hip
    // that carries the weight (so that hip never rises past its leg); its
    // place goes into the hips' bounded motion with the rest (the muscles
    // move the whole pelvis). `have` keeps its place; its turn without the
    // list follows the desired one's.
    const math::Vec3 localForward = math::inverse(restPelvis_.rotation) * math::kForward;
    have.rotation = math::normalize(have.rotation * math::inverse(math::fromAxisAngle(localForward, appliedList_)));
    // The hip of the leg with more weight (the same at no list); the list
    // eases with the weight's share over supportBlendTime.
    appliedPivot_ = hipOffset_[support_[0] >= support_[1] ? 0 : 1];
    appliedList_ = listAngle_;
    const math::Quat roll = math::fromAxisAngle(localForward, appliedList_);
    const math::Vec3 wantPosition = want.position + want.rotation * (appliedPivot_ - roll * appliedPivot_);
    if (dt > 0.0f) {
        const math::Vec3 d = wantPosition - have.position;
        const float dist = math::length(d);
        // Never faster than lands on the target this tick: the continuous
        // law alone overshoots within a dt^2 of it and the standing plan
        // chattered (pelvis +-0.17 mm in a 6-tick cycle, knee 4.5..5.5 deg).
        const float speed = std::min({s.hipsMaxSpeed, std::sqrt(2.0f * std::max(s.hipsMaxAccel, 0.0f) * dist), dist / dt});
        const math::Vec3 vWant = dist > 1e-6f ? d * (speed / dist) : math::Vec3(0.0f);
        hipsVel_ += clampLength(vWant - hipsVel_, std::max(s.hipsMaxAccel, 0.0f) * dt);
        have.position += hipsVel_ * dt;
        have.rotation = limitTurn(have.rotation, want.rotation, s.hipsMaxTurnRate, dt);
    } else {
        have = math::Transform{wantPosition, want.rotation};
        hipsVel_ = math::Vec3(0.0f);
    }
    have.rotation = math::normalize(have.rotation * roll);
    // The height under every grounded leg's reach is the desired pose's
    // (solveLegs), so the hips get there within the legs' bounds.
    placePelvis(output_, have);

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
        r.shortfall = std::max({0.0f, math::distance(from, target) - reach, rangeMiss_[e]});
        r.reached = r.shortfall <= 1e-3f;
        r.direction = math::safeNormalize(target - planPos(output_, end));
    }
}

// ---- the balance step (the capture point) ----------------------------------------------

float BodyDecider::outsideFeet(math::Vec3 p, const math::Vec3* ankles, int n, const BodyDeciderSettings& s) {
    // The support's outline: each foot a rectangle (character axes) round
    // its centre, the two joined (their hull: the segment between the
    // centres swept by the rectangle). The distance out of it, on the ground.
    if (n <= 0) return 0.0f;
    math::Vec3 centre[2];
    for (int i = 0; i < n && i < 2; ++i) centre[i] = ankles[i] + math::Vec3(0.0f, 0.0f, s.footCentreAhead);
    auto boxOut = [&](math::Vec3 c) {
        const float dx = std::max(0.0f, std::abs(p.x - c.x) - s.footHalfWidth);
        const float dz = std::max(0.0f, std::abs(p.z - c.z) - s.footHalfLength);
        return std::sqrt(dx * dx + dz * dz);
    };
    if (n == 1) return boxOut(centre[0]);
    constexpr int kSamples = 16;
    float best = boxOut(centre[0]);
    for (int k = 1; k <= kSamples; ++k)
        best = std::min(best, boxOut(math::lerp(centre[0], centre[1], static_cast<float>(k) / kSamples)));
    return best;
}

float BodyDecider::pathStepTime(const TargetSet& t, int i, math::Vec3 from, math::Vec3 spot, int steps,
                                const BodyDeciderSettings& s) const {
    // The sync formula: a step on the hips' path lands as the hips get to its
    // end: the path's time left, no shorter than the hips take at their
    // most speed; a way longer than a step is shared by `steps` steps (each
    // landing plans the next). No shorter than the leg swings there (the
    // swing at rest at both ends peaks at 1.875 D / T, the foot at most
    // legMaxRate x the leg's length).
    const Target& hips = given(t, HipsW);
    const float kw = std::clamp(hips.weightPosition, 0.0f, 1.0f);
    // A path that has ended times nothing: a step after it (the foot left
    // behind follows) takes its own time (timeWeightShift).
    if (act_[HipsW] <= 0.0f || kw <= 0.0f || hips.pathLeft <= t.dt) return -1.0f;
    math::Vec3 lag = drivenPart(hips, hips.pathEnd - pelvisOf(output_).position) * kw;
    lag.y = 0.0f;
    const float hipsTime = math::length(lag) / std::max(s.hipsMaxSpeed, 1e-3f);
    const float time = std::max(hips.pathLeft, hipsTime) / static_cast<float>(std::max(steps, 1));
    return std::max(time, swingTimeAtLeast(i, from, spot, s));
}

float BodyDecider::swingTime(int i, math::Vec3 from, math::Vec3 spot, const BodyDeciderSettings& s) const {
    const float d = std::sqrt((spot.x - from.x) * (spot.x - from.x) + (spot.z - from.z) * (spot.z - from.z));
    return std::max(math::fittsTime(d, s.fittsWidth, s.fittsA, s.fittsB), swingTimeAtLeast(i, from, spot, s));
}

float BodyDecider::swingTimeAtLeast(int i, math::Vec3 from, math::Vec3 spot, const BodyDeciderSettings& s) const {
    const float d = std::sqrt((spot.x - from.x) * (spot.x - from.x) + (spot.z - from.z) * (spot.z - from.z));
    return 1.875f * d / std::max(s.legMaxRate * legLength_[i], 1e-3f);
}

float BodyDecider::pendulumRate(float groundY, const BodyDeciderSettings& s) const {
    if (s.gravity <= 0.0f || totalMass_ <= 0.0f) return 0.0f;
    const float height = std::max(centreOfMass(output_).y - groundY, 0.3f);
    return std::sqrt(s.gravity / height);
}

void BodyDecider::timeWeightShift(int f, math::Vec3 spot, bool fixedTotal, const BodyDeciderSettings& s) {
    // The capture point (DCM) starts where the plan's hips and their motion
    // put it (x + v / w0, whatever moved them: support, a hips target, a
    // reach); by the lift it must be where the swing on the other foot
    // carries it to the centre of the new stance as the foot lands
    // (xi_lift = p + (xi_end - p) e^(-w0 T_swing)). It moves there from its
    // own speed (the body's: walking, it is never at rest; standing, the
    // minimum-jerk path), and the shift takes the shortest time whose centre
    // of pressure, xi - xi' / w0, stays within the feet the whole way (none
    // does: the step goes at once). (2026-11-21: a capture point taken as at
    // rest gave 0.5 s when it started just inside the feet, 0.12 s when 3 mm
    // outside: a pause every other step of a slow walk.) At least
    // the foot's unloading time (supportBlendTime, Winter 1995), at most
    // weightShiftMost; a foot already off the ground has no weight to shift.
    // The swing is the foot's aimed move: Fitts' law for its way (swingTime).
    // A path gives the whole time (the sync formula): the swing takes its
    // own first (all of it if the path is shorter), the shift what is left.
    StepPlannerSettings& timing = stepNow_[f];
    const int o = 1 - f;
    const float w0 = pendulumRate(anchor_[o].y, s);
    const math::Vec3 hips = pelvisOf(output_).position;
    dcmHips_ = math::Vec3(hips.x, dcmHips_.y, hips.z);
    dcmStart_ = dcmHips_ + (w0 > 0.0f ? level(hipsVel_) / w0 : math::Vec3(0.0f));
    dcmXi_ = dcmStart_;
    const float total = std::max(timing.duration, 1e-3f);
    const float own = swingTime(f, anchor_[f], spot, s);
    const float swing = fixedTotal ? std::min(own, total) : own;
    float shift = grounded_[f] ? std::max(s.supportBlendTime, 0.0f) : 0.0f;
    // The capture point moves on as the body does (walking: it is never at
    // rest), so its path through the shift starts at the body's speed.
    dcmStartVel_ = w0 > 0.0f ? level(hipsVel_) : math::Vec3(0.0f);
    if (grounded_[f] && w0 > 0.0f) {
        const math::Vec3 p = anchor_[o];
        const math::Vec3 end = 0.5f * (p + spot);
        math::Vec3 to = p + (end - p) * std::exp(-w0 * swing);
        to.y = dcmStart_.y;
        // The shortest shift whose centre of pressure (xi - xi' / w0) stays
        // within the feet the whole way; none does: the body falls, the step
        // goes at once (as when the capture point starts outside the feet).
        auto fits = [&](float T) {
            constexpr int kSamples = 16;
            constexpr float kInside = 1e-3f;  // m
            for (int k = 0; k <= kSamples; ++k) {
                const float u = static_cast<float>(k) / kSamples;
                const math::Vec3 cop = shiftPath(dcmStart_, dcmStartVel_, to, T, u) -
                                       shiftRate(dcmStart_, dcmStartVel_, to, T, u) / w0;
                if (outsideFeet(cop, anchor_, 2, s) > kInside) return false;
            }
            return true;
        };
        const float least = shift, most = std::max(s.weightShiftMost, shift);
        if (!fits(least) && fits(most)) {
            float lo = least, hi = most;
            constexpr int kHalvings = 12;
            for (int n = 0; n < kHalvings; ++n) {
                const float m = 0.5f * (lo + hi);
                (fits(m) ? hi : lo) = m;
            }
            shift = hi;
        }
    }
    if (fixedTotal) {
        shift = std::min(shift, total - swing);
        timing.liftAt = std::clamp(shift / total, 0.0f, 0.9f);
    } else {
        timing.duration = shift + swing;
        timing.liftAt = timing.duration > 1e-3f ? std::clamp(shift / timing.duration, 0.0f, 0.9f) : 0.0f;
    }
}

math::Vec3 BodyDecider::stepHips(int timed, float dt, const BodyDeciderSettings& s) {
    // The other foot carries the body: the capture point runs off it,
    // xi = p + (xi_end - p) e^(-w0 t_left), to the new stance's centre as
    // the foot lands; through the weight shift it moves from where the step
    // began to where the swing needs it (minimum jerk, timeWeightShift). The
    // hips follow it: x' = -w0 (x - xi).
    const int down = 1 - timed;
    const math::Vec3 p = anchor_[down];
    const StepPlanner& step = steps_[timed];
    const StepPlannerSettings& timing = stepNow_[timed];
    const float w0 = pendulumRate(p.y, s);
    const math::Vec3 end = 0.5f * (p + step.landing());
    const float lift = std::clamp(timing.liftAt, 0.0f, 0.9f);
    auto onSwing = [&](float left) { return p + (end - p) * std::exp(-w0 * std::max(left, 0.0f)); };
    if (step.phase() == StepPlanner::Phase::Start && lift > 0.0f) {
        const float u = std::clamp(step.share() / lift, 0.0f, 1.0f);
        dcmXi_ = shiftPath(dcmStart_, dcmStartVel_, onSwing(timing.duration * (1.0f - lift)), timing.duration * lift, u);
    } else {
        dcmXi_ = onSwing(step.timeLeft(timing));
    }
    dcmOn_ = true;
    dcmHips_ += (dcmXi_ - dcmHips_) * std::min(1.0f, w0 * dt);
    return dcmHips_;
}

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

float BodyDecider::endReach(int e) const { return e < 2 ? armLength_[e] : legLength_[e - 2]; }

float BodyDecider::limbReach(int e, const BodyDeciderSettings& s) const {
    const float bend = std::clamp(s.limbMinBend, 0.0f, 1.5f);
    return e < 2 ? bentSpan(upperArmLength_[e], forearmLength_[e], bend)
                 : bentSpan(thighLength_[e - 2], shinLength_[e - 2], bend);
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
        const math::Quat restShoulder = rest.locals[static_cast<size_t>(c.start)].rotation;
        shoulder = restShoulder * math::fromAxisAngle(math::kRight, -armAngle_[h]);
        if (limits_) {
            // At its range the arm stops (its swing is the angle it has, so
            // it never winds on past the range and comes round the other way).
            shoulder = ik::clampRotation(math::normalize(shoulder), limits_->limitIn(c.start, d.pose));
            math::Quat swing = math::normalize(math::inverse(restShoulder) * shoulder);
            if (swing.w < 0.0f) swing = -swing;
            armAngle_[h] = -2.0f * std::atan2(swing.x, swing.w);
        }
        refresh(d, joints_.clavicles[h] >= 0 ? joints_.clavicles[h] : c.start);
        markPlanned(c.start, out);
        markPlanned(c.mid, out);
    };
    for (int h = 0; h < 2; ++h)
        if (free[h]) place(h);
    if (totalMass_ <= 0.0f || t.dt <= 0.0f) {
        passiveHands(free, passiveShare(t.passive.hands, s.passiveHands), out);
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
    passiveHands(free, passiveShare(t.passive.hands, s.passiveHands), out);
}

void BodyDecider::passiveHands(const bool free[2], float share, BodyQueue& out) {
    // A hand without a goal still has its target (hanging at rest in the
    // chest's frame): the swing goes on, and the hand goes the passive share
    // of the way from where the swing has it to that target (the elbow
    // bending back, as it hangs).
    Plan& d = desired_;
    const float k = share;
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
    // The head is stabilised in space (Pozzo et al. 1990, BIOMECHANICS 4):
    // level in the world and facing the heading (the root's way: where the
    // player looks), whatever the trunk below does (its turn, sway, lean);
    // the neck takes the difference within its range. A driven head turns
    // the given way by its blended weight (head, then neck).
    Plan& d = desired_;
    if (joints_.head < 0) return;
    const int parent = rig_->parent(joints_.head);
    if (parent < 0) return;
    math::Quat want = restHead_;
    if (act_[HeadW] > 0.0f) {
        // Driven: by its blended weight, the head first, the rest down the
        // chain (passUp): neck, chest, spine_02, spine_01, the pelvis.
        const Target& head = given(t, HeadW);
        const float k = act_[HeadW] * std::clamp(head.weightRotation, 0.0f, 1.0f);
        want = math::slerp(want, head.rotation, k);
        const int chain[6] = {joints_.head, joints_.neck, joints_.spine[2], joints_.spine[1], joints_.spine[0],
                              joints_.pelvis};
        bool pelvisTurned = false;
        passUp(d, chain, 6, math::rotationVector(math::normalize(want * math::inverse(planRot(d, joints_.head)))), s,
               pelvisTurned);
        if (pelvisTurned) solveLegs(d, t, s, out);
        markPlanned(joints_.head, out);
        return;
    }
    math::Quat& head = d.pose.locals[static_cast<size_t>(joints_.head)].rotation;
    head = math::normalize(math::inverse(planRot(d, parent)) * want);
    if (limits_) head = ik::clampRotation(head, limits_->limitIn(joints_.head, d.pose));
    refresh(d, joints_.head);
    markPlanned(joints_.head, out);
}

} // namespace motion
