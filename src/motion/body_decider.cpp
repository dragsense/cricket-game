#include "motion/body_decider.h"

#include "core/log.h"
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

// Rotation about `pivot` (unit axis, angle >= 0; 0 when already within reach)
// that moves `end` so its distance to `target` becomes `reach`, or, if it
// cannot, points it straight at the target: r = |end - pivot|, d = |target -
// pivot|, reach^2 = r^2 + d^2 - 2 r d cos(angle).
struct Swing {
    math::Vec3 axis{1, 0, 0};
    float angle = 0.0f;
};

Swing swingToward(math::Vec3 pivot, math::Vec3 end, math::Vec3 target, float reach) {
    Swing s;
    if (math::distance(end, target) - reach <= kReachEpsilon) return s;
    const math::Vec3 toEnd = end - pivot, toTarget = target - pivot;
    const float r = math::length(toEnd), d = math::length(toTarget);
    if (r < math::kEpsilon || d < math::kEpsilon) return s;
    const math::Vec3 a = toEnd / r, b = toTarget / d;
    const math::Vec3 axis = math::cross(a, b);
    const float sinAB = math::length(axis);
    if (sinAB < math::kEpsilon) return s;  // already aligned (or exactly opposite): nothing to swing toward
    const float between = std::atan2(sinAB, math::dot(a, b));
    const float cosNeeded = math::clamp((r * r + d * d - reach * reach) / (2.0f * r * d), -1.0f, 1.0f);
    s.angle = std::max(0.0f, between - std::acos(cosNeeded));
    s.axis = axis / sinAB;
    return s;
}

// The smallest turn about `axis` through the origin that brings `p` within
// `reach` of `q` (as near as it comes, if it never does).
float turnAbout(math::Vec3 axis, math::Vec3 p, math::Vec3 q, float reach) {
    const math::Vec3 along = axis * math::dot(p, axis), across = p - along, turned = math::cross(axis, across);
    const float a = math::dot(across, q), b = math::dot(turned, q);
    const float c = 0.5f * (math::dot(p, p) + math::dot(q, q) - reach * reach) - math::dot(along, q);
    const float r = std::sqrt(a * a + b * b);
    if (r < math::kEpsilon) return 0.0f;
    float nearest = std::atan2(b, a);
    if (nearest < -0.5f * math::kPi) nearest += math::kTwoPi;
    return std::max(0.0f, nearest - (c < r ? std::acos(std::max(c / r, -1.0f)) : 0.0f));
}

// The easiest turn of a joint that brings `end` within `reach` of `target`:
// the least sum of (turn / room left)^2 about its own axes (x forward, y turn,
// z sideways; `frame` takes them to the model, `at` its turn now, min..max its
// range). From a forward bend (the way a body has most room) to where each
// axis turns by room^2 x what it gives (the least-effort condition).
Swing easiestSwing(math::Vec3 pivot, math::Vec3 end, math::Vec3 target, float reach, math::Quat frame, math::Vec3 at,
                   math::Vec3 min, math::Vec3 max) {
    Swing s;
    if (math::distance(end, target) - reach <= kReachEpsilon) return s;
    const math::Vec3 p = math::inverse(frame) * (end - pivot), q = math::inverse(frame) * (target - pivot);
    math::Vec3 axis = math::kRight;
    float angle = turnAbout(axis, p, q, reach);
    constexpr int kPasses = 3;
    for (int pass = 0; pass < kPasses; ++pass) {
        const math::Vec3 g = math::fromAxisAngle(axis, -0.5f * angle) * math::cross(math::fromAxisAngle(axis, angle) * p, q);
        math::Vec3 v(0.0f);
        for (int a = 0; a < 3; ++a) {
            const float room = std::max(0.0f, g[a] > 0.0f ? max[a] - at[a] : at[a] - min[a]);
            v[a] = g[a] * room * room;
        }
        const float l = math::length(v);
        if (l < 1e-9f) return Swing{};
        axis = v / l;
        angle = turnAbout(axis, p, q, reach);
    }
    s.axis = frame * axis;
    s.angle = angle;
    return s;
}

// The shortest swing with a hand up, the easiest one with a hand down (`deep`
// 0..1: between them). The shortest alone bent sideways: right under the
// shoulder it is the shoulder's side offset that decides (Davis 2002: people
// bend forward, 1-4 deg sideways).
Swing blendSwing(float deep, math::Vec3 pivot, math::Vec3 end, math::Vec3 target, float reach, math::Quat frame, math::Vec3 at,
                 math::Vec3 min, math::Vec3 max) {
    const Swing shortest = swingToward(pivot, end, target, reach);
    if (deep <= 0.0f) return shortest;
    const Swing easy = easiestSwing(pivot, end, target, reach, frame, at, min, max);
    if (easy.angle <= 0.0f) return shortest;
    const math::Vec3 v = math::lerp(shortest.axis * shortest.angle, easy.axis * easy.angle, deep);
    Swing s;
    s.angle = math::length(v);
    if (s.angle > 1e-5f) s.axis = v / s.angle;
    return s;
}

// How far (m, >= 0) `end` must go straight down so its distance to `target`
// becomes `reach`: the smaller root of |target - end + d up| = reach (or, if
// it cannot, down to the target's height). 0 when within reach or the target
// is not below.
float dropToward(math::Vec3 end, math::Vec3 target, float reach) {
    const math::Vec3 v = target - end;
    if (v.y >= 0.0f || math::length(v) - reach <= kReachEpsilon) return 0.0f;
    // |v + d up|^2 = reach^2: d^2 + 2 v.y d + |v|^2 - reach^2 = 0.
    const float disc = v.y * v.y - math::dot(v, v) + reach * reach;
    return disc < 0.0f ? -v.y : -v.y - std::sqrt(disc);
}

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
    query_.pose = rig.restPose();
    query_.models.resize(n);
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
    for (int e = 0; e < kEnds; ++e) trunkFirst_[e] = trunkEnd_[e] = -1;
    lastValid_ = false;
    armAngle_[0] = armAngle_[1] = 0.0f;
    lastStepped_ = -1;
    wasSwinging_[0] = wasSwinging_[1] = false;
    steps_[0] = steps_[1] = StepPlanner{};
    lastStarted_ = -1;
    reach_ = ReachStep{};
    anchorYaw_[0] = anchorYaw_[1] = stanceYaw_ = 0.0f;
    lastHandLeft_[0] = lastHandLeft_[1] = -1.0f;
    trunkOn_[0] = trunkOn_[1] = false;
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
    reach_.hips = toNew.transformPoint(reach_.hips);
    for (int i = 0; i < 2; ++i)
        for (math::Vec3* p : {&lastHandEnd_[i], &trunkTarget_[i], &trunkFrom_[i], &stepFrom_[i]})
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
                       &stanceYaw_, &reach_.turn})
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
        trunkFirst_[e] = trunkEnd_[e] = -1;
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
            if (reach_.on) reach_.stepped = true;
            footEvent_ = true;
        }
    }
    if (stepping) {
        const PassiveShares& p = given.passive;
        if (p.head < 0.0f && p.hands < 0.0f && p.hips < 0.0f && p.chest < 0.0f) work_.passive = s.stepPassive;
    }

    if (s.balanceStepOn && s.gravity <= 0.0f && !warnedNoGravity_) {
        warnedNoGravity_ = true;
        LOG_WARN("body decider: gravity is {} m/s^2 (<= 0): no balance step and no capture-point pendulum", s.gravity);
    }
    blendWeights(t, s);
    pinFeet(t, s);
    rangePelvis(s);
    supportAndHips(t, ranged_, out);
    desiredPose(t, ranged_, out);
    limitOutput(t, ranged_, out);
    report(t, ranged_, out);
    timeSwings(t, ranged_);
    balanceStep(t, ranged_, out);

    // The balance step's advice: a step starts (from the foot's pinned spot;
    // one at a time) or the one under way re-aims; it runs from the next
    // tick.
    const StepAdvice& a = out.step;
    if (a.wanted && a.foot >= 0 && given.balanceSteps) {
        if (!steps_[0].running() && !steps_[1].running()) {
            timeStep(s);
            stepNow_[a.foot] = nextStep_;
            if (a.time > 0.0f) stepNow_[a.foot].duration = a.time;  // on the hips' path: lands as they arrive
            timeWeightShift(a.foot, a.spot, a.time > 0.0f, s);
            // A reach's step takes its own time, or its hand's path's if longer.
            const float handLeft = reach_.on ? std::max(lastHandLeft_[0], lastHandLeft_[1]) : -1.0f;
            if (a.time <= 0.0f && handLeft > stepNow_[a.foot].duration) {
                stepNow_[a.foot].duration = handLeft;
                timeWeightShift(a.foot, a.spot, true, s);
            }
            startStep(a.foot, anchor_[a.foot], reach_.on ? reach_.turn : 0.0f);
            steps_[a.foot].start(a.foot, anchor_[a.foot], a.spot, anchorNormal_[a.foot]);
            lastStarted_ = a.foot;
            footEvent_ = true;
        } else if (steps_[a.foot].running()) {
            steps_[a.foot].retarget(a.spot);
        }
    }
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
        } else if (reach_.on) {
            continue;  // a reach's step keeps its own time (planned with it, its hips from it)
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
    const float restAnkleY = 0.5f * (restFootY_[0] + restFootY_[1]);
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
        // Over the weighted support feet, the hips' own target (standing on
        // the root) pulling by its passive share; as high as every grounded
        // leg reaches at the clamp (the knees keep their bend; a slope or a
        // stair comes in through the feet's heights).
        const float k = passiveShare(t.passive.hips, s.passiveHips);
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
    {
        // A reach turns the hips (planReach) as far as the feet under them let.
        bool hand = false;
        for (int h = 0; h < 2; ++h) hand = hand || (t.hand[h].mode == TargetMode::Driven && lastHandLeft_[h] >= 0.0f);
        if (hand && act_[HipsW] <= 0.0f && reach_.turn != 0.0f)
            pelvisYaw += std::clamp(reach_.turn - stanceYaw_, s.pelvisMin.y, s.pelvisMax.y);
    }
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
        endTarget[h] = restWorld + drivenPart(hand, hand.position - restWorld) * k;
        endOn[h] = true;
    }
    // A hand further than a body reaches standing: a step first (planReach),
    // the trunk does only its own share: its target is the hand's less the
    // way the hips still come, gone to from where it was by the feet's
    // progress (so the body steps, then bends).
    for (int h = 0; h < 2; ++h) {
        if (endOn[h] && !trunkOn_[h]) trunkTarget_[h] = endTarget[h];
        trunkOn_[h] = endOn[h];
    }
    planReach(t, s);
    if (reach_.on) {
        const math::Vec3 coming = level(reach_.hips - out.hipsReference);
        if (reach_.there) {
            // No more steps: the hips go the rest themselves, standing balance
            // (below) keeping the centre of mass within the feet.
            movePelvis(d, coming);
        } else {
            math::Vec3 feet(0.0f);  // their middle, a foot on its way where its target is
            for (int i = 0; i < 2; ++i)
                feet += 0.5f * (t.foot[i].mode == TargetMode::Driven ? t.foot[i].position : anchor_[i]);
            const float left = math::length(level(reach_.hips - feet));
            const float k = std::clamp(1.0f - left / std::max(reach_.way, 1e-4f), 0.0f, 1.0f);
            for (int h = 0; h < 2; ++h)
                if (endOn[h]) endTarget[h] = math::lerp(trunkFrom_[h], endTarget[h] - coming, k);
        }
    }
    for (int h = 0; h < 2; ++h)
        if (endOn[h]) trunkTarget_[h] = endTarget[h];
    for (int h = 0; h < 2; ++h)
        if (endOn[h]) handTrunk(h, endTarget[h], s, out);
    for (int i = 0; i < 2; ++i) {
        if (joints_.legs[i].end < 0 || grounded_[i] || act_[FootLW + i] <= 0.0f || t.foot[i].mode != TargetMode::Driven)
            continue;
        const Target& foot = given(t, FootLW + i);
        const float k = act_[FootLW + i] * std::clamp(foot.weightPosition, 0.0f, 1.0f);
        endTarget[2 + i] = anchor_[i] + drivenPart(foot, foot.position - anchor_[i]) * k;
        endOn[2 + i] = true;
        footHike(i, endTarget[2 + i], s);
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
    // A hand the shoulder's range still keeps off its target: the trunk turns
    // to it.
    bool trunkTurned = false;
    for (int h = 0; h < 2; ++h) {
        if (endOn[h] && armMiss(h, endTarget[h], s) >
                            std::max(0.0f, math::distance(planPos(d, joints_.arms[h].start), endTarget[h]) - limbReach(h, s)) + kProgress) {
            turnTrunk(h, endTarget[h], s, turnHint_[h]);
            trunkTurned = true;
        } else {
            turnHint_[h] = 0.0f;
        }
    }
    if (trunkTurned) {
        balanceStanding(t, s);
        solveLegs(d, t, s, out);
    }

    // Driven chest and head: the chain turns by its blended weight, shared by
    // place along it (chest: chest, spine_02, spine_01, pelvis; head: head,
    // neck).
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
        const int chain[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
        const int first = out.count;
        turnChain(chain, 4, delta, chest.angularVelocity, Goal::Chest, s, out);
        for (int q = first; q < out.count; ++q) {
            const PartOrder& o = out.orders[static_cast<size_t>(q)];
            if (o.share <= 0.0f) continue;
            if (o.joint == joints_.pelvis) {
                // Within the pelvis's range, as everywhere it turns.
                math::Transform p = pelvisOf(d);
                p.rotation = math::normalize(math::slerp(math::identityQuat(), delta, o.share) * p.rotation);
                const math::Quat from = math::normalize(p.rotation * math::inverse(pelvisBase()));
                const math::Vec3 v = math::rotationVector(from);
                bool beyond = false;
                for (int a = 0; a < 3; ++a) beyond = beyond || v[a] < s.pelvisMin[a] || v[a] > s.pelvisMax[a];
                if (beyond) p.rotation = math::normalize(clampRotationVector(from, s.pelvisMin, s.pelvisMax) * pelvisBase());
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
    // outward turn is the pelvis turning right, the right one's left.
    ranged_ = s;
    if (!limits_ || joints_.legs[0].start < 0) return;
    const ik::JointLimit& hip = limits_->limit(joints_.legs[0].start);  // left: + external
    const float stance = stanceNow();
    float lo = s.pelvisMin.y, hi = s.pelvisMax.y;
    for (int i = 0; i < 2; ++i) {
        if (!grounded_[i] || joints_.legs[i].start < 0) continue;
        const float foot = std::remainder(anchorYaw_[i] - stance, math::kTwoPi);
        lo = std::max(lo, i == 0 ? foot - hip.twistMax : foot + hip.twistMin);
        hi = std::min(hi, i == 0 ? foot - hip.twistMin : foot + hip.twistMax);
    }
    if (lo > hi) lo = hi = 0.5f * (lo + hi);  // feet turned past what the hips span: between
    ranged_.pelvisMin.y = lo;
    ranged_.pelvisMax.y = hi;
}

void BodyDecider::planReach(const TargetSet& t, const BodyDeciderSettings& s) {
    constexpr float kPathMoved = 0.01f;  // m
    bool any = false, event = false;
    for (int h = 0; h < 2; ++h) {
        const Target& hand = t.hand[h];
        if (hand.mode != TargetMode::Driven || hand.pathLeft < 0.0f || joints_.arms[h].start < 0) {
            lastHandLeft_[h] = -1.0f;
            continue;
        }
        any = true;
        if (lastHandLeft_[h] < 0.0f || math::distance(hand.pathEnd, lastHandEnd_[h]) > kPathMoved) {
            event = true;
            lastHandEnd_[h] = hand.pathEnd;
        }
        lastHandLeft_[h] = hand.pathLeft;
    }
    bool held = false;  // a foot held up: no step
    for (int i = 0; i < 2; ++i)
        if (t.foot[i].mode == TargetMode::Driven && t.foot[i].swing < 0.0f) held = true;
    if (!any || held || act_[HipsW] > 0.0f || !t.balanceSteps || !s.balanceStepOn || s.gravity <= 0.0f) {
        reach_.on = false;
        return;
    }
    if (!event) return;
    const bool stepped = reach_.on && reach_.stepped;
    reach_ = ReachStep{};
    float spreadTurn = 0.0f;
    // Measured from the stance the body stands in (a step under way: where
    // it began), its hips between the feet, upright, turned as the feet turn
    // it: never from a lean of the moment.
    math::Vec3 feet[2];
    for (int i = 0; i < 2; ++i) feet[i] = steps_[i].running() ? stepFrom_[i] : anchor_[i];
    const math::Vec3 mid = 0.5f * (feet[0] + feet[1]);
    const math::Quat yaw = math::fromAxisAngle(math::kUp, -s.pelvisTurnPerStep * (feet[0].z - feet[1].z));
    const float groundY = mid.y - 0.5f * (restFootY_[0] + restFootY_[1]);
    const float ahead = s.standingReachShare * rig_->height(), side = s.standingReachSideShare * rig_->height();
    // A hand down beside the body: the body turns to face it and bends forward
    // (people free to move their feet bend sideways there no more than straight
    // ahead, Lavender 1995; 4 deg, Marras 2000; BIOMECHANICS 9.8).
    const float yawNow = 0.5f * (anchorYaw_[0] + anchorYaw_[1]);
    float facing = 0.0f;
    for (int h = 0; h < 2; ++h) {
        if (lastHandLeft_[h] < 0.0f) continue;
        const float turn = facingTurn(h, t.hand[h].pathEnd, mid, yawNow, groundY, s);
        if (std::abs(turn) > std::abs(facing)) facing = turn;
    }
    const math::Quat faced = math::fromAxisAngle(math::kUp, facing);
    float need = 0.0f;  // m the hips must go, toward `dir`
    math::Vec3 dir(0.0f), shoulder(0.0f);
    for (int h = 0; h < 2; ++h) {
        if (lastHandLeft_[h] < 0.0f) continue;
        const math::Vec3 from = faced * level(yaw * (restShoulder_[h] - restPelvis_.position));
        const math::Vec3 way = level(t.hand[h].pathEnd - mid) - from;
        // The trunk's bend toward it, from standing (hips between the feet).
        const math::Vec3 hipsAt(mid.x, restPelvis_.position.y + groundY, mid.z);
        const float bend = bendWay(hipsAt + restPelvis_.rotation * hipCentre_, hipsAt + restShoulder_[h] - restPelvis_.position,
                                   t.hand[h].pathEnd, limbReach(h, s));
        if (std::abs(bend) > std::abs(spreadTurn)) spreadTurn = bend;
        const float far = math::length(way);
        if (far < 1e-4f) continue;
        const math::Vec3 d = way / far;
        const math::Vec3 dFaced = math::inverse(faced) * d;
        const float ex = dFaced.x / std::max(side, 1e-3f), ez = dFaced.z / std::max(ahead, 1e-3f);
        const float top = limbReach(h, s) + 1.0f / std::sqrt(ex * ex + ez * ez);
        const float up = std::clamp((t.hand[h].pathEnd.y - groundY) / std::max(restShoulder_[h].y, 1e-3f), 0.0f, 1.0f);
        const float over = far - math::lerp(std::min(s.floorReach, top), top, up);
        if (over > need) {
            need = over;
            dir = d;
            shoulder = from;
        }
    }
    // The trunk at its ranges and the hand still short: the hips turn.
    const bool swinging = steps_[0].running() || steps_[1].running();
    reach_.turn = yawNow + facing;
    if (need <= s.stepMargin && !swinging && shortStanding(t, s, reach_.turn) > s.stepMargin) {
        // As little as lets the body reach it, toward the bend's way (or the
        // way the trunk still has to turn, past what the hips turn on the feet).
        if (shortTurn_ != 0.0f)
            spreadTurn = facing + shortTurn_ + std::copysign(s.pelvisMax.y, shortTurn_);
        constexpr int kTurns = 8;
        float best = 1e9f;
        for (int k = 1; k <= kTurns && spreadTurn != 0.0f; ++k) {
            const float turn = yawNow + facing + (spreadTurn - facing) * static_cast<float>(k) / kTurns;
            const float left = shortStanding(t, s, turn);
            if (left < best - 1e-3f) {
                best = left;
                reach_.turn = turn;
            }
            if (left <= s.stepMargin) break;
        }
    }
    const bool turns = std::abs(reach_.turn - yawNow) > s.pelvisMax.y;
    if (turns) {
        // Further than the hips turn on these feet: a leg is freed and goes
        // round with its hip (about the standing one); the other follows if
        // that is not enough. Which: the one that takes the hips toward the
        // hand (a target ahead, the turn's far foot comes forward: Konz &
        // Bhasin 1974; behind, the near one goes back).
        math::Vec3 ends[2];
        for (int i = 0; i < 2; ++i) ends[i] = 0.5f * (feet[1 - i] + turnedFoot(i, reach_.turn));
        float off[2] = {0.0f, 0.0f};
        for (int i = 0; i < 2; ++i)
            for (int h = 0; h < 2; ++h)
                if (lastHandLeft_[h] >= 0.0f) off[i] += math::length(level(t.hand[h].pathEnd - ends[i]));
        const int f = steps_[0].running() != steps_[1].running() ? (steps_[0].running() ? 0 : 1) : off[0] < off[1] ? 0 : 1;
        reach_.on = true;
        reach_.stepped = false;
        reach_.foot = f;
        reach_.hips = ends[f];
        // Still further than a body reaches standing, turned: the step goes on
        // toward it (what the turn itself does not bring).
        if (need > s.stepMargin)
            reach_.hips += dir * std::max(0.0f, need - math::dot(ends[f] - mid, dir));
        math::Vec3 from(0.0f);
        for (int i = 0; i < 2; ++i) {
            from += 0.5f * (t.foot[i].mode == TargetMode::Driven ? t.foot[i].position : anchor_[i]);
            trunkFrom_[i] = trunkTarget_[i];
        }
        reach_.way = math::length(level(reach_.hips - from));
        return;
    }
    if (need <= s.stepMargin) return;
    // The feet's middle goes that far, so a foot twice as far; its step turns
    // the pelvis (pelvisTurnPerStep), which carries the shoulder on: that
    // much less for the hips. The foot: one that gets there in a step (not
    // crossing), then the one whose turn helps the reach (the hand's side:
    // pelvis and trunk turn the same way, Bohannon 2020).
    float gain[2], miss[2];
    math::Vec3 hips[2];
    for (int i = 0; i < 2; ++i) {
        const float z = 2.0f * (mid.z + dir.z * need) - feet[1 - i].z;
        const float turn = -s.pelvisTurnPerStep * (i == 0 ? z - feet[0].z : feet[1].z - z);
        gain[i] = math::dot(math::fromAxisAngle(math::kUp, turn) * shoulder - shoulder, dir);
        hips[i] = mid + dir * (need * need / std::max(need + gain[i], 0.5f * need));
        stepSpot(i, hips[i], math::Vec3(0.0f), hips[i], true, s, miss[i]);
    }
    constexpr float kTie = 0.01f;  // m
    if (steps_[0].running() != steps_[1].running()) reach_.foot = steps_[0].running() ? 0 : 1;
    else if (std::abs(miss[0] - miss[1]) > kTie && std::min(miss[0], miss[1]) < kTie) reach_.foot = miss[0] < miss[1] ? 0 : 1;
    else reach_.foot = gain[0] >= gain[1] ? 0 : 1;
    reach_.on = true;
    reach_.stepped = stepped;
    reach_.hips = hips[reach_.foot];
    math::Vec3 now(0.0f);
    for (int i = 0; i < 2; ++i) {
        now += 0.5f * (t.foot[i].mode == TargetMode::Driven ? t.foot[i].position : anchor_[i]);
        trunkFrom_[i] = trunkTarget_[i];
    }
    reach_.way = math::length(level(reach_.hips - now));
}

float BodyDecider::bendWay(math::Vec3 hips, math::Vec3 shoulder, math::Vec3 target, float reach) const {
    const Swing sw = swingToward(hips, shoulder, target, reach);
    const math::Vec3 axis = level(sw.axis) * sw.angle;  // the bend, about a level axis
    return math::length(axis) > kMinAngle ? std::atan2(-axis.z, axis.x) : 0.0f;
}

float BodyDecider::lowness(int h, math::Vec3 target, float groundY, const BodyDeciderSettings& s) const {
    const float hang = groundY + restShoulder_[h].y - limbReach(h, s);
    return std::clamp((hang - target.y) / std::max(s.facingDrop * (hang - groundY), 1e-3f), 0.0f, 1.0f);
}

float BodyDecider::facingTurn(int h, math::Vec3 target, math::Vec3 mid, float yaw, float groundY,
                              const BodyDeciderSettings& s) const {
    // The least turn that puts the target in front: between the body's midline
    // and its shoulder. Comes in with how far the hand goes below where it
    // hangs: all of it facingDrop of that height down.
    const float down = lowness(h, target, groundY, s);
    const math::Vec3 off = math::fromAxisAngle(math::kUp, -yaw) * level(target - mid);
    const float far = math::length(off);
    if (down <= 0.0f || far < 1e-3f) return 0.0f;
    const float shoulder = restShoulder_[h].x - restPelvis_.position.x;
    const float in = std::clamp(off.x, std::min(0.0f, shoulder), std::max(0.0f, shoulder));
    if (off.z >= 0.0f && off.x == in) return 0.0f;
    const float way = std::atan2(off.x, off.z);  // + to the left
    // Turned, it lies at the nearest edge of that band (ahead of the body).
    const float edge = way * shoulder > 0.0f ? shoulder : 0.0f;
    const float at = std::asin(std::clamp(edge / far, -1.0f, 1.0f));
    const float turn = way - at;
    return turn * way > 0.0f ? turn * down : 0.0f;
}

float BodyDecider::loadShare(const TargetSet& t, int i) {
    const float own = t.foot[i].load, other = t.foot[1 - i].load;
    return 2.0f * (own >= 0.0f ? std::min(own, 1.0f) : other >= 0.0f ? 1.0f - std::min(other, 1.0f) : 0.5f);
}

float BodyDecider::hipTurn(const BodyDeciderSettings& s) const {
    if (!limits_ || joints_.legs[0].start < 0) return s.pelvisMax.y;
    const ik::JointLimit& hip = limits_->limit(joints_.legs[0].start);
    return std::min(-hip.twistMin, hip.twistMax);
}

math::Vec3 BodyDecider::turnedFoot(int f, float turn) const {
    math::Vec3 at = anchor_[1 - f] + math::fromAxisAngle(math::kUp, turn) * math::Vec3(2.0f * hipOffset_[f].x, 0.0f, 0.0f);
    at.y = anchor_[f].y;
    return at;
}

float BodyDecider::shortStanding(const TargetSet& t, const BodyDeciderSettings& s, float turn) {
    Plan& d = desired_;
    query_.pose.locals = d.pose.locals;  // same size: no allocation
    query_.models = d.models;
    queryQueue_.count = 0;
    math::Vec3 anchor[2] = {anchor_[0], anchor_[1]}, reachFrom[2] = {reachFrom_[0], reachFrom_[1]};
    const math::Vec3 ref = hipsRef_;
    const float stance = stanceYaw_;
    const math::Quat rot0 = anchorRot_[0], rot1 = anchorRot_[1];
    const float yaw0 = anchorYaw_[0], yaw1 = anchorYaw_[1];
    if (std::abs(turn - stanceYaw_) > s.pelvisMax.y) {
        const int f = turn > stanceYaw_ ? 0 : 1;
        anchor_[f] = reachFrom_[f] = turnedFoot(f, turn);
        anchorYaw_[f] = turn;
        anchorRot_[f] = restFootLevel(f);
        if (std::abs(turn - anchorYaw_[1 - f]) > hipTurn(s)) {
            anchorYaw_[1 - f] = turn;
            anchorRot_[1 - f] = restFootLevel(1 - f);
        }
        stanceYaw_ = 0.5f * (anchorYaw_[0] + anchorYaw_[1]);
    }
    // Over the feet by their load shares (as supportAndHips places them).
    const float w0 = loadShare(t, 0), w1 = loadShare(t, 1);
    const math::Vec3 mid = (w0 * anchor_[0] + w1 * anchor_[1]) / std::max(w0 + w1, 1e-3f);
    hipsRef_ = math::Vec3(mid.x, ref.y, mid.z);
    const float pelvis = stanceYaw_ + std::clamp(turn - stanceYaw_, s.pelvisMin.y, s.pelvisMax.y);
    placePelvis(d, math::Transform{hipsRef_, math::fromAxisAngle(math::kUp, pelvis) * restPelvis_.rotation});
    math::Vec3 target[2];
    auto left = [&] {
        float most = 0.0f;
        for (int h = 0; h < 2; ++h)
            if (lastHandLeft_[h] >= 0.0f)
                most = std::max(most, math::distance(planPos(d, joints_.arms[h].start), target[h]) - limbReach(h, s));
        return most;
    };
    for (int h = 0; h < 2; ++h) {
        if (lastHandLeft_[h] < 0.0f) continue;
        target[h] = t.hand[h].pathEnd;
        handTrunk(h, target[h], s, queryQueue_);
    }
    balanceStanding(t, s);
    solveLegs(d, t, s, queryQueue_);
    float last = left();
    for (int round = 0; round < 8 && last > 1e-3f; ++round) {
        for (int h = 0; h < 2; ++h)
            if (lastHandLeft_[h] >= 0.0f && trunkFirst_[h] >= 0)
                planTrunk(h, target[h], trunkFirst_[h], trunkEnd_[h], s, queryQueue_);
        balanceStanding(t, s);
        solveLegs(d, t, s, queryQueue_);
        const float now = left();
        if (last - now < 1e-3f) break;
        last = now;
    }
    float missing = left();
    shortTurn_ = 0.0f;
    for (int h = 0; h < 2; ++h) {
        if (lastHandLeft_[h] < 0.0f) continue;
        if (armMiss(h, target[h], s) > missing + 1e-3f) {
            float hint = 0.0f;
            const float more = turnTrunk(h, target[h], s, hint);
            if (std::abs(more) > std::abs(shortTurn_)) shortTurn_ = more;
        }
        missing = std::max(missing, armMiss(h, target[h], s));
    }
    anchor_[0] = anchor[0], anchor_[1] = anchor[1];
    reachFrom_[0] = reachFrom[0], reachFrom_[1] = reachFrom[1];
    anchorRot_[0] = rot0, anchorRot_[1] = rot1;
    anchorYaw_[0] = yaw0, anchorYaw_[1] = yaw1;
    stanceYaw_ = stance;
    hipsRef_ = ref;
    d.pose.locals = query_.pose.locals;
    d.models = query_.models;
    for (int e = 0; e < kEnds; ++e) trunkFirst_[e] = trunkEnd_[e] = -1;
    return missing;
}

void BodyDecider::startStep(int i, math::Vec3 from, float yaw) {
    stepFrom_[i] = from;
    stepYawFrom_[i] = anchorYaw_[i];
    stepYaw_[i] = yaw;
}

void BodyDecider::reachStep(int swing, const BodyDeciderSettings& s, BodyQueue& out) {
    float miss[2];
    if (swing >= 0) {  // under way: its spot for where the hips must end now
        out.step.wanted = true;
        out.step.foot = swing;
        out.step.spot = stepSpot(swing, reach_.hips, math::Vec3(0.0f), reach_.hips, true, s, miss[0], nullptr, nullptr, reach_.turn);
        return;
    }
    if (reach_.there) return;
    // The feet stand where the hips must end; or, a step of the reach taken,
    // the hips can go the rest: the centre of mass stays within the feet and
    // no leg is left behind.
    const math::Vec3 rest = level(reach_.hips - pelvisOf(output_).position);
    const bool between = math::length(level(reach_.hips - 0.5f * (anchor_[0] + anchor_[1]))) <= s.stepMargin;
    const math::Vec3 com = centreOfMass(output_) + rest;
    const bool turned = std::abs(reach_.turn - 0.5f * (anchorYaw_[0] + anchorYaw_[1])) <= s.pelvisMax.y &&
                        std::max(std::abs(reach_.turn - anchorYaw_[0]), std::abs(reach_.turn - anchorYaw_[1])) <= hipTurn(s);
    if (turned && (between || (reach_.stepped && legLeftBehind(rest, s) < 0 &&
                    outsideFeet(math::Vec3(com.x, anchor_[0].y, com.z), anchor_, 2, s) <= 0.0f))) {
        reach_.there = true;
        return;
    }
    // The planned foot; one that a step's reach (or the other foot in its
    // way) leaves shorter than the other: the other one (a way longer than a
    // step: they take turns).
    math::Vec3 spot[2];
    bool moves[2];
    for (int i = 0; i < 2; ++i) {
        spot[i] = stepSpot(i, reach_.hips, math::Vec3(0.0f), reach_.hips, true, s, miss[i], nullptr, nullptr, reach_.turn);
        moves[i] = math::length(level(spot[i] - anchor_[i])) > s.stepMargin ||
                   (!turned && std::abs(reach_.turn - anchorYaw_[i]) > hipTurn(s));
    }
    constexpr float kTie = 0.01f;  // m
    // Still to turn: the foot that is further from it.
    const float off[2] = {std::abs(reach_.turn - anchorYaw_[0]), std::abs(reach_.turn - anchorYaw_[1])};
    int foot = !turned ? (std::abs(off[0] - off[1]) > 1e-3f ? (off[0] > off[1] ? 0 : 1) : reach_.foot)
               : std::abs(miss[0] - miss[1]) > kTie ? (miss[0] < miss[1] ? 0 : 1) : reach_.foot;
    if (!moves[foot]) foot = 1 - foot;
    if (!moves[foot]) return;
    out.step.wanted = true;
    out.step.foot = foot;
    out.step.spot = spot[foot];
    pathSteps_[foot] = 1;
}

int BodyDecider::legLeftBehind(math::Vec3 way, const BodyDeciderSettings& s) const {
    const math::Transform pelvis = pelvisOf(output_);
    int behind = -1;
    float worst = 0.0f;
    for (int i = 0; i < 2; ++i) {
        const math::Vec3 hip = pelvis.position + way + pelvis.rotation * hipOffset_[i];
        const math::Vec3 restHip = restPelvis_.position + restPelvis_.rotation * hipOffset_[i];
        // The leg's height allowed: reachDrop below standing, or where the
        // plan holds the hips if lower (a crouch on purpose, a reach's
        // bend: the leg is not at its reach there, no reason to step).
        const float up = std::min(restHip.y - restFootY_[i] - s.reachDrop, hip.y - anchor_[i].y);
        const float reach = limbReach(2 + i, s);
        const float flatMax = up < reach ? std::sqrt(reach * reach - up * up) : 0.0f;
        const float dx = hip.x - anchor_[i].x, dz = hip.z - anchor_[i].z;
        const float over = std::sqrt(dx * dx + dz * dz) - flatMax;
        if (over > worst) {
            worst = over;
            behind = i;
        }
    }
    return behind;
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

void BodyDecider::balanceStanding(const TargetSet& t, const BodyDeciderSettings& s) {
    // Standing (both feet grounded, none on its way, the hips not driven):
    // the hips go where the plan's centre of mass is over the support (a
    // bend takes the hips back: anticipatory posture, Massion 1992). On the
    // way through a step the hips follow the support reference itself
    // (walking is not statically balanced: the body vaults over the stance
    // leg on its own momentum).
    // A foot held up (driven, no step's timing) is standing too, on the other
    // foot: the centre of mass over it, so a leg going forward takes the hips
    // back as it goes (its path's progress, each tick) and one going back
    // takes them forward. A step on its way is walking (above).
    Plan& d = desired_;
    bool stepping = false;
    for (int i = 0; i < 2; ++i)
        if (t.foot[i].mode == TargetMode::Driven && t.foot[i].swing >= 0.0f) stepping = true;
    const bool standing = !stepping && (grounded_[0] || grounded_[1]) && act_[HipsW] <= 0.0f && totalMass_ > 0.0f;
    if (!standing) return;
    // On both feet the centre of mass may go anywhere over them (a reach
    // leans the body out to the toes, the hips staying: Moriyama 2022's
    // functional reach, the pressure 0.05 H forward, the pelvis +-0.01 H):
    // the hips go back only as far as keeps it within the feet. On one foot
    // it stays over that foot's ankle (its outline is narrow).
    const bool bothFeet = grounded_[0] && grounded_[1];
    constexpr int kPasses = 3;
    constexpr float kOver = 1e-3f;  // m: the centre of mass over the support
    for (int pass = 0; pass < kPasses; ++pass) {
        const math::Vec3 com = planCentreOfMass();
        math::Vec3 off(hipsRef_.x - com.x, 0.0f, hipsRef_.z - com.z);
        if (bothFeet) {
            const float out = outsideFeet(math::Vec3(com.x, anchor_[0].y, com.z), anchor_, 2, s);
            if (out <= kOver) break;
            off = math::safeNormalize(off) * (out + kOver);  // back toward the support's centre, to its edge
        }
        if (math::length(off) <= kOver) break;
        movePelvis(d, off);  // the upper body moves with the hips; the next pass measures again
    }
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
            // A foot held up (no step's timing): its knee points the way the
            // foot goes, the hip turning the thigh out or in (within its twist
            // range), so a foot out to the side lifts with the knee out. A
            // foot right under the hip or behind it keeps the knee forward (it
            // bends one way): the foot's way counts from kneeForwardBias of
            // the leg ahead. A step's swing (walking, running, a side step)
            // keeps the knee forward. As far as the thigh stays within the
            // hip's abduction range; a foot further out than the hip and the
            // pelvis's hike (footHike) reach is held at that range, short of
            // its target (reported).
            const math::Quat pelvisRot = planRot(p, joints_.pelvis);
            const math::Quat turned = pelvisRot * math::inverse(restPelvis_.rotation);  // from standing
            const float side = i == 0 ? 1.0f : -1.0f;  // +X is the left leg's outside
            auto poleAt = [&](float a) {
                return math::fromAxisAngle(turned * math::kUp, side * a) * (pelvisRot * s.kneePole);
            };
            float turn = 0.0f;
            if (limits_ && joints_.legs[0].start >= 0) {
                const ik::JointLimit& hipLimit = limits_->limit(joints_.legs[0].start);  // left: + external, + abduction
                // Held at the range: the target's way out to the side cut
                // (its height and fore-aft kept) until the thigh, knee
                // forward, is within it.
                auto inRange = [&](math::Vec3 at) {
                    const float abd = thighAbduction(i, pelvisRot, hip, kneeToward(i, hip, at, poleAt(0.0f)));
                    return abd >= hipLimit.swing2Min - 1e-3f && abd <= hipLimit.swing2Max + 1e-3f;
                };
                if (!inRange(target)) {
                    const math::Vec3 v = math::inverse(turned) * (target - hip);
                    auto cut = [&](float share) {
                        return hip + turned * math::Vec3(v.x * share, v.y, v.z);
                    };
                    float lo = 0.0f, hi = 1.0f;
                    constexpr int kHalvings = 8;
                    for (int n = 0; n < kHalvings; ++n) {
                        const float mid = 0.5f * (lo + hi);
                        (inRange(cut(mid)) ? lo : hi) = mid;
                    }
                    target = cut(lo);
                }
                if (t.foot[i].swing < 0.0f) {
                    const math::Vec3 v = math::inverse(turned) * (target - hip);
                    turn = std::clamp(std::atan2(side * v.x, std::max(v.z, 0.0f) + s.kneeForwardBias * legLength_[i]),
                                      hipLimit.twistMin, hipLimit.twistMax);
                    auto within = [&](float a) {
                        const float abd = thighAbduction(i, pelvisRot, hip, kneeToward(i, hip, target, poleAt(a)));
                        return abd >= hipLimit.swing2Min - 1e-3f && abd <= hipLimit.swing2Max + 1e-3f;
                    };
                    if (!within(turn)) {
                        // The largest turn (from forward toward it) that keeps it.
                        float lo = 0.0f, hi = turn;
                        constexpr int kHalvings = 8;
                        for (int n = 0; n < kHalvings; ++n) {
                            const float mid = 0.5f * (lo + hi);
                            (within(mid) ? lo : hi) = mid;
                        }
                        turn = lo;
                    }
                }
            }
            pole = poleAt(turn);
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
            if (grounded_[i]) refresh(p, leg.start);
            else clampLimb(p, 2 + i, chain, 2, target);
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
    // (no fixed direction, so none the arm can line up with); out of the
    // shoulder's range, the elbow goes round as little as brings it in
    // (fitPole). Free hands
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
        const math::Vec3 restWorld = chestPos + chest * restHandChest_[h];
        for (int j : {arm.start, arm.mid})
            p.pose.locals[static_cast<size_t>(j)].rotation = rest.locals[static_cast<size_t>(j)].rotation;
        refresh(p, arm.start);
        const math::Vec3 from = planPos(p, arm.start);
        const float reach = limbReach(h, s);
        const math::Vec3 target = clampToReach(from, math::lerp(restWorld, hand.position, k), reach);
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

math::Vec3 BodyDecider::stepSpot(int i, math::Vec3 xcom, math::Vec3 velocity, math::Vec3 hipsEnd, bool hipsGoal,
                                 const BodyDeciderSettings& s, float& miss, math::Vec3* idealOut,
                                 bool* beyondReach, float yaw) const {
    if (beyondReach) *beyondReach = false;
    // The heel (the ankle over it: the foot lands heel first, its pressure
    // there) where the capture point will be, landingBehind short of it along
    // the motion, under its own hip to the side.
    const math::Vec3 dir = math::safeNormalize(math::Vec3(velocity.x, 0.0f, velocity.z));
    math::Vec3 ideal = xcom - dir * s.landingBehind + math::Vec3(hipOffset_[i].x, 0.0f, 0.0f);
    const int o = 1 - i;
    const bool otherDown = grounded_[o] && joints_.legs[o].end >= 0;
    if (otherDown && hipsGoal) {
        // The hips stopping there: the foot lands so they end between the
        // feet (the other foot mirrored about them: a normal stance), by how
        // slowly they get there (moving on: the capture point's). Only where
        // a hips target sets that place: free hips settle between the feet,
        // so their place would follow the step's own spot (a loop: the
        // spot crept on with the reach's lean, measured 2026-11-15).
        const float speed = std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        const float stop = 1.0f - std::clamp(speed / std::max(s.stopBlendSpeed, 1e-3f), 0.0f, 1.0f);
        ideal.x = math::lerp(ideal.x, 2.0f * hipsEnd.x - anchor_[o].x, stop);
        ideal.z = math::lerp(ideal.z, 2.0f * hipsEnd.z - anchor_[o].z, stop);
    }
    ideal.y = anchor_[i].y;
    if (idealOut) *idealOut = ideal;
    math::Vec3 spot = ideal;
    if (otherDown) {
        // Never across the other foot, never further from it than a step
        // reaches (an ellipse: further fore-aft than across).
        const math::Vec3 other = anchor_[o];
        const float side = hipOffset_[i].x >= hipOffset_[o].x ? 1.0f : -1.0f;
        const math::Quat turned = math::fromAxisAngle(math::kUp, yaw);  // the body's way there
        math::Vec3 d = math::inverse(turned) * level(spot - other);
        if (d.x * side < s.minFootGap) d.x = side * s.minFootGap;
        const float ex = d.x / std::max(s.maxStepSide, 1e-3f), ez = d.z / std::max(s.maxStepForward, 1e-3f);
        const float over = std::sqrt(ex * ex + ez * ez);
        if (over > 1.0f) {
            d /= over;
            if (beyondReach) *beyondReach = true;
        }
        d = turned * d;
        spot.x = other.x + d.x;
        spot.z = other.z + d.z;
    }
    miss = std::sqrt((spot.x - ideal.x) * (spot.x - ideal.x) + (spot.z - ideal.z) * (spot.z - ideal.z));
    return spot;
}

void BodyDecider::balanceStep(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out) {
    // The extrapolated centre of mass (Hof, Gazendam & Sinke 2005): where
    // the body's motion takes it, XCoM = CoM + v / w0, w0 = sqrt(g / l) (the
    // inverted pendulum of the leg). Out of the support: a step, the foot
    // landing where the XCoM will be by then (the motion carries it on over
    // the step's time). General: whatever moves the body asks for it.
    out.step = StepAdvice{};
    // The foot that stepped last: a timed swing that has landed.
    for (int i = 0; i < 2; ++i) {
        const bool swinging = t.foot[i].mode == TargetMode::Driven && t.foot[i].swing >= 0.0f;
        if (wasSwinging_[i] && t.foot[i].mode == TargetMode::Pinned) lastStepped_ = i;
        wasSwinging_[i] = swinging;
    }
    if (!s.balanceStepOn || s.gravity <= 0.0f || totalMass_ <= 0.0f) return;
    int swing = -1;
    for (int i = 0; i < 2; ++i) {
        const Target& f = t.foot[i];
        if (f.mode != TargetMode::Driven) continue;
        if (f.swing < 0.0f) return;  // a foot held up (no step): no balance step
        swing = i;
    }
    math::Vec3 standing[2];
    float groundY = 0.0f;
    int n = 0;
    for (int i = 0; i < 2; ++i)
        if (i != swing && grounded_[i] && joints_.legs[i].end >= 0) {
            standing[n++] = anchor_[i];
            groundY += anchor_[i].y;
        }
    if (n == 0) return;  // no support (flight)
    groundY /= static_cast<float>(n);
    const math::Vec3 com = centreOfMass(output_);
    const float w0 = pendulumRate(groundY, s);
    const math::Vec3 v(hipsVel_.x, 0.0f, hipsVel_.z);  // the plan's hips carry the body
    const math::Vec3 xcom = math::Vec3(com.x, groundY, com.z) + v / w0;
    // Where it will be in `time`: a driven hips target is where the hips go
    // (the plan lags it only within the muscles' bounds), so by its share
    // the body goes to the target and on at the target's own speed. Free
    // hips settle over the support (between the feet: no motion of their
    // own), so only the drive carries the body on, never the plan's speed
    // now (that stops at its reference).
    // A driven hips target on a path (the sync formula) goes to the path's
    // end and stops there: the body goes there, with no speed on (planned
    // from the whole path, not from the drive still blending in).
    const Target& hipsGoal = given(t, HipsW);
    const float kh = act_[HipsW] * std::clamp(hipsGoal.weightPosition, 0.0f, 1.0f);
    const bool onPath = kh > 0.0f && hipsGoal.pathLeft >= 0.0f;
    math::Vec3 lag = onPath ? drivenPart(hipsGoal, hipsGoal.pathEnd - pelvisOf(output_).position) *
                                  std::clamp(hipsGoal.weightPosition, 0.0f, 1.0f)
                            : drivenPart(hipsGoal, hipsGoal.position - pelvisOf(output_).position) * kh;
    lag.y = 0.0f;
    math::Vec3 vGoal = onPath ? math::Vec3(0.0f) : drivenPart(hipsGoal, hipsGoal.velocity) * kh;
    vGoal.y = 0.0f;
    // The centre of mass (on the ground) `time` from now, and its capture point then.
    auto hipsAt = [&](float time) { return math::Vec3(com.x, groundY, com.z) + lag + vGoal * time; };
    auto future = [&](float time) { return hipsAt(time) + vGoal / w0; };
    out.step.capturePoint = xcom;
    out.step.velocity = vGoal;
    out.step.outside = outsideFeet(xcom, standing, n, s);
    if (swing < 0) timeStep(s);  // the next step's shape (a step under way keeps its own)
    // The weight shift and the swing: a step under way's own; one not yet
    // decided, the least shift (the foot's unloading) and the foot's aimed
    // move (Fitts' law) to where the capture point is now.
    float lead = std::max(s.supportBlendTime, 0.0f), swingFor = 0.0f;
    if (swing >= 0) {
        const StepPlannerSettings& timing = stepNow_[swing];
        const float lift = std::clamp(timing.liftAt, 0.0f, 0.9f);
        lead = timing.duration * lift;
        swingFor = timing.duration * (1.0f - lift);
    } else {
        math::Vec3 centre(0.0f);
        for (int k = 0; k < n; ++k) centre += standing[k];
        centre /= static_cast<float>(n);
        swingFor = math::fittsTime(math::length(level(xcom - centre)), s.fittsWidth, s.fittsA, s.fittsB);
    }
    if (reach_.on) {  // a hand further than standing: its step, planned on its path
        reachStep(swing, s, out);
        return;
    }
    float miss = 0.0f;
    if (swing >= 0) {
        // A step under way: its spot follows the capture point to the landing.
        const float u = std::clamp(t.foot[swing].swing, 0.0f, 1.0f);
        const float left = swingFor * (1.0f - u) + (u <= 0.0f ? lead : 0.0f);
        out.step.wanted = true;
        out.step.foot = swing;
        out.step.spot = stepSpot(swing, future(left), vGoal, hipsAt(left), kh > 0.0f, s, miss);
        return;
    }
    // A step takes its time: it starts once the capture point will be out of
    // the support by the time a step could land (standing still: once it is
    // out), so a moving body steps on without a pause.
    const math::Vec3 at = future(lead + swingFor), hipsEnd = hipsAt(lead + swingFor);
    // On the hips' path the step lands as the hips arrive; a way longer than
    // a step reaches (the ideal spot past the reached one) is shared by more
    // steps, each landing planning the next.
    auto timed = [&](int foot, math::Vec3 spot, math::Vec3 ideal, bool beyondReach) {
        const float need = std::sqrt((ideal.x - anchor_[foot].x) * (ideal.x - anchor_[foot].x) +
                                     (ideal.z - anchor_[foot].z) * (ideal.z - anchor_[foot].z));
        const float got = std::sqrt((spot.x - anchor_[foot].x) * (spot.x - anchor_[foot].x) +
                                    (spot.z - anchor_[foot].z) * (spot.z - anchor_[foot].z));
        // More steps only when this one, cut short by its reach, still leaves
        // the centre of mass out of the support (a wide side step may catch
        // it although the hips do not end between the feet).
        const math::Vec3 after[2] = {spot, anchor_[1 - foot]};
        const bool caught = outsideFeet(at, after, 2, s) <= s.stepMargin;
        const int steps = beyondReach && !caught && got > 1e-3f
                              ? std::clamp(static_cast<int>(std::ceil(need / got - 1e-3f)), 1, std::max(s.mostSteps, 1))
                              : 1;
        out.step.wanted = true;
        out.step.foot = foot;
        out.step.spot = spot;
        out.step.time = pathStepTime(t, foot, anchor_[foot], spot, steps, s);
        pathSteps_[foot] = steps;
    };
    // A foot left behind: by the time a step could land the hips will be so
    // far from it that its leg, at its reach, would hold them more than
    // reachDrop below standing; it steps (a leg's reach ends its stance:
    // the trailing leg's toe-off), under where the capture point will be.
    // General: whatever moves the hips away from a foot.
    const int behind = n == 2 && s.reachDrop > 0.0f ? legLeftBehind(lag + vGoal * (lead + swingFor), s) : -1;
    // A step that would put the foot back where it stands (within the
    // margin) changes nothing: no step (it would be taken again and again).
    auto moves = [&](int foot, math::Vec3 spot) {
        const float dx = spot.x - anchor_[foot].x, dz = spot.z - anchor_[foot].z;
        return std::sqrt(dx * dx + dz * dz) > s.stepMargin;
    };
    if (outsideFeet(at, standing, n, s) <= s.stepMargin) {
        if (behind < 0) return;
        math::Vec3 ideal;
        bool beyond = false;
        const math::Vec3 spot = stepSpot(behind, at, vGoal, hipsEnd, kh > 0.0f, s, miss, &ideal, &beyond);
        if (!moves(behind, spot)) return;
        timed(behind, spot, ideal, beyond);
        return;
    }
    // The foot whose step leaves the capture point (where it will be by the
    // landing) best inside the new support (not crossing, within a step); a
    // tie, or neither reaching it (each step makes room for the next): the
    // feet take turns (gait alternates), the first step the one with further
    // to go.
    math::Vec3 spot[2], ideal[2];
    bool beyond[2] = {false, false};
    float after[2], travel[2];
    for (int i = 0; i < 2; ++i) {
        spot[i] = stepSpot(i, at, vGoal, hipsEnd, kh > 0.0f, s, miss, &ideal[i], &beyond[i]);
        const math::Vec3 feet[2] = {spot[i], anchor_[1 - i]};
        after[i] = outsideFeet(at, feet, 2, s);
        const math::Vec3 d = spot[i] - anchor_[i];
        travel[i] = std::sqrt(d.x * d.x + d.z * d.z);
    }
    constexpr float kTie = 0.01f;  // m
    int foot;
    const bool neither = after[0] > s.stepMargin && after[1] > s.stepMargin;
    if (std::abs(after[0] - after[1]) > kTie && !(neither && lastStepped_ >= 0)) foot = after[0] < after[1] ? 0 : 1;
    else if (lastStepped_ >= 0) foot = 1 - lastStepped_;
    else foot = travel[0] >= travel[1] ? 0 : 1;
    if (!moves(foot, spot[foot])) return;
    timed(foot, spot[foot], ideal[foot], beyond[foot]);
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

float BodyDecider::armMiss(int h, math::Vec3 target, const BodyDeciderSettings& s) const {
    const Plan& d = desired_;
    const math::Vec3 shoulder = planPos(d, joints_.arms[h].start);
    float miss = std::max(0.0f, math::distance(shoulder, target) - limbReach(h, s));
    if (limits_) fitPole(armShape(h, d, s), armHang(h, d), armFrame(h, d), shoulder, target, miss);
    return miss;
}

float BodyDecider::turnTrunk(int h, math::Vec3 target, const BodyDeciderSettings& s, float& hint) {
    if (!limits_ || joints_.spine[2] < 0) return 0.0f;
    Plan& d = desired_;
    const rig::Pose& rest = rig_->restPose();
    // Turning toward a point is mostly thoracic, the pelvis last (BIOMECHANICS
    // 9.3); about the trunk's own long axis.
    const int joint[4] = {joints_.spine[2], joints_.spine[1], joints_.spine[0], joints_.pelvis};
    // Behind the shoulder to the hand's own side (a right hand: to the right);
    // ahead toward the target's side.
    const math::Vec3 way = math::inverse(planRot(d, joint[0])) * (target - planPos(d, joints_.arms[h].start));
    const float sign = way.z < 0.0f ? (h == 0 ? 1.0f : -1.0f) : way.x >= 0.0f ? 1.0f : -1.0f;
    float missing = 0.0f;
    constexpr int kRounds = 2;  // the turned shoulder is elsewhere: once more from there
    for (int round = 0; round < kRounds; ++round) {
        const math::Vec3 axis = planRot(d, joint[0]) * (math::kUp * sign);
        const float turn = turnToReach(armShape(h, d, s), armHang(h, d), armFrame(h, d), planPos(d, joints_.arms[h].start),
                                       target, planPos(d, joint[0]), axis, round == 0 ? hint : 0.0f);
        if (round == 0) hint = turn;
        missing = turn;
        if (turn <= kMinAngle) return 0.0f;
        // Each one's room, then its share of the whole turn by place; what one
        // at its range cannot take goes to the others in the same shares.
        float room[4], give[4] = {0.0f, 0.0f, 0.0f, 0.0f}, weight[4], w = 1.0f;
        for (int k = 0; k < 4; ++k) {
            weight[k] = w;
            w *= s.queueShare;
            room[k] = 0.0f;
            if (joint[k] < 0) continue;
            if (k == 3) {
                const float now = math::rotationVector(math::normalize(pelvisOf(d).rotation * math::inverse(pelvisBase()))).y;
                room[k] = sign > 0.0f ? s.pelvisMax.y - now : now - s.pelvisMin.y;
                weight[k] *= 1.0f - act_[HipsW];
            } else {
                const ik::JointLimit l = limits_->limitIn(joint[k], d.pose);
                const size_t j = static_cast<size_t>(joint[k]);
                const float now = math::rotationVector(math::normalize(d.pose.locals[j].rotation * math::inverse(rest.locals[j].rotation))).y;
                room[k] = sign > 0.0f ? l.twistMax - now : now - l.twistMin;
            }
            room[k] = std::max(0.0f, room[k]);
        }
        // The spine first (each joint its share by place; what one at its
        // range cannot take, the others), the pelvis only what the spine at
        // its range still cannot give.
        float left = turn;
        for (int pass = 0; pass < 8 && left > kMinAngle; ++pass) {
            const bool pelvis = pass >= 4;
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k)
                if ((k == 3) == pelvis && room[k] - give[k] > kMinAngle) sum += weight[k];
            if (sum <= 1e-6f) {
                if (pelvis) break;
                pass = 3;
                continue;
            }
            const float now = left;
            for (int k = 0; k < 4; ++k) {
                if ((k == 3) != pelvis || room[k] - give[k] <= kMinAngle) continue;
                const float take = std::min(now * weight[k] / sum, room[k] - give[k]);
                give[k] += take;
                left -= take;
            }
        }
        missing = std::max(0.0f, left);
        if (give[3] > 0.0f) {
            math::Transform p = pelvisOf(d);
            const math::Vec3 hips = p.position + p.rotation * hipCentre_;
            const math::Quat by = math::fromAxisAngle(math::kUp, sign * give[3]);
            p.rotation = math::normalize(by * p.rotation);
            p.position = hips + by * (p.position - hips);
            placePelvis(d, p);
        }
        for (int k = 2; k >= 0; --k)
            if (give[k] > 0.0f) turnPlanned(d, joint[k], math::fromAxisAngle(planRot(d, joint[k]) * math::kUp, sign * give[k]));
        if (missing > kMinAngle) break;  // at their ranges
    }
    return sign * missing;
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

math::Vec3 BodyDecider::kneeToward(int i, math::Vec3 hip, math::Vec3 target, math::Vec3 pole) const {
    // The two bones from the hip to the target: the knee on the circle where
    // they meet, on the pole's side of the hip-target line.
    const float a = thighLength_[i], b = shinLength_[i];
    const math::Vec3 axis = math::safeNormalize(target - hip, -math::kUp);
    const float d = std::clamp(math::distance(hip, target), 1e-4f, a + b - 1e-4f);
    const float x = (a * a - b * b + d * d) / (2.0f * d);
    const float r = std::sqrt(std::max(a * a - x * x, 0.0f));
    const math::Vec3 perp = math::safeNormalize(pole - axis * math::dot(pole, axis), math::kForward);
    return hip + axis * x + perp * r;
}

float BodyDecider::thighAbduction(int i, math::Quat pelvisRot, math::Vec3 hip, math::Vec3 knee) const {
    // The thigh's angle out of the pelvis's forward-up plane to its own side
    // (the pelvis's turn from standing taken out; character axes: +X left):
    // a thigh lifted forward has none, one swung out to the side all of it.
    const math::Vec3 v = math::inverse(pelvisRot * math::inverse(restPelvis_.rotation)) * (knee - hip);
    const float side = i == 0 ? 1.0f : -1.0f;  // +X is the left leg's outside
    return std::asin(std::clamp(side * math::safeNormalize(v, -math::kUp).x, -1.0f, 1.0f));
}

void BodyDecider::footHike(int i, math::Vec3 target, const BodyDeciderSettings& s) {
    // A lifted foot out to the side further than the hip abducts (its range):
    // the pelvis hikes on that side, about the standing hip, by what the hip
    // cannot give, within the pelvis's range (pelvisMin / pelvisMax).
    const ik::TwoBoneChain& leg = joints_.legs[i];
    if (!limits_ || leg.start < 0 || joints_.legs[0].start < 0 || joints_.legs[1 - i].start < 0) return;
    Plan& d = desired_;
    math::Transform p = pelvisOf(d);
    const float side = i == 0 ? 1.0f : -1.0f;  // +X is the left leg's outside
    // The thigh with the knee forward (the least abduction a lift asks for).
    const math::Vec3 hip = planPos(d, leg.start);
    const math::Vec3 reached = clampToReach(hip, target, limbReach(2 + i, s));
    const float abduction =
        thighAbduction(i, p.rotation, hip, kneeToward(i, hip, reached, p.rotation * s.kneePole));
    const float excess = abduction - limits_->limit(joints_.legs[0].start).swing2Max;  // left: + abduction
    if (excess <= 0.0f) return;
    const math::Vec3 pivot = planPos(d, joints_.legs[1 - i].start);
    const math::Vec3 forward = p.rotation * (math::inverse(restPelvis_.rotation) * math::kForward);
    const math::Quat want = math::normalize(math::fromAxisAngle(forward, side * excess) * p.rotation);
    const math::Quat rel =
        clampRotationVector(math::normalize(want * math::inverse(pelvisBase())), s.pelvisMin, s.pelvisMax);
    const math::Quat turn = math::normalize(rel * pelvisBase() * math::inverse(p.rotation));
    p.rotation = math::normalize(turn * p.rotation);
    p.position = pivot + turn * (p.position - pivot);
    placePelvis(d, p);
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
    // The range counts in the hips' own frame (turned with the feet).
    const math::Quat base = pelvisBase(), yawed = math::fromAxisAngle(math::kUp, stanceYaw_);
    float deep = 0.0f;  // how far down a hand goes
    if (e < 2) {
        const float groundY = 0.5f * (anchor_[0].y + anchor_[1].y - restFootY_[0] - restFootY_[1]);
        deep = lowness(e, target, groundY, s);
    }
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
                math::Transform p = pelvisOf(d);
                const math::Quat now = math::normalize(math::inverse(yawed) * p.rotation * math::inverse(base) * yawed);
                const Swing sw = e < 2 ? blendSwing(deep, hips, shoulder, target, reach, yawed, math::rotationVector(now), s.pelvisMin, s.pelvisMax)
                                       : swingToward(hips, shoulder, target, reach);
                const math::Quat want = math::normalize(math::fromAxisAngle(sw.axis, sw.angle * o.share) * p.rotation);
                const math::Quat own = math::normalize(math::inverse(yawed) * want * math::inverse(base) * yawed);
                const math::Quat rel = math::normalize(yawed * clampRotationVector(own, s.pelvisMin, s.pelvisMax) * math::inverse(yawed));
                const math::Quat turn = math::normalize(rel * base * math::inverse(p.rotation));
                p.rotation = math::normalize(turn * p.rotation);
                p.position = hips + turn * (p.position - hips);
                placePelvis(d, p);
            } else if (o.joint == joints_.spine[0] || o.joint == joints_.spine[1] || o.joint == joints_.spine[2] ||
                       o.joint == joints_.clavicles[0] || o.joint == joints_.clavicles[1]) {
                // A spine joint or the shoulder girdle: it turns at its own
                // joint, within its range.
                Swing sw;
                if (e < 2 && limits_ && o.joint != joints_.clavicles[0] && o.joint != joints_.clavicles[1]) {
                    const ik::JointLimit l = limits_->limitIn(o.joint, d.pose);
                    const math::Quat local = d.pose.locals[static_cast<size_t>(o.joint)].rotation;
                    const math::Quat rest = rig_->restPose().locals[static_cast<size_t>(o.joint)].rotation;
                    sw = blendSwing(deep, planPos(d, o.joint), shoulder, target, reach, planRot(d, rig_->parent(o.joint)),
                                      math::rotationVector(math::normalize(local * math::inverse(rest))),
                                      math::Vec3(l.swing1Min, l.twistMin, l.swing2Min), math::Vec3(l.swing1Max, l.twistMax, l.swing2Max));
                } else {
                    sw = swingToward(planPos(d, o.joint), shoulder, target, reach);
                }
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
