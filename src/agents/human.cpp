#include "agents/human.h"

#include "core/config.h"
#include "core/log.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace agents {

namespace {

math::Vec3 modelPos(const std::vector<math::Mat4>& models, int joint) {
    return math::Vec3(models[static_cast<size_t>(joint)][3]);
}

math::Quat modelRot(const std::vector<math::Mat4>& models, int joint) {
    return math::toTransform(models[static_cast<size_t>(joint)]).rotation;
}

// --- The old formula repositioning (see RepositionSettings in human.h) ---

// Moves `pelvis` so this leg's hip is within [lo, hi] of its foot. The top
// eases in over `band` (smooth, no kink as the leg nears straight).
math::Vec3 clampLeg(math::Vec3 pelvis, const LegSupport& leg, float lo, float hi, float band) {
    math::Vec3 d = pelvis + leg.hipOffset - leg.foot;
    float len = math::length(d);
    float clamped = std::max(len, lo);
    const float knee = hi - band;
    if (band > 0.0f && clamped > knee) clamped = hi - band * std::exp(-(clamped - knee) / band);
    else clamped = std::min(clamped, hi);
    if (std::fabs(clamped - len) < 1e-5f) return pelvis;
    return pelvis + math::safeNormalize(d, math::kUp) * (clamped - len);
}

// One semi-implicit step of a damped spring toward `goal` (frequency w, ratio z).
template <typename T>
void spring(T& x, T& v, T goal, float w, float z, float dt) {
    v += (w * w * (goal - x) - 2.0f * z * w * v) * dt;
    x += v * dt;
}

// Knee-limited reach: hip-to-ankle distance with the knee bent `kneeFlex`
// radians (law of cosines over thigh and shin).
float bentReach(float thigh, float shin, float kneeFlex) {
    return std::sqrt(std::max(thigh * thigh + shin * shin + 2.0f * thigh * shin * std::cos(kneeFlex), 0.0f));
}

// The leg clamp: moves `pelvis` so each hip is within reach of its foot:
// `first` up to maxReach, `second` within [minReach, maxReach] (x legLength,
// soft top), `first` then `second`, s.iterations times.
math::Vec3 clampToLegs(math::Vec3 pelvis, const LegSupport& first, const LegSupport& second, float legLength,
                       const RepositionSettings& s) {
    const float lo = s.minReach * legLength, hi = s.maxReach * legLength, band = s.softBand * legLength;
    for (int i = 0; i < s.iterations; ++i) {
        pelvis = clampLeg(pelvis, first, 0.0f, hi, band);
        pelvis = clampLeg(pelvis, second, lo, hi, band);
    }
    return pelvis;
}

// Advances `body` one tick toward the feet. `stanceWeight` in [0, 1]: 0 = left
// leg carries the body, 1 = right leg. All points in the same space, +Y up.
void moveBodyToFeet(BodyState& body, const LegSupport& left, const LegSupport& right, float legLength,
                    float stanceWeight, const RepositionSettings& s, float dt) {
    const float w = math::clamp(stanceWeight, 0.0f, 1.0f);
    const float single = std::fabs(2.0f * w - 1.0f);  // 0 both feet loaded, 1 one foot

    // Where: between the feet by the weight each carries.
    auto stand = [&](const LegSupport& leg) { return leg.foot + math::kUp * (s.standReach * legLength) - leg.hipOffset; };
    const math::Vec3 overL = stand(left), overR = stand(right);
    math::Vec3 goal = math::lerp(overL, overR, w);
    goal.x = math::lerp(0.5f * (overL.x + overR.x), goal.x, s.sideFollow);
    goal.z = math::lerp(s.restZ, goal.z, s.forwardFollow);

    // How high: as high as every loaded leg reaches straight (standReach);
    // the lowest one decides. A lifted foot sets nothing.
    const float reach = s.standReach * legLength;
    auto hipHeight = [&](const LegSupport& leg) {
        const math::Vec3 hip = goal + leg.hipOffset;
        const math::Vec2 out(hip.x - leg.foot.x, hip.z - leg.foot.z);
        return leg.foot.y + std::sqrt(std::max(reach * reach - math::dot(out, out), 0.0f)) - leg.hipOffset.y;
    };
    const float yL = hipHeight(left), yR = hipHeight(right);
    const float both = std::min(yL, yR), one = w >= 0.5f ? yR : yL;
    goal.y = math::lerp(goal.y, math::lerp(both, one, single), s.standShare);
    body.goal = goal;

    // On one foot the lifted side drops (BIOMECHANICS 4.3).
    spring(body.drop, body.dropRate, -s.standShare * s.stanceDrop * (2.0f * w - 1.0f), s.followRate, s.damping, dt);
    const math::Quat tilt = body.rotation();
    const LegSupport l{tilt * left.hipOffset, left.foot}, r{tilt * right.hipOffset, right.foot};
    spring(body.position, body.velocity, goal, s.followRate, s.damping, dt);

    // Rule 1: the legs reach. The unloaded leg only limits the reach (a lifted
    // foot must not push the body away; its knee bends more); stance leg last.
    const math::Vec3 sprung = body.position;
    body.position = clampToLegs(sprung, w >= 0.5f ? l : r, w >= 0.5f ? r : l, legLength, s);
    // The clamp stops the spring: drop the velocity pushing into it, so the
    // body does not jump once the limit lets go.
    const math::Vec3 push = body.position - sprung;
    const float pushLength = math::length(push);
    if (pushLength > 1e-6f) {
        const math::Vec3 n = push / pushLength;
        const float into = math::dot(body.velocity, n);
        if (into < 0.0f) body.velocity -= n * into;
    }
}

// Fitts' law movement time (s) to move `distance` onto a target `width`
// wide: a + b * log2(2D / W), never below a (MacKenzie 1992; BIOMECHANICS 9.6).
float movementTime(float distance, float width, float a, float b) {
    const float id = std::log2(std::max(2.0f * distance / std::max(width, 1e-3f), 1.0f));
    return a + b * id;
}

} // namespace

bool Human::init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion) {
    def_ = def;
    jointsConfig_ = &joints;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);
    previousRoot_ = root_;
    mover_.reset(def.spawnYaw);
    legPhase_ = armAmount_ = headYaw_ = 0.0f;
    loadLocomotion(locomotion);

    ik_.kneePole = joints.getVec3("ik.kneePole", ik_.kneePole);
    ik_.elbowPole = joints.getVec3("ik.elbowPole", ik_.elbowPole);
    ik_.soften = joints.getFloat("ik.soften", ik_.soften);
    ik_.chestAimWeight = joints.getFloat("ik.chestAimWeight", ik_.chestAimWeight);
    ik_.headAimWeight = joints.getFloat("ik.headAimWeight", ik_.headAimWeight);
    reposition_.standReach = joints.getFloat("reposition.standReach", reposition_.standReach);
    reposition_.maxReach = joints.getFloat("reposition.maxReach", reposition_.maxReach);
    reposition_.followRate = joints.getFloat("reposition.followRate", reposition_.followRate);
    reposition_.softBand = joints.getFloat("reposition.softBand", reposition_.softBand);
    reposition_.damping = joints.getFloat("reposition.damping", reposition_.damping);
    reposition_.stanceDrop = math::radians(joints.getFloat("reposition.stanceDropDeg", math::degrees(reposition_.stanceDrop)));
    stanceBlendHeight_ = joints.getFloat("reposition.stanceBlendHeight", stanceBlendHeight_);
    return setHeight(height);
}

void Human::loadLocomotion(const core::Config& c) {
    // Load-time only (builds path strings). Walking is the only pace (jog and
    // run removed, the user's call 2026-11-09).
    auto load = [&](const std::string& block, motion::MoveSettings& w) {
        auto get = [&](const char* key, float v) { return c.getFloat(block + "." + key, v); };
        auto deg = [&](const char* key, float radians) { return math::radians(get(key, math::degrees(radians))); };
        w.speed = get("speed", w.speed);
        w.backSpeed = get("backSpeed", w.backSpeed);
        w.sideSpeed = get("sideSpeed", w.sideSpeed);
        w.jogSpeed = get("jogSpeed", w.jogSpeed);
        w.runSpeed = get("runSpeed", w.runSpeed);
        w.acceleration = get("acceleration", w.acceleration);
        w.deceleration = get("deceleration", w.deceleration);
        w.turnRate = deg("turnRateDeg", w.turnRate);
        w.stepK = get("stepK", w.stepK);
        w.refSpeed = get("refSpeed", w.refSpeed);
        w.swingFraction = get("swingFraction", w.swingFraction);
        w.minSwing = get("minSwing", w.minSwing);
        w.maxSwing = get("maxSwing", w.maxSwing);
        w.stepHeight = get("stepHeight", w.stepHeight);
        w.settleDistance = get("settleDistance", w.settleDistance);
        w.bob = get("bob", w.bob);
        w.sway = get("sway", w.sway);
        w.leanPerSpeed = deg("leanPerSpeedDeg", w.leanPerSpeed);
        w.leanPerAccel = deg("leanPerAccelDeg", w.leanPerAccel);
        w.turnLean = deg("turnLeanDeg", w.turnLean);
        w.maxLean = deg("maxLeanDeg", w.maxLean);
        w.leanRate = get("leanRate", w.leanRate);
        w.armSwing = deg("armSwingDeg", w.armSwing);
        w.armBackBias = deg("armBackBiasDeg", w.armBackBias);
        w.elbowBias = deg("elbowBiasDeg", w.elbowBias);
        w.elbowSwing = deg("elbowSwingDeg", w.elbowSwing);
        w.armRate = get("armRate", w.armRate);
        w.headStabilise = get("headStabilise", w.headStabilise);
        w.headLead = deg("headLeadDeg", w.headLead);
        w.headRate = get("headRate", w.headRate);
        w.flightFraction = get("flightFraction", w.flightFraction);
        w.toeOffPitch = deg("toeOffPitchDeg", w.toeOffPitch);
        w.landPitch = deg("landPitchDeg", w.landPitch);
        w.swingPeakAt = get("swingPeakAt", w.swingPeakAt);
        w.liftOffSpeed = get("liftOffSpeed", w.liftOffSpeed);
        w.pushTime = get("pushTime", w.pushTime);
        w.widthScale = get("widthScale", w.widthScale);
        w.landAhead = get("landAhead", w.landAhead);
        w.pelvisTwist = deg("pelvisTwistDeg", w.pelvisTwist);
        w.chestCounter = get("chestCounter", w.chestCounter);
        w.pelvisDrop = get("pelvisDrop", w.pelvisDrop);
        w.sideFollow = get("sideFollow", w.sideFollow);
    };
    load("move", moveSettings_);
    postureLean_ = math::radians(c.getFloat("move.postureLeanDeg", math::degrees(postureLean_)));
    stepSettings_.stepLength = c.getFloat("step.stepLength", stepSettings_.stepLength);
    toeClearance_ = c.getFloat("step.toeClearance", toeClearance_);
    stepSettings_.backStepLength = c.getFloat("step.backStepLength", stepSettings_.backStepLength);
    stepSettings_.sideStepLength = c.getFloat("step.sideStepLength", stepSettings_.sideStepLength);
    stepSettings_.stepDuration = c.getFloat("step.stepDuration", stepSettings_.stepDuration);
    stepSettings_.stepHeight = c.getFloat("step.stepHeight", stepSettings_.stepHeight);
    stepSettings_.peakAt = c.getFloat("step.swingPeakAt", stepSettings_.peakAt);
    stepSettings_.liftAt = c.getFloat("step.liftAt", stepSettings_.liftAt);
    speedStep_ = c.getFloat("step.speedStep", speedStep_);
    speedMin_ = c.getFloat("step.speedMin", speedMin_);
    speedMax_ = c.getFloat("step.speedMax", speedMax_);
    {
        motion::JumpPlannerSettings& j = jumpPlan_;
        j.crouchDrop = c.getFloat("jump.crouchDrop", j.crouchDrop);
        j.crouchTime = c.getFloat("jump.crouchTime", j.crouchTime);
        j.pushTime = c.getFloat("jump.pushTime", j.pushTime);
        j.pushLift = c.getFloat("jump.pushLift", j.pushLift);
        j.landDrop = c.getFloat("jump.landDrop", j.landDrop);
        j.landTime = c.getFloat("jump.landTime", j.landTime);
        j.recoverTime = c.getFloat("jump.recoverTime", j.recoverTime);
        j.maxFlight = c.getFloat("jump.maxFlight", j.maxFlight);
    }
    for (auto [block, shares] : {std::pair{"step", &stepPassive_}, std::pair{"jump", &jumpPassive_}}) {
        const std::string at = std::string(block) + ".passive.";
        shares->head = c.getFloat(at + "head", shares->head);
        shares->hands = c.getFloat(at + "hands", shares->hands);
        shares->hips = c.getFloat(at + "hips", shares->hips);
        shares->chest = c.getFloat(at + "chest", shares->chest);
    }
    baseStep_ = stepSettings_;
    speedScale_ = 1.0f;
}

void Human::changeWalkSpeed(int direction) {
    if (direction == 0) return;
    speedScale_ = std::clamp(speedScale_ * (direction > 0 ? speedStep_ : 1.0f / speedStep_), speedMin_, speedMax_);
    LOG_INFO("walk pace x{:.2f} (step {:.2f} m in {:.2f} s)", speedScale_, baseStep_.stepLength * std::sqrt(speedScale_),
             baseStep_.stepDuration / std::sqrt(speedScale_));
}

bool Human::setHeight(float height) {
    if (!rig_.build(def_, height)) return false;
    if (!jointsConfig_ || !limits_.load(*jointsConfig_, rig_)) return false;

    // Knee flexion is +X, elbow flexion -X; ozz's mid axis opens the joint.
    auto chain = [&](const char* a, const char* b, const char* c, math::Vec3 axis) {
        return ik::TwoBoneChain{rig_.findJoint(a), rig_.findJoint(b), rig_.findJoint(c), axis};
    };
    joints_.pelvis = rig_.findJoint("pelvis");
    joints_.chest = rig_.findJoint("chest");
    joints_.head = rig_.findJoint("head");
    joints_.toeL = rig_.findJoint("toe_l");
    joints_.toeR = rig_.findJoint("toe_r");
    joints_.clavicleL = rig_.findJoint("clavicle_l");
    joints_.clavicleR = rig_.findJoint("clavicle_r");
    joints_.spine[0] = rig_.findJoint("spine_01");
    joints_.spine[1] = rig_.findJoint("spine_02");
    joints_.spine[2] = joints_.chest;
    joints_.legL = chain("thigh_l", "shin_l", "foot_l", {-1, 0, 0});
    joints_.legR = chain("thigh_r", "shin_r", "foot_r", {-1, 0, 0});
    joints_.armL = chain("upper_arm_l", "forearm_l", "hand_l", {1, 0, 0});
    joints_.armR = chain("upper_arm_r", "forearm_r", "hand_r", {1, 0, 0});

    current_ = rig_.restPose();
    previous_ = current_;

    // Segment masses: each joint's share sits midway to its only child, at the
    // leaf tip, or at the joint itself when it branches (pelvis, chest).
    const int n = rig_.numJoints();
    segmentMass_.assign(static_cast<size_t>(n), 0.0f);
    segmentEnd_.assign(static_cast<size_t>(n), -1);
    segmentTip_.assign(static_cast<size_t>(n), math::Vec3(0.0f));
    std::vector<int> children(static_cast<size_t>(n), 0);
    for (int j = 0; j < n; ++j) {
        const rig::BoneDef& bone = def_.bones[static_cast<size_t>(rig_.boneDefIndex(j))];
        segmentMass_[static_cast<size_t>(j)] = bone.massPercent;
        segmentTip_[static_cast<size_t>(j)] = bone.tip * rig_.height();
        const int p = rig_.parent(j);
        if (p < 0) continue;
        if (children[static_cast<size_t>(p)]++ == 0) segmentEnd_[static_cast<size_t>(p)] = j;
        else segmentEnd_[static_cast<size_t>(p)] = -1;
    }
    models_.assign(static_cast<size_t>(rig_.numJoints()), math::Mat4(1.0f));
    body_ = BodyState{};
    body_.position = current_.locals[static_cast<size_t>(joints_.pelvis)].position;  // pelvis is the root joint
    rig_.localToModel(current_, models_);
    // Deepest crouch: the leg with the knee bent to its joint limit.
    {
        const float thigh = math::distance(modelPos(models_, joints_.legL.start), modelPos(models_, joints_.legL.mid));
        const float shin = math::distance(modelPos(models_, joints_.legL.mid), modelPos(models_, joints_.legL.end));
        reposition_.minReach = bentReach(thigh, shin, limits_.limit(joints_.legL.mid).twistMax) / rig_.legLength();
    }
    if (joints_.toeL >= 0) {
        rig_.localToModel(current_, models_);
        const math::Vec3 d = modelPos(models_, joints_.toeL) - modelPos(models_, joints_.legL.end);
        ballDistance_ = math::length(math::Vec3(d.x, 0.0f, d.z));
        ballDrop_ = -d.y;
    }
    plantFeet();
    return true;
}

void Human::placeAt(math::Vec3 position, float yaw) {
    root_.position = position;
    root_.rotation = math::fromAxisAngle(math::kUp, yaw);
    previousRoot_ = root_;
    mover_.reset(yaw);
    if (physics_) physics_->placeCharacter(capsule_, position);
    plantFeet();
}

void Human::followCapsule(math::Vec3 from, float dt) {
    if (!physics_ || capsule_ == physics::kInvalidCharacter || dt <= 0.0f) return;
    math::Vec3 velocity = (root_.position - from) / dt;  // the walker's move (turning only now)
    if (humanBody_.ready() && supportOn_) {
        // The capsule follows the real hips (DECISIONS 2026-11-11): under the
        // physics pelvis every tick, moved (not placed) so it still collides.
        // No loop: the hold's anchor rides the plan's pelvis, not the capsule,
        // and the plan stays put in the world as the root moves (rebase).
        const math::Vec3 hips = physics_->transform(humanBody_.parts()[0].body).position;
        const math::Vec3 at = physics_->characterFeet(capsule_);
        velocity = math::Vec3(hips.x - at.x, 0.0f, hips.z - at.z) / dt;
    }
    const math::Vec3 feet = physics_->moveCharacter(capsule_, velocity, dt);
    const math::Transform before = root_;
    root_.position.x = feet.x;
    root_.position.z = feet.z;
    // On the ground the root is on its surface (the capsule floats its
    // padding above it); off it (a drop), at the capsule.
    root_.position.y = physics_->characterOnGround(capsule_) ? physics_->groundHeightAt(feet) : feet.y;
    stepper_.rebase(before, root_);
    const math::Transform toNew = math::inverse(root_) * before;
    decider_.rebase(toNew);
}

void Human::plantFeet() {
    // Feet planted where they stand in the rest pose (character space).
    rig_.localToModel(rig_.restPose(), models_);
    stepper_.reset(modelPos(models_, joints_.legL.end), modelPos(models_, joints_.legR.end));
}

void Human::tick(double dt) {
    if (joints_.pelvis < 0) return;  // not built yet (SimWorld ticks once while configuring)
    tickSeconds_ = static_cast<float>(dt);
    previous_.locals = current_.locals;  // same size: no allocation
    previousRoot_ = root_;
    current_.locals = rig_.restPose().locals;
    // Single steps: a step input takes steps (held: walking).
    if (math::length2(stepInput_) > 1e-6f) stepWalk_ = true;
    const math::Vec3 walkFrom = root_.position;
    walk(static_cast<float>(dt));
    followCapsule(walkFrom, static_cast<float>(dt));
    targets_.footL = root_.transformPoint(stepper_.foot(motion::Foot::Left));
    targets_.footR = root_.transformPoint(stepper_.foot(motion::Foot::Right));
    // Feet stand on the world's real surface under them (Jolt ground query:
    // the field, later a step or a slope), not on an assumed flat ground.
    if (physics_) {
        const float base = physics_->groundHeightAt(root_.position);
        targets_.footL.y += physics_->groundHeightAt(targets_.footL) - base;
        targets_.footR.y += physics_->groundHeightAt(targets_.footR) - base;
    }
    // The body follows the feet (repositionBody); with the physics body the
    // decider's plan (steps, jump, limb test) is what the muscles pull to.
    repositionBody(static_cast<float>(dt));
    solveIk();
    swingArms(static_cast<float>(dt));
    stabiliseHead(static_cast<float>(dt));
    limits_.apply(current_);
    // The full-physics body: its muscles pull toward the decider's plan,
    // the capsule holding it up.
    if (physics_ && humanBody_.ready()) {
        rig_.localToModel(current_, models_);
        const std::vector<math::Mat4>* plan = limbTestPlan(static_cast<float>(dt));
        physicsTick(plan ? *plan : models_, static_cast<float>(dt));
    }
}

void Human::walk(float dt) {
    // Move and turn the root; planted feet stay where they are in the world.
    // While posing (limb test) the fielder stands and does not turn.
    const bool posing = phase_ == Phase::Pose;
    // With the physics body WASD moves the hips (the walker's eased velocity,
    // at the pace E / Q set); the decider's balance step takes the steps.
    // The root does not move with it: it follows the real hips (the capsule).
    const bool body = physics_ && humanBody_.ready();
    motion::MoveSettings paced = moveSettings_;
    paced.speed *= speedScale_;
    paced.backSpeed *= speedScale_;
    paced.sideSpeed *= speedScale_;
    if (pace_ > 0) paced.speed = pace_ == 2 ? moveSettings_.runSpeed : moveSettings_.jogSpeed;  // forward only
    mover_.tick(dt, body && !posing ? stepInput_ : math::Vec2(0.0f), face_ && !posing, faceYaw_, paced);
    const motion::MoveSettings& w = moveSettings_;
    const math::Transform from = root_;
    if (!body) root_.position += mover_.velocity() * dt;
    root_.rotation = math::fromAxisAngle(math::kUp, mover_.yaw());
    stepper_.rebase(from, root_);
    {
        // The planted feet (and a step's points) keep their place and heading
        // in the world as the root turns; only a swinging foot turns with it.
        const math::Transform turned = math::inverse(root_) * from;
        decider_.rebase(turned);
    }

    const math::Vec3 velocity = math::inverse(root_).transformDirection(mover_.velocity());
    // Below walking speed, steps keep the walk's rhythm (full size from the
    // first step and while stopping); faster, they follow the actual speed.
    const float walkSpeed = moveSettings_.speed;
    const float timingSpeed = mover_.speed() > walkSpeed ? mover_.speed() : std::min(mover_.gaitSpeed(), walkSpeed);
    // Single steps: the stepper picks the foot (the leading side's foot, else
    // alternating) and swings it to its spot; the body follows from the
    // body reposition. The root (and the camera with it) stays between the
    // feet every tick, so it glides along with the swinging foot.
    if (body) {
        // The decider's own balance step: its shape (locomotion.json
        // "step"), its timing by the walk's gait, its passive shares.
        motion::StepPlannerSettings& shape = deciderSettings_.stepShape;
        shape.duration = stepSettings_.stepDuration;
        shape.liftAt = stepSettings_.liftAt;
        shape.height = stepSettings_.stepHeight;
        shape.peakAt = stepSettings_.peakAt;
        shape.toeOffPitch = moveSettings_.toeOffPitch;
        shape.ballReach = ballReach_;
        deciderSettings_.gait = &moveSettings_;
        deciderSettings_.stepPassive = stepPassive_;
        return;
    }
    if (stepWalk_) {
        stepper_.tick(dt, stepInput_, stepSettings_);
        const math::Transform before = root_;
        const math::Vec3 mid = root_.transformPoint(0.5f * (stepper_.foot(motion::Foot::Left) + stepper_.foot(motion::Foot::Right)));
        root_.position.x = mid.x;
        root_.position.z = mid.z;
        stepper_.rebase(before, root_);
        return;
    }
    if (posing) {
        // No gait while posing: only a step already under way finishes.
        stepper_.tick(dt, math::Vec2(0.0f), stepSettings_);
        return;
    }
    stepper_.tickGait(dt, velocity, motion::gaitTiming(timingSpeed, rig_.height(), w), w.stepHeight,
                      w.settleDistance, gaitStyle());
}

motion::GaitStyle Human::gaitStyle() const {
    const motion::MoveSettings& w = moveSettings_;
    motion::GaitStyle s;
    s.toeOffPitch = w.toeOffPitch;
    s.landPitch = w.landPitch;
    s.peakAt = w.swingPeakAt;
    s.liftOffSpeed = w.liftOffSpeed;
    s.pushTime = w.pushTime;
    s.widthScale = w.widthScale;
    s.landAhead = w.landAhead;
    return s;
}

void Human::repositionBody(float dt) {
    // Hip offsets from the rest pose; the pelvis is the root joint, so its
    // local position is in character space.
    rig_.localToModel(rig_.restPose(), models_);
    const math::Vec3 restPelvis = modelPos(models_, joints_.pelvis);
    const math::Transform toModel = math::inverse(root_);
    LegSupport left{modelPos(models_, joints_.legL.start) - restPelvis, toModel.transformPoint(targets_.footL)};
    LegSupport right{modelPos(models_, joints_.legR.start) - restPelvis, toModel.transformPoint(targets_.footR)};

    // The lower foot carries the body; even weight when both are level.
    const float stance = math::clamp(0.5f + 0.5f * (left.foot.y - right.foot.y) / stanceBlendHeight_, 0.0f, 1.0f);
    RepositionSettings reposition = reposition_;
    reposition.sideFollow = moveSettings_.sideFollow;  // runners barely shift sideways
    // Moving, the root already travels smoothly; chasing the stance foot
    // (which slides back every step) would make the body lag and surge.
    reposition.forwardFollow = 1.0f - math::clamp(mover_.speed() / std::max(moveSettings_.speed, 0.1f), 0.0f, 1.0f);
    reposition.restZ = restPelvis.z;
    // Standing rules (straight-leg height, hip drop); moving, the gait's own
    // pelvis motion (addGaitMotion) takes over.
    reposition.standShare = reposition.forwardFollow;

    moveBodyToFeet(body_, left, right, rig_.legLength(), stance, reposition, dt);
    math::Transform& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)];
    pelvis.position = body_.position;
    pelvis.rotation = body_.rotation() * pelvis.rotation;
}


void Human::solveIk() {
    const math::Transform toModel = math::inverse(root_);
    auto model = [&](math::Vec3 w) { return toModel.transformPoint(w); };

    // Knees point where the pelvis faces: the pole turns with the pelvis's
    // twist about the vertical (not its forward tilt). With a fixed pole, a
    // pelvis turned while bent forward would abduct the thighs past their limit.
    math::Quat pelvisSwing, pelvisTurn;
    math::swingTwist(current_.locals[static_cast<size_t>(joints_.pelvis)].rotation, math::kUp, pelvisSwing, pelvisTurn);
    const math::Vec3 kneePole = pelvisTurn * ik_.kneePole;

    // A lifted heel raises the ankle about the ball of the foot (last tick's
    // lift): the ball stays put, the ankle swings up round it.
    auto ankle = [&](math::Vec3 target, int i) {
        const float p = heelLift_[i];
        return model(target) + math::kUp * (ballDistance_ * std::sin(p) - ballDrop_ * (1.0f - std::cos(p))) +
               math::kForward * (ballDistance_ * (1.0f - std::cos(p)) + ballDrop_ * std::sin(p));
    };
    rig_.localToModel(current_, models_);
    ik::solveTwoBone(joints_.legL, models_, ankle(targets_.footL, 0), kneePole, ik_.soften, current_);
    ik::solveTwoBone(joints_.legR, models_, ankle(targets_.footR, 1), kneePole, ik_.soften, current_);
    limits_.apply(current_);

    // Feet keep the stepper's pitch in character space (flat when walking).
    // Past the ankle's range the foot turns with the leg (BIOMECHANICS 3.1):
    //   tip = how far the ankle's joint limit turns the flat foot's toes down;
    //   tip > 0: the heel lifts by it (toes bend back as much); tip < 0: the toes rise.
    // The heel lift raises the ankle next tick so the ball of the foot stays
    // down (heelLift_, smoothed so it cannot flicker).
    rig_.localToModel(current_, models_);
    const ik::TwoBoneChain* legs[2] = {&joints_.legL, &joints_.legR};
    const int toes[2] = {joints_.toeL, joints_.toeR};
    for (int i = 0; i < 2; ++i) {
        const motion::Foot f = static_cast<motion::Foot>(i);
        const math::Quat shin = modelRot(models_, legs[i]->mid);
        const bool planted = !(stepper_.stepping() && stepper_.swingFoot() == i);
        const math::Quat flat = math::fromAxisAngle(math::kRight, stepper_.footPitch(f));
        const math::Quat limited = shin * ik::clampRotation(math::normalize(math::inverse(shin) * flat), limits_.limit(legs[i]->end));
        auto toesDown = [](math::Quat q) { return std::asin(math::clamp(-(q * math::kForward).y, -1.0f, 1.0f)); };
        const float tip = planted ? toesDown(limited) - toesDown(flat) : 0.0f;
        const float heel = std::max(tip, 0.0f), toesUp = std::max(-tip, 0.0f);
        heelLift_[i] += (heel - heelLift_[i]) * std::min(1.0f, kHeelRate * tickSeconds_);

        const float pitch = std::max(stepper_.footPitch(f), heelLift_[i]) - toesUp;
        current_.locals[static_cast<size_t>(legs[i]->end)].rotation = math::normalize(
            math::inverse(shin) * math::fromAxisAngle(math::kRight, pitch));
        if (toes[i] >= 0)
            current_.locals[static_cast<size_t>(toes[i])].rotation =
                math::fromAxisAngle(math::kRight, std::min(stepper_.toeBend(f), -heelLift_[i]));  // toes stay flat
    }
    limits_.apply(current_);
}

void Human::swingArms(float dt) {
    // Leg position from the steps: the swinging leg travels back to front;
    // held between steps. The arms swing opposite the same-side leg.
    const int swing = stepper_.swingFoot();
    const float sweep = std::cos(math::kPi * stepper_.swingProgress());
    if (swing == static_cast<int>(motion::Foot::Left)) legPhase_ = -sweep;
    else if (swing == static_cast<int>(motion::Foot::Right)) legPhase_ = sweep;
    const motion::MoveSettings& w = moveSettings_;

    // Full swing walking forward, little when sidestepping or backing up.
    const float forward = math::dot(math::inverse(root_).transformDirection(mover_.velocity()), math::kForward);
    const float want = math::clamp(forward / std::max(w.speed, 0.1f), 0.0f, 1.0f);
    armAmount_ += (want - armAmount_) * std::min(1.0f, w.armRate * dt);

    // Each arm opposite its leg; positive about +X moves a hanging arm back,
    // negative bends the elbow (rest rotations are identity).
    const float half = 0.5f * w.armSwing * armAmount_;
    auto arm = [&](const ik::TwoBoneChain& chain, float legForward) {
        const float shoulder = w.armBackBias * armAmount_ + half * legForward;
        const float elbow = -armAmount_ * (w.elbowBias + w.elbowSwing * 0.5f * (1.0f - legForward));
        current_.locals[static_cast<size_t>(chain.start)].rotation = math::fromAxisAngle(math::kRight, shoulder);
        current_.locals[static_cast<size_t>(chain.mid)].rotation = math::fromAxisAngle(math::kRight, elbow);
    };
    arm(joints_.armL, legPhase_);
    arm(joints_.armR, -legPhase_);

}

void Human::stabiliseHead(float dt) {
    const motion::MoveSettings& w = moveSettings_;

    // Lead turns: the head looks toward the requested heading before the body.
    float want = 0.0f;
    if (face_) {
        const float diff = std::remainder(faceYaw_ - mover_.yaw(), math::kTwoPi);
        want = math::clamp(diff, -w.headLead, w.headLead);
    }
    headYaw_ += (want - headYaw_) * std::min(1.0f, w.headRate * dt);

    // Keep the head upright in character space, cancelling the lean and sway
    // that reach it through the spine.
    rig_.localToModel(current_, models_);
    const int parent = rig_.parent(joints_.head);
    if (parent < 0) return;
    const math::Quat desired = math::fromAxisAngle(math::kUp, headYaw_);
    const math::Quat level = math::normalize(math::inverse(modelRot(models_, parent)) * desired);
    math::Quat& head = current_.locals[static_cast<size_t>(joints_.head)].rotation;
    head = math::normalize(math::slerp(head, level, math::clamp(w.headStabilise, 0.0f, 1.0f)));
}

math::Vec3 Human::modelCentreOfMass() {
    math::Vec3 sum(0.0f);
    float total = 0.0f;
    for (size_t j = 0; j < segmentMass_.size(); ++j) {
        const float m = segmentMass_[j];
        const math::Vec3 at = math::Vec3(models_[j][3]);
        const int end = segmentEnd_[j];
        const math::Vec3 p = end >= 0 ? 0.5f * (at + math::Vec3(models_[static_cast<size_t>(end)][3]))
                                      : math::Vec3(models_[j] * math::Vec4(0.5f * segmentTip_[j], 1.0f));
        sum += p * m;
        total += m;
    }
    return total > 0.0f ? sum / total : math::Vec3(0.0f);
}

bool Human::attachPhysics(physics::PhysicsWorld& world) {
    deciderSettings_.gravity = -world.world().gravity.y;  // free arms hang on it (the decider)
    {
        // The standing ankle's height (rest pose, character space: the
        // ground at y = 0) plus the toe clearance: a lifted foot's lowest goal.
        std::vector<math::Mat4> rest(static_cast<size_t>(rig_.numJoints()));
        if (rig_.localToModel(rig_.restPose(), rest) && joints_.legL.end >= 0)
            liftedAnkleMin_ = rest[static_cast<size_t>(joints_.legL.end)][3].y + toeClearance_;
        // The ankle ahead of the ball of the foot (the toe joint), level: the
        // foot rolls over it at push-off.
        if (joints_.legL.end >= 0 && joints_.toeL >= 0) {
            const math::Vec3 d = math::Vec3(rest[static_cast<size_t>(joints_.toeL)][3]) -
                                 math::Vec3(rest[static_cast<size_t>(joints_.legL.end)][3]);
            ballReach_ = std::sqrt(d.x * d.x + d.z * d.z);
        }
    }
    physics_ = &world;
    capsule_ = world.createCharacter(rig_.height(), root_.position);
    if (capsule_ == physics::kInvalidCharacter) {
        LOG_ERROR("human: no room for the capsule in the physics world");
        physics_ = nullptr;
        return false;
    }
    humanBody_ = HumanBody{};
    // The centre of mass over the root at rest (the body starts standing).
    rig_.localToModel(rig_.restPose(), models_);
    const math::Vec3 com = modelCentreOfMass();
    restComHeight_ = com.y;
    static uint32_t nextGroup = 1;  // each human's parts skip only each other
    MuscleSettings muscles;
    if (jointsConfig_) {
        muscles.activationTime = jointsConfig_->getFloat("muscles.activationTime", muscles.activationTime);
        muscles.damping = jointsConfig_->getFloat("muscles.damping", muscles.damping);
        const nlohmann::json& root = jointsConfig_->root();
        if (auto m = root.find("muscles"); m != root.end())
            if (auto t = m->find("maxTorque"); t != m->end() && t->is_object())
                for (auto it = t->begin(); it != t->end(); ++it)
                    if (it->is_number()) muscles.maxTorque.emplace_back(it.key(), it->get<float>());
        if (auto m = root.find("muscles"); m != root.end())
            if (auto t = m->find("directional"); t != m->end() && t->is_object())
                for (auto it = t->begin(); it != t->end(); ++it) {
                    const nlohmann::json& v = *it;
                    if (!v.is_object() || !v.contains("axis") || !v.contains("plus") || !v.contains("minus")) {
                        LOG_WARN("fielder: muscles.directional.{} needs axis, plus and minus", it.key());
                        continue;
                    }
                    const std::string axis = v["axis"].is_string() ? v["axis"].get<std::string>() : "";
                    if (axis != "x" && axis != "y" && axis != "z") {
                        LOG_WARN("fielder: muscles.directional.{}.axis must be x, y or z", it.key());
                        continue;
                    }
                    MuscleSettings::Directional d;
                    d.bone = it.key();
                    d.axis = axis[0] - 'x';
                    d.plus = v["plus"].get<float>();
                    d.minus = v["minus"].get<float>();
                    muscles.directional.push_back(d);
                }
        if (auto m = root.find("muscles"); m != root.end())
            for (auto [key, list] : {std::pair{"stiffness", &muscles.stiffness}, std::pair{"jointDamping", &muscles.jointDamping}})
                if (auto t = m->find(key); t != m->end() && t->is_object())
                    for (auto it = t->begin(); it != t->end(); ++it)
                        if (it->is_number()) list->emplace_back(it.key(), it->get<float>());
    }
    if (!humanBody_.init(world, def_, rig_, limits_, muscles, root_, nextGroup++)) {
        LOG_ERROR("fielder: no room for the body in the physics world");
        physics_ = nullptr;
        return false;
    }
    // The physics loop's parts: decider and muscle timing (config).
    deciderJoints_.pelvis = joints_.pelvis;
    for (int i = 0; i < 3; ++i) deciderJoints_.spine[i] = joints_.spine[i];
    deciderJoints_.neck = rig_.findJoint("neck");
    deciderJoints_.head = joints_.head;
    deciderJoints_.clavicles[0] = joints_.clavicleL;
    deciderJoints_.clavicles[1] = joints_.clavicleR;
    deciderJoints_.legs[0] = joints_.legL;
    deciderJoints_.legs[1] = joints_.legR;
    deciderJoints_.arms[0] = joints_.armL;
    deciderJoints_.arms[1] = joints_.armR;
    if (!decider_.init(rig_, deciderJoints_, &limits_)) {
        LOG_ERROR("human: the body decider could not start");
        return false;
    }
    {
        std::vector<float> mass;
        std::vector<math::Vec3> centre;
        humanBody_.jointMasses(rig_.numJoints(), mass, centre);
        decider_.setMasses(mass, centre);
    }
    if (jointsConfig_) {
        muscleTiming_.fittsA = jointsConfig_->getFloat("muscles.fittsA", muscleTiming_.fittsA);
        muscleTiming_.fittsB = jointsConfig_->getFloat("muscles.fittsB", muscleTiming_.fittsB);
        muscleTiming_.fittsWidth = jointsConfig_->getFloat("muscles.fittsWidth", muscleTiming_.fittsWidth);
        deciderSettings_.queueShare = jointsConfig_->getFloat("decider.queueShare", deciderSettings_.queueShare);
        deciderSettings_.armCancel = jointsConfig_->getFloat("decider.armCancel", deciderSettings_.armCancel);
        deciderSettings_.passiveHead = jointsConfig_->getFloat("decider.passiveInfluence.head", deciderSettings_.passiveHead);
        deciderSettings_.passiveHands = jointsConfig_->getFloat("decider.passiveInfluence.hands", deciderSettings_.passiveHands);
        deciderSettings_.passiveHips = jointsConfig_->getFloat("decider.passiveInfluence.hips", deciderSettings_.passiveHips);
        deciderSettings_.passiveChest = jointsConfig_->getFloat("decider.passiveInfluence.chest", deciderSettings_.passiveChest);
        auto dv = [&](const char* key, float value) { return jointsConfig_->getFloat(key, value); };
        deciderSettings_.hipsSway = dv("decider.hipsSway", deciderSettings_.hipsSway);
        deciderSettings_.pelvisTurnShare = dv("decider.pelvisTurnShare", deciderSettings_.pelvisTurnShare);
        deciderSettings_.trunkTurnFrequency = dv("decider.trunkTurn.frequency", deciderSettings_.trunkTurnFrequency);
        deciderSettings_.trunkTurnDamping = dv("decider.trunkTurn.damping", deciderSettings_.trunkTurnDamping);
        deciderSettings_.limbMinBend = math::radians(dv("decider.limbMinBendDeg", math::degrees(deciderSettings_.limbMinBend)));
        deciderSettings_.blendTime = dv("decider.blendTime", deciderSettings_.blendTime);
        deciderSettings_.supportBlendTime = dv("decider.supportBlendTime", deciderSettings_.supportBlendTime);
        deciderSettings_.hipsMaxSpeed = dv("decider.hipsMaxSpeed", deciderSettings_.hipsMaxSpeed);
        deciderSettings_.hipsMaxAccel = dv("decider.hipsMaxAccel", deciderSettings_.hipsMaxAccel);
        deciderSettings_.hipsMaxTurnRate = dv("decider.hipsMaxTurnRate", deciderSettings_.hipsMaxTurnRate);
        deciderSettings_.trunkMaxRate = dv("decider.trunkMaxRate", deciderSettings_.trunkMaxRate);
        deciderSettings_.headMaxRate = dv("decider.headMaxRate", deciderSettings_.headMaxRate);
        deciderSettings_.limbMaxRate = dv("decider.limbMaxRate", deciderSettings_.limbMaxRate);
        deciderSettings_.legMaxRate = dv("decider.legMaxRate", deciderSettings_.legMaxRate);
        deciderSettings_.walkRunFroude = dv("decider.run.walkRunFroude", deciderSettings_.walkRunFroude);
        deciderSettings_.walkRunBand = dv("decider.run.walkRunBand", deciderSettings_.walkRunBand);
        deciderSettings_.legStiffness = dv("decider.run.legStiffness", deciderSettings_.legStiffness);
        deciderSettings_.flightTimeJog = dv("decider.run.flightTimeJog", deciderSettings_.flightTimeJog);
        deciderSettings_.flightTimeRun = dv("decider.run.flightTimeRun", deciderSettings_.flightTimeRun);
        deciderSettings_.swingHeightJog = dv("decider.run.swingHeightJog", deciderSettings_.swingHeightJog);
        deciderSettings_.swingHeightRun = dv("decider.run.swingHeightRun", deciderSettings_.swingHeightRun);
        deciderSettings_.runSwingPeakAt = dv("decider.run.swingPeakAt", deciderSettings_.runSwingPeakAt);
        deciderSettings_.twoLegKneeMax =
            math::radians(dv("decider.twoLegKneeMaxDeg", math::degrees(deciderSettings_.twoLegKneeMax)));
        deciderSettings_.oneLegKneeMax =
            math::radians(dv("decider.oneLegKneeMaxDeg", math::degrees(deciderSettings_.oneLegKneeMax)));
        // The pelvis tilt limits (BIOMECHANICS 9.3), degrees in config.
        deciderSettings_.pelvisMin =
            math::radians(jointsConfig_->getVec3("decider.pelvisMinDeg", math::degrees(deciderSettings_.pelvisMin)));
        deciderSettings_.pelvisMax =
            math::radians(jointsConfig_->getVec3("decider.pelvisMaxDeg", math::degrees(deciderSettings_.pelvisMax)));
        deciderSettings_.balanceStepOn = jointsConfig_->getBool("decider.balanceStep.on", deciderSettings_.balanceStepOn);
        deciderSettings_.stepMargin = dv("decider.balanceStep.margin", deciderSettings_.stepMargin);
        deciderSettings_.footHalfWidth = dv("decider.balanceStep.footHalfWidth", deciderSettings_.footHalfWidth);
        deciderSettings_.footHalfLength = dv("decider.balanceStep.footHalfLength", deciderSettings_.footHalfLength);
        deciderSettings_.footCentreAhead = dv("decider.balanceStep.footCentreAhead", deciderSettings_.footCentreAhead);
        deciderSettings_.landingBehind = dv("decider.balanceStep.landingBehind", deciderSettings_.landingBehind);
        deciderSettings_.minFootGap = dv("decider.balanceStep.minFootGap", deciderSettings_.minFootGap);
        deciderSettings_.maxStepForward = dv("decider.balanceStep.maxStepForward", deciderSettings_.maxStepForward);
        deciderSettings_.maxStepSide = dv("decider.balanceStep.maxStepSide", deciderSettings_.maxStepSide);
        deciderSettings_.reachDrop = dv("decider.balanceStep.reachDrop", deciderSettings_.reachDrop);
        deciderSettings_.kneePole = ik_.kneePole;  // knees forward
        const nlohmann::json& root = jointsConfig_->root();
        if (auto d = root.find("decider"); d != root.end()) {
            auto bands = d->find("lumbarHipBandsDeg");
            auto ratios = d->find("lumbarHipRatio");
            for (int i = 0; i < 3; ++i) {
                if (bands != d->end() && bands->is_array() && bands->size() > static_cast<size_t>(i))
                    deciderSettings_.lumbarHipBand[i] = math::radians((*bands)[static_cast<size_t>(i)].get<float>());
                if (ratios != d->end() && ratios->is_array() && ratios->size() > static_cast<size_t>(i))
                    deciderSettings_.lumbarHipRatio[i] = (*ratios)[static_cast<size_t>(i)].get<float>();
            }
        }
    }
    physicsModels_.resize(static_cast<size_t>(rig_.numJoints()));
    physicsPose_ = rig_.restPose();
    jumpPlanner_ = motion::JumpPlanner{};
    strength_ = appliedStrength_ = 1.0f;
    pelvisError_ = PelvisError::Small;
    pelvisErrorNow_ = pelvisErrorTime_ = 0.0f;
    restPelvisHeight_ = std::max(models_[static_cast<size_t>(joints_.pelvis)][3].y, 0.1f);
    // The capsule's hold: a hidden kinematic body riding the plan's pelvis,
    // sprung to the pelvis on all six axes (physics_world.json "character").
    physics::BodyDesc anchor;
    anchor.shape = physics::ShapeType::Sphere;
    anchor.radius = 0.01f;
    anchor.kinematic = true;
    anchor.collides = false;
    anchor.pose = math::Transform{world.transform(humanBody_.parts()[0].body).position, root_.rotation};
    anchor_ = world.createBody(anchor);
    const physics::CharacterSettings& c = world.settings().character;
    // The hold moves the whole body (not the pelvis alone) at its frequency:
    // stiffness from the body's mass and its inertia about the pelvis at
    // rest (k = I w^2, damping 2 zeta I w).
    float inertia = 0.0f;
    const math::Vec3 pelvisAt = world.transform(humanBody_.parts()[0].body).position;
    for (const HumanBody::Part& p : humanBody_.parts())
        inertia += p.mass * math::length2(world.transform(p.body).position - pelvisAt);
    const float w = math::kTwoPi * c.supportFrequency;
    physics::MotorDesc linear, angular;
    linear.stiffness = humanBody_.totalMass() * w * w;
    linear.damping = 2.0f * c.supportDamping * humanBody_.totalMass() * w;
    angular.stiffness = inertia * w * w;
    angular.damping = 2.0f * c.supportDamping * inertia * w;
    linear.maxTorque = c.supportMaxForce;
    angular.maxTorque = c.supportMaxTorque;
    support_ = anchor_ == physics::kInvalidBody ? physics::kInvalidJoint
                                                : world.createSupport(anchor_, humanBody_.parts()[0].body, linear, angular,
                                                                    c.supportFreeDown);
    if (support_ == physics::kInvalidJoint) {
        LOG_ERROR("human: no room for the capsule's hold in the physics world");
        physics_ = nullptr;
        return false;
    }
    supportOn_ = true;
    supportShare_ = 1.0f;
    restTime_ = 0.0f;
    return true;
}

bool Human::capsuleDebug(math::Vec3& feet, float& height, float& radius, math::Vec3& feetCentre, math::Vec3& hips) const {
    if (!physics_ || capsule_ == physics::kInvalidCharacter || !humanBody_.ready()) return false;
    feet = physics_->characterFeet(capsule_);
    height = physics_->characterHeight(capsule_);
    radius = physics_->settings().character.radius;
    feetCentre = math::Vec3(0.0f);
    for (int f = 0; f < 2; ++f) {
        const int part = humanBody_.partOf(f == 0 ? joints_.legL.end : joints_.legR.end);
        if (part >= 0) feetCentre += 0.5f * physics_->transform(humanBody_.parts()[static_cast<size_t>(part)].body).position;
    }
    hips = physics_->transform(humanBody_.parts()[0].body).position;
    return true;
}

void Human::setJumpInput(bool held) {
    // Held: the crouch starts (standing, the physics body, no step under
    // way); let go: the push.
    if (held && !jumpHeld_ && !jumpPlanner_.running() && decider_.stepFoot() < 0 && physics_ && humanBody_.ready()) {
        jumpPlanner_.start(restPelvisHeight_);
    }
    if (!held && jumpHeld_) jumpPlanner_.release();
    jumpHeld_ = held;
}

const std::vector<math::Mat4>* Human::limbTestPlan(float dt) {
    // The physics body now (character space): only where a hand's or foot's
    // path starts; the decider never reads it (its plan carries on).
    humanBody_.boneModels(*physics_, 1.0f, rig_, current_, root_, physicsModels_);
    // The commanded point on its way to the goal (GoalPath), from the end's
    // real position when a move starts.
    auto travel = [&](GoalPath& path, math::Vec3 goal, int end) {
        const bool done = path.time >= path.duration;
        if (!path.on || path.end != end || (done && math::distance(goal, path.to) > 1e-3f)) {
            path.from = path.on && path.end == end ? path.at : math::Vec3(physicsModels_[static_cast<size_t>(end)][3]);
            path.end = end;
            path.time = 0.0f;
            path.duration = movementTime(math::distance(path.from, goal), muscleTiming_.fittsWidth,
                                                 muscleTiming_.fittsA, muscleTiming_.fittsB);
            path.on = true;
        }
        path.to = goal;  // mid-way: the same path, retargeted
        path.time += dt;
        const float s = std::clamp(path.time / std::max(path.duration, 1e-3f), 0.0f, 1.0f);
        path.at = math::lerp(path.from, path.to, s * s * s * (10.0f + s * (6.0f * s - 15.0f)));  // minimum jerk
        return path.at;
    };
    // The planner's targets: every foot pinned where the stepper has it (on
    // the ground), the hips, chest and head free (the decider's own frames),
    // then the limb test's one goal or the step.
    motion::TargetSet& t = bodyTargets_;
    t = motion::TargetSet{};
    t.dt = dt;
    t.strength = strength_;
    for (int i = 0; i < 2; ++i) {
        t.foot[i].mode = motion::TargetMode::Pinned;
        t.foot[i].position = stepper_.foot(static_cast<motion::Foot>(i));
        t.foot[i].contactNormal = math::kUp;
    }
    // The limb test's target (the one set last): turned once the player
    // turned it (character space), else as the part's own default.
    const int g = poseLast_;
    auto turn = [&](motion::Target& target) {
        if (g < 0 || !poseTurned_[g]) return;
        target.rotation = math::normalize(math::inverse(root_.rotation) * poseRotation_[g]);
        target.weightRotation = 1.0f;
    };
    const bool poseHand = g == static_cast<int>(motion::Goal::HandL) || g == static_cast<int>(motion::Goal::HandR);
    if (poseHand) {
        const bool left = g == static_cast<int>(motion::Goal::HandL);
        motion::Target& hand = t.hand[left ? 0 : 1];
        hand.mode = motion::TargetMode::Driven;
        hand.position = travel(handPath_, math::inverse(root_).transformPoint(posePosition_[g]),
                               left ? joints_.armL.end : joints_.armR.end);
        turn(hand);
    } else {
        handPath_.on = false;
    }
    if (g == static_cast<int>(motion::Goal::Hips) && joints_.pelvis >= 0) {
        // The hips go there (every direction) and turn, within the pelvis's range.
        t.hips.mode = motion::TargetMode::Driven;
        t.hips.position = travel(hipsPath_, math::inverse(root_).transformPoint(posePosition_[g]), joints_.pelvis);
        t.hips.weightRotation = 0.0f;
        turn(t.hips);
    } else {
        hipsPath_.on = false;
    }
    for (motion::Goal part : {motion::Goal::Chest, motion::Goal::Head}) {
        if (g != static_cast<int>(part)) continue;
        // The chest goes there (the trunk bends, within its joints' ranges)
        // and turns; the head only turns.
        motion::Target& target = part == motion::Goal::Chest ? t.chest : t.head;
        target.mode = motion::TargetMode::Driven;
        target.position = math::inverse(root_).transformPoint(posePosition_[g]);
        target.weightPosition = part == motion::Goal::Chest ? 1.0f : 0.0f;
        target.weightRotation = 0.0f;
        turn(target);
    }
    // WASD: the hips target moves at the walker's velocity (world), from
    // where the plan's hips are when it starts; the height stays the
    // support's. Stopped, it holds there until the last balance step has
    // landed and the body is over its feet, then lets go (the hips settle
    // between the feet); let go at once, the hips would swing back and step.
    const math::Vec3 hipsVelocity = mover_.velocity();
    const bool moving = math::length2(hipsVelocity) > 1e-6f;
    const bool settling = hipsDriven_ && (decider_.stepFoot() >= 0 || queue_.step.wanted);
    if ((moving || settling) && phase_ != Phase::Pose && joints_.pelvis >= 0 && g != static_cast<int>(motion::Goal::Hips)) {
        if (!hipsDriven_) {
            const std::vector<math::Mat4>& plan = decider_.outputModels();
            const size_t pelvis = static_cast<size_t>(joints_.pelvis);
            hipsGoal_ = root_.transformPoint(pelvis < plan.size() ? math::Vec3(plan[pelvis][3])
                                                                  : math::Vec3(physicsModels_[pelvis][3]));
            hipsDriven_ = true;
        }
        hipsGoal_ += hipsVelocity * dt;
        // Active along the way WASD moves (kept while it settles); across and
        // up the body's own: the hips sway over the standing foot, the legs
        // set the height.
        if (moving) hipsDirection_ = math::normalize(math::Vec3(hipsVelocity.x, 0.0f, hipsVelocity.z));
        t.hips.mode = motion::TargetMode::Driven;
        t.hips.position = math::inverse(root_).transformPoint(hipsGoal_);
        t.hips.velocity = math::inverse(root_).transformDirection(hipsVelocity);
        t.hips.weightRotation = 0.0f;
        t.hips.directions[0] = math::inverse(root_).transformDirection(hipsDirection_);
        t.hips.directionCount = 1;
    } else {
        hipsDriven_ = false;
    }
    if (jumpPlanner_.running()) {
        // The jump: the planner gives the hips only (the active target);
        // the feet stay pinned as always, the rest follows by its passive
        // share. In the air it waits for the real feet to reach the ground;
        // they are pinned where they came down.
        if (jumpPlanner_.airborne() && jumpPlanner_.phaseTime() > 0.1f) {
            const float ground = liftedAnkleMin_ - toeClearance_;
            const math::Vec3 l(physicsModels_[static_cast<size_t>(joints_.legL.end)][3]);
            const math::Vec3 r(physicsModels_[static_cast<size_t>(joints_.legR.end)][3]);
            if (std::min(l.y, r.y) <= ground + 0.02f) {
                jumpPlanner_.touchdown();
            }
        }
        jumpPlanner_.tick(dt, jumpPlan_, t);
        t.passive = jumpPassive_;
        handPath_.on = footPath_.on = false;
        decider_.decide(t, deciderSettings_, queue_);  // the decider as for everything else
        return queue_.plan;
    }
    const bool poseFoot = g == static_cast<int>(motion::Goal::FootL) || g == static_cast<int>(motion::Goal::FootR);
    if (poseFoot) {
        const int f = g == static_cast<int>(motion::Goal::FootL) ? 0 : 1;
        const int ankle = f == 0 ? joints_.legL.end : joints_.legR.end;
        math::Vec3 goal = math::inverse(root_).transformPoint(posePosition_[g]);
        if (goal.y <= liftedAnkleMin_) {
            // On or under the ground: the foot is at once on the ground under
            // the target and stands there (pinned: it carries weight, the
            // hips go between the feet); it stays there after.
            const math::Vec3 spot(goal.x, liftedAnkleMin_ - toeClearance_, goal.z);
            stepper_.place(static_cast<motion::Foot>(f), spot);
            t.foot[f].mode = motion::TargetMode::Pinned;
            t.foot[f].position = spot;
            footPath_.on = false;
        } else {
            // Above the ground: the foot is held up there (no weight on it).
            t.foot[f].mode = motion::TargetMode::Driven;
            t.foot[f].position = travel(footPath_, goal, ankle);
            turn(t.foot[f]);
        }
    } else {
        footPath_.on = false;
    }
    // The decider decides the whole body, its balance steps too (a foot
    // moves only by a step, never slides; none in the limb test's foot case:
    // that foot is placed by hand), and gives the muscles its plan. A step
    // that landed: the foot stays on its spot from now on.
    t.balanceSteps = !poseFoot;
    decider_.decide(t, deciderSettings_, queue_);
    for (int i = 0; i < 2; ++i) {
        if (!queue_.landed[i]) continue;
        const motion::Foot f = static_cast<motion::Foot>(i);
        stepper_.place(f, queue_.landedSpot[i]);
        stepper_.setLastStepped(f);
    }
    return queue_.plan;
}

void Human::physicsTick(const std::vector<math::Mat4>& target, float dt) {
    physics::PhysicsWorld& world = *physics_;
    const physics::CharacterSettings& c = world.settings().character;
    // The muscles: they carry the weight beyond each joint, and Jolt's pose
    // drive corrects toward this tick's pose; both at the body's strength.
    humanBody_.holdWeight(world);
    if (strength_ != appliedStrength_) {
        humanBody_.setStrength(world, strength_);
        appliedStrength_ = strength_;
    }
    humanBody_.drivePose(world, target, root_, joints_.head);
    // The capsule is as tall as the pose's pelvis is high (crouch, stride).
    const float pelvisY = target[static_cast<size_t>(joints_.pelvis)][3].y;
    world.setCharacterHeight(capsule_, rig_.height() * std::clamp(pelvisY / restPelvisHeight_, 0.0f, 1.0f));

    if (supportOn_) {
        // The hold rides the plan's pelvis (turned the root's way, yaw only)
        // and pulls the real pelvis onto it: the spring by the gap, the
        // damper by the speed off the PLAN's (a still anchor would damp the
        // hips toward standing still and hold them back). Not the capsule:
        // that follows the real hips, and a hold riding it would chase them
        // (the 11-09 loop). Never higher than standing (as the capsule is
        // never taller than the player): pulled higher, the hold would lift
        // the feet off the ground.
        math::Transform planned = humanBody_.partPose(0, target);
        // The jump's push: the hold carries the hips with the plan's, past standing too.
        const bool pushing = jumpPlanner_.phase() == motion::JumpPlanner::Phase::Push;
        if (pushing) planned.position.y += bodyTargets_.hips.position.y - pelvisY;  // the jump's own hips target
        else planned.position.y -= std::max(0.0f, pelvisY - restPelvisHeight_);
        const math::Transform pelvis = root_ * planned;  // world
        const math::Quat turn = root_.rotation;
        world.moveKinematic(anchor_, math::Transform{pelvis.position, turn}, dt);
        world.setSupportTarget(support_, math::Vec3(0.0f), math::inverse(turn) * pelvis.rotation);
        // The hold's strength: eased back in after a recovery, the body's
        // strength, and the support under the feet (none in flight: the
        // body is ballistic then).
        if (supportShare_ < 1.0f) supportShare_ = std::min(1.0f, supportShare_ + dt / std::max(c.recoverTime, 1e-3f));
        const float onFeet = std::min(1.0f, queue_.supportWeight[0] + queue_.supportWeight[1]);
        // The jump's flight: the hold's joint is off (its down limit too),
        // the body flies on the speed the push gave it; the hold eases back
        // in from the landing.
        if (jumpPlanner_.airborne() != holdOff_) {
            holdOff_ = jumpPlanner_.airborne();
            world.setJointEnabled(support_, !holdOff_);
            if (!holdOff_) supportShare_ = 0.0f;
        }
        const float k = supportShare_ * strength_ * (pushing ? 1.0f : onFeet);
        world.setSupportLimits(support_, c.supportMaxForce * k, c.supportMaxTorque * k);
        // The real pelvis off the hold's target (level): small is nothing;
        // past medium for a while the planner is asked for a recovery step
        // (reported); past large the hold lets go (when the release is on).
        const math::Vec3 real = world.transform(humanBody_.parts()[0].body).position;
        const math::Vec3 off = real - pelvis.position;
        pelvisErrorNow_ = std::sqrt(off.x * off.x + off.z * off.z);
        pelvisErrorTime_ = pelvisErrorNow_ > c.errorMedium ? pelvisErrorTime_ + dt : 0.0f;
        pelvisError_ = pelvisErrorNow_ > c.errorLarge      ? PelvisError::Large
                       : pelvisErrorTime_ >= c.errorTime ? PelvisError::Medium
                                                         : PelvisError::Small;
        // Let go once the hips lean past the release angles, seen from the
        // centre of the feet, or the pelvis's error is large. After a fall
        // the check waits until the body is back up inside the limits with
        // the hold at full strength.
        math::Vec3 base(0.0f);
        int feet = 0;
        for (int f = 0; f < 2; ++f) {
            const int part = humanBody_.partOf(f == 0 ? joints_.legL.end : joints_.legR.end);
            if (part < 0) continue;
            base += world.transform(humanBody_.parts()[static_cast<size_t>(part)].body).position;
            ++feet;
        }
        if (feet == 0 || !c.releaseOn) return;
        base /= static_cast<float>(feet);
        const math::Vec3 lean = math::inverse(root_.rotation) * (real - base);  // character axes: +X left, +Z forward
        const float height = std::max(lean.y, 0.1f);
        const float forward = std::atan2(lean.z, height), side = std::atan2(lean.x, height);
        const float f = forward / std::max(forward >= 0.0f ? c.releaseForward : c.releaseBack, 1e-3f);
        const float l = side / std::max(side >= 0.0f ? c.releaseLeft : c.releaseRight, 1e-3f);
        const bool over = f * f + l * l > 1.0f || pelvisError_ == PelvisError::Large;
        if (!armed_) {
            armed_ = supportShare_ >= 1.0f && !over;
            return;
        }
        if (over) {
            world.setJointEnabled(support_, false);
            supportOn_ = false;
            restTime_ = 0.0f;
            LOG_INFO("human: lean {:.1f} deg forward, {:.1f} deg left, pelvis {:.2f} m off: the capsule lets go",
                     math::degrees(forward), math::degrees(side), pelvisErrorNow_);
        }
        return;
    }
    // Let go: the body falls on its own. At rest, the capsule goes under the
    // pelvis and the hold eases back in.
    restTime_ = math::length(humanBody_.centreOfMassVelocity(world)) < c.restSpeed ? restTime_ + dt : 0.0f;
    if (restTime_ < c.recoverWait) return;
    const math::Vec3 pelvis = world.transform(humanBody_.parts()[0].body).position;
    math::Vec3 feet(pelvis.x, root_.position.y, pelvis.z);
    feet.y = world.groundHeightAt(feet);
    world.placeCharacter(capsule_, feet);
    const math::Transform before = root_;
    root_.position = feet;
    previousRoot_ = root_;
    stepper_.rebase(before, root_);
    world.resetBody(anchor_, math::Transform{pelvis, root_.rotation});  // the plan starts at the landed pelvis
    // The plan starts over from the landed pose (no fixed get-up).
    humanBody_.boneModels(world, 1.0f, rig_, current_, root_, physicsModels_);
    rig_.modelToLocal(physicsModels_, physicsPose_);
    decider_.seed(physicsPose_);
    jumpPlanner_ = motion::JumpPlanner{};
    supportShare_ = 0.0f;
    world.setSupportLimits(support_, 0.0f, 0.0f);
    world.setJointEnabled(support_, true);
    holdOff_ = false;
    supportOn_ = true;
    armed_ = false;
    restTime_ = 0.0f;
    LOG_INFO("human: at rest, the capsule takes the body again");
}

math::Vec3 Human::centreOfMass() {
    rig_.localToModel(current_, models_);
    return root_.transformPoint(modelCentreOfMass());
}

void Human::setPoseGoal(motion::Goal goal, math::Vec3 world, math::Quat rotation, bool turned) {
    if (!physics_) return;
    const int g = static_cast<int>(goal);
    if (g < 0 || g >= 7) return;
    phase_ = Phase::Pose;
    poseLast_ = g;
    posePosition_[g] = world;
    poseRotation_[g] = math::normalize(rotation);
    poseTurned_[g] = turned;
}

void Human::setPoseHand(bool left, math::Vec3 world) {
    setPoseGoal(left ? motion::Goal::HandL : motion::Goal::HandR, world, math::identityQuat(), false);
}

void Human::setPoseFoot(motion::Foot foot, math::Vec3 world) {
    setPoseGoal(foot == motion::Foot::Left ? motion::Goal::FootL : motion::Goal::FootR, world, math::identityQuat(), false);
}

int Human::goalJoint(motion::Goal goal) const {
    switch (goal) {
    case motion::Goal::HandL: return joints_.armL.end;
    case motion::Goal::HandR: return joints_.armR.end;
    case motion::Goal::FootL: return joints_.legL.end;
    case motion::Goal::FootR: return joints_.legR.end;
    case motion::Goal::Hips: return joints_.pelvis;
    case motion::Goal::Chest: return joints_.chest;
    case motion::Goal::Head: return joints_.head;
    }
    return -1;
}

void Human::resetPhysicsBody() {
    if (!physics_ || !humanBody_.ready()) return;
    rig_.localToModel(current_, models_);
    humanBody_.reset(*physics_, models_, root_);
    physics_->resetBody(anchor_, math::Transform{(root_ * humanBody_.partPose(0, models_)).position, root_.rotation});
    physics_->setJointEnabled(support_, true);
    holdOff_ = false;
    supportOn_ = armed_ = true;
    supportShare_ = 1.0f;
    restTime_ = 0.0f;
    strength_ = 1.0f;
    pelvisError_ = PelvisError::Small;
    pelvisErrorNow_ = pelvisErrorTime_ = 0.0f;
    jumpPlanner_ = motion::JumpPlanner{};
    decider_.seed(current_);  // the plan starts over from the standing pose
    const physics::CharacterSettings& c = physics_->settings().character;
    physics_->setSupportLimits(support_, c.supportMaxForce, c.supportMaxTorque);
}

void Human::clearPose() {
    if (phase_ != Phase::Pose) return;
    phase_ = Phase::Idle;
    poseLast_ = -1;
    for (bool& turned : poseTurned_) turned = false;
    plantFeet();
}

bool Human::physicalModels(const physics::PhysicsWorld& world, float alpha, std::vector<math::Mat4>& models) const {
    if (!physics_ || !humanBody_.ready() || models.size() != static_cast<size_t>(rig_.numJoints())) return false;
    // A bone without a body rides its parent as planned.
    humanBody_.boneModels(world, alpha, rig_, current_, interpolatedRoot(alpha), models);
    return true;
}

void Human::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
    