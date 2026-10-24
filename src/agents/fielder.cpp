#include "agents/fielder.h"

#include "core/config.h"

#include <algorithm>
#include <cmath>

namespace agents {

namespace {

math::Vec3 modelPos(const std::vector<math::Mat4>& models, int joint) {
    return math::Vec3(models[static_cast<size_t>(joint)][3]);
}

math::Quat modelRot(const std::vector<math::Mat4>& models, int joint) {
    return math::toTransform(models[static_cast<size_t>(joint)]).rotation;
}

} // namespace

bool Fielder::init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion) {
    def_ = def;
    jointsConfig_ = &joints;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);
    previousRoot_ = root_;
    walker_.reset(def.spawnYaw);
    lean_ = math::Vec2(0.0f);
    legPhase_ = armAmount_ = headYaw_ = 0.0f;
    loadLocomotion(locomotion);

    ik_.kneePole = joints.getVec3("ik.kneePole", ik_.kneePole);
    ik_.elbowPole = joints.getVec3("ik.elbowPole", ik_.elbowPole);
    ik_.soften = joints.getFloat("ik.soften", ik_.soften);
    ik_.chestAimWeight = joints.getFloat("ik.chestAimWeight", ik_.chestAimWeight);
    ik_.headAimWeight = joints.getFloat("ik.headAimWeight", ik_.headAimWeight);
    reposition_.standReach = joints.getFloat("reposition.standReach", reposition_.standReach);
    reposition_.minReach = joints.getFloat("reposition.minReach", reposition_.minReach);
    reposition_.maxReach = joints.getFloat("reposition.maxReach", reposition_.maxReach);
    reposition_.followRate = joints.getFloat("reposition.followRate", reposition_.followRate);
    stanceBlendHeight_ = joints.getFloat("reposition.stanceBlendHeight", stanceBlendHeight_);
    return setHeight(height);
}

void Fielder::loadLocomotion(const core::Config& c) {
    motion::WalkSettings& w = walkSettings_;
    auto deg = [&](const char* path, float radians) { return math::radians(c.getFloat(path, math::degrees(radians))); };
    w.speed = c.getFloat("walk.speed", w.speed);
    w.backSpeed = c.getFloat("walk.backSpeed", w.backSpeed);
    w.sideSpeed = c.getFloat("walk.sideSpeed", w.sideSpeed);
    w.acceleration = c.getFloat("walk.acceleration", w.acceleration);
    w.deceleration = c.getFloat("walk.deceleration", w.deceleration);
    w.turnRate = deg("walk.turnRateDeg", w.turnRate);
    w.stepK = c.getFloat("walk.stepK", w.stepK);
    w.refSpeed = c.getFloat("walk.refSpeed", w.refSpeed);
    w.swingFraction = c.getFloat("walk.swingFraction", w.swingFraction);
    w.minSwing = c.getFloat("walk.minSwing", w.minSwing);
    w.maxSwing = c.getFloat("walk.maxSwing", w.maxSwing);
    w.stepHeight = c.getFloat("walk.stepHeight", w.stepHeight);
    w.settleDistance = c.getFloat("walk.settleDistance", w.settleDistance);
    w.bob = c.getFloat("walk.bob", w.bob);
    w.sway = c.getFloat("walk.sway", w.sway);
    w.leanPerSpeed = deg("walk.leanPerSpeedDeg", w.leanPerSpeed);
    w.leanPerAccel = deg("walk.leanPerAccelDeg", w.leanPerAccel);
    w.turnLean = deg("walk.turnLeanDeg", w.turnLean);
    w.maxLean = deg("walk.maxLeanDeg", w.maxLean);
    w.leanRate = c.getFloat("walk.leanRate", w.leanRate);
    w.armSwing = deg("walk.armSwingDeg", w.armSwing);
    w.armBackBias = deg("walk.armBackBiasDeg", w.armBackBias);
    w.elbowBias = deg("walk.elbowBiasDeg", w.elbowBias);
    w.elbowSwing = deg("walk.elbowSwingDeg", w.elbowSwing);
    w.armRate = c.getFloat("walk.armRate", w.armRate);
    w.headStabilise = c.getFloat("walk.headStabilise", w.headStabilise);
    w.headLead = deg("walk.headLeadDeg", w.headLead);
    w.headRate = c.getFloat("walk.headRate", w.headRate);
}

bool Fielder::setHeight(float height) {
    if (!rig_.build(def_, height)) return false;
    if (!jointsConfig_ || !limits_.load(*jointsConfig_, rig_)) return false;

    // Knee flexion is +X, elbow flexion -X; ozz's mid axis opens the joint.
    auto chain = [&](const char* a, const char* b, const char* c, math::Vec3 axis) {
        return ik::TwoBoneChain{rig_.findJoint(a), rig_.findJoint(b), rig_.findJoint(c), axis};
    };
    joints_.pelvis = rig_.findJoint("pelvis");
    joints_.chest = rig_.findJoint("chest");
    joints_.head = rig_.findJoint("head");
    joints_.legL = chain("thigh_l", "shin_l", "foot_l", {-1, 0, 0});
    joints_.legR = chain("thigh_r", "shin_r", "foot_r", {-1, 0, 0});
    joints_.armL = chain("upper_arm_l", "forearm_l", "hand_l", {1, 0, 0});
    joints_.armR = chain("upper_arm_r", "forearm_r", "hand_r", {1, 0, 0});

    current_ = rig_.restPose();
    previous_ = current_;
    models_.assign(static_cast<size_t>(rig_.numJoints()), math::Mat4(1.0f));
    pelvis_ = current_.locals[static_cast<size_t>(joints_.pelvis)].position;  // pelvis is the root joint
    plantFeet();
    return true;
}

void Fielder::plantFeet() {
    // Feet planted where they stand in the rest pose (character space).
    rig_.localToModel(rig_.restPose(), models_);
    stepper_.reset(modelPos(models_, joints_.legL.end), modelPos(models_, joints_.legR.end));
}

void Fielder::tick(double dt) {
    if (joints_.pelvis < 0) return;  // not built yet (SimWorld ticks once while configuring)
    previous_.locals = current_.locals;  // same size: no allocation
    previousRoot_ = root_;
    current_.locals = rig_.restPose().locals;
    walk(static_cast<float>(dt));
    targets_.footL = root_.transformPoint(stepper_.foot(motion::Foot::Left));
    targets_.footR = root_.transformPoint(stepper_.foot(motion::Foot::Right));
    repositionBody(static_cast<float>(dt));
    addGaitMotion(static_cast<float>(dt));
    solveIk();
    swingArms(static_cast<float>(dt));
    stabiliseHead(static_cast<float>(dt));
    limits_.apply(current_);
}

void Fielder::walk(float dt) {
    // Move and turn the root; planted feet stay where they are in the world.
    walker_.tick(dt, moveInput_, face_, faceYaw_, walkSettings_);
    const math::Transform from = root_;
    root_.position += walker_.velocity() * dt;
    root_.rotation = math::fromAxisAngle(math::kUp, walker_.yaw());
    stepper_.rebase(from, root_);

    const math::Vec3 velocity = math::inverse(root_).transformDirection(walker_.velocity());
    stepper_.tickGait(dt, velocity, motion::gaitTiming(walker_.gaitSpeed(), rig_.height(), walkSettings_),
                      walkSettings_.stepHeight, walkSettings_.settleDistance);
}

void Fielder::addGaitMotion(float dt) {
    const motion::WalkSettings& w = walkSettings_;
    const float amount = math::clamp(walker_.speed() / std::max(w.speed, 0.1f), 0.0f, 1.0f);

    // Bob: lowest in double support, highest mid-swing (stance leg under the
    // hip). Sway: toward the stance foot (+X is left).
    const int swing = stepper_.swingFoot();
    const float arc = swing >= 0 ? std::sin(math::kPi * stepper_.swingProgress()) : 0.0f;
    const float side = swing == static_cast<int>(motion::Foot::Right) ? 1.0f : -1.0f;
    const math::Vec3 offset(side * w.sway * amount * arc, w.bob * amount * (arc - 0.5f), 0.0f);

    // Lean forward with speed and acceleration, and into turns (about +Z,
    // positive tips the body toward -X, the right).
    const float forward = math::dot(math::inverse(root_).transformDirection(walker_.velocity()), math::kForward);
    const math::Vec2 want(w.leanPerSpeed * forward + w.leanPerAccel * walker_.acceleration(),
                          -w.turnLean * walker_.yawRate() * walker_.speed());
    lean_ += (math::clamp(want, math::Vec2(-w.maxLean), math::Vec2(w.maxLean)) - lean_) * std::min(1.0f, w.leanRate * dt);

    math::Transform& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)];
    pelvis.position = pelvis_ + offset;
    pelvis.rotation = math::fromAxisAngle(math::kRight, lean_.x) * math::fromAxisAngle(math::kForward, lean_.y) *
                      pelvis.rotation;
}

void Fielder::repositionBody(float dt) {
    // Hip offsets from the rest pose; the pelvis is the root joint, so its
    // local position is in character space.
    rig_.localToModel(rig_.restPose(), models_);
    const math::Vec3 restPelvis = modelPos(models_, joints_.pelvis);
    const math::Transform toModel = math::inverse(root_);
    motion::LegSupport left{modelPos(models_, joints_.legL.start) - restPelvis, toModel.transformPoint(targets_.footL)};
    motion::LegSupport right{modelPos(models_, joints_.legR.start) - restPelvis, toModel.transformPoint(targets_.footR)};

    // The lower foot carries the body; even weight when both are level.
    float stance = math::clamp(0.5f + 0.5f * (left.foot.y - right.foot.y) / stanceBlendHeight_, 0.0f, 1.0f);
    pelvis_ = motion::repositionBody(pelvis_, left, right, rig_.legLength(), stance, reposition_, dt);
    current_.locals[static_cast<size_t>(joints_.pelvis)].position = pelvis_;
}

void Fielder::solveIk() {
    const math::Transform toModel = math::inverse(root_);
    auto model = [&](math::Vec3 w) { return toModel.transformPoint(w); };

    rig_.localToModel(current_, models_);
    ik::solveTwoBone(joints_.legL, models_, model(targets_.footL), ik_.kneePole, ik_.soften, current_);
    ik::solveTwoBone(joints_.legR, models_, model(targets_.footR), ik_.kneePole, ik_.soften, current_);
    limits_.apply(current_);

    // Feet stay flat (rest orientation in character space) whatever the shin does.
    rig_.localToModel(current_, models_);
    for (const ik::TwoBoneChain* leg : {&joints_.legL, &joints_.legR}) {
        math::Quat shin = modelRot(models_, leg->mid);
        current_.locals[static_cast<size_t>(leg->end)].rotation = math::normalize(math::inverse(shin));
    }
    limits_.apply(current_);
}

void Fielder::swingArms(float dt) {
    const motion::WalkSettings& w = walkSettings_;

    // Left leg position from the gait: it travels back to front while it
    // swings, front to back while the right swings. Held when standing.
    const int swing = stepper_.swingFoot();
    const float sweep = std::cos(math::kPi * stepper_.swingProgress());
    if (swing == static_cast<int>(motion::Foot::Left)) legPhase_ = -sweep;
    else if (swing == static_cast<int>(motion::Foot::Right)) legPhase_ = sweep;

    // Full swing walking forward, little when sidestepping or backing up.
    const float forward = math::dot(math::inverse(root_).transformDirection(walker_.velocity()), math::kForward);
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

void Fielder::stabiliseHead(float dt) {
    const motion::WalkSettings& w = walkSettings_;

    // Lead turns: the head looks toward the requested heading before the body.
    float want = 0.0f;
    if (face_) {
        const float diff = std::remainder(faceYaw_ - walker_.yaw(), math::kTwoPi);
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

void Fielder::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
