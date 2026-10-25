#include "agents/fielder.h"

#include "core/config.h"
#include "motion/reach.h"

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

} // namespace

bool Fielder::init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion) {
    def_ = def;
    jointsConfig_ = &joints;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);
    previousRoot_ = root_;
    walker_.reset(def.spawnYaw);
    gait_ = motion::Gait::Walk;
    lean_ = math::Vec2(0.0f);
    accel_ = 0.0f;
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
    reposition_.softBand = joints.getFloat("reposition.softBand", reposition_.softBand);
    stanceBlendHeight_ = joints.getFloat("reposition.stanceBlendHeight", stanceBlendHeight_);
    reach_.armReach = joints.getFloat("reach.armReach", reach_.armReach);
    reach_.wristAboveTarget = joints.getFloat("reach.wristAboveTarget", reach_.wristAboveTarget);
    reach_.maxSpeed = joints.getFloat("reach.maxSpeed", reach_.maxSpeed);
    // Weights are normalised so each set sums to 1.
    auto weights = [&](const char* key, math::Vec3 v) {
        v = joints.getVec3(key, v);
        v = math::Vec3(std::max(v.x, 0.0f), std::max(v.y, 0.0f), std::max(v.z, 0.0f));
        const float sum = v.x + v.y + v.z;
        return sum > 0.0f ? v / sum : math::Vec3(1.0f / 3.0f);
    };
    reach_.spineBend = weights("reach.spineBend", reach_.spineBend);
    reach_.spineTwist = weights("reach.spineTwist", reach_.spineTwist);
    reach_.spineIterations = joints.getInt("reach.spineIterations", reach_.spineIterations);
    auto degs = [&](const char* key, math::Vec3 radiansV) {
        return math::radians(joints.getVec3(key, math::degrees(radiansV)));
    };
    reach_.pelvisMin = degs("reach.pelvisMinDeg", reach_.pelvisMin);
    reach_.pelvisMax = degs("reach.pelvisMaxDeg", reach_.pelvisMax);
    return setHeight(height);
}

void Fielder::loadLocomotion(const core::Config& c) {
    // Load-time only (builds path strings). "jog" starts from the walk values,
    // "run" from the jog values.
    auto load = [&](const std::string& block, motion::WalkSettings& w) {
        auto get = [&](const char* key, float v) { return c.getFloat(block + "." + key, v); };
        auto deg = [&](const char* key, float radians) { return math::radians(get(key, math::degrees(radians))); };
        w.speed = get("speed", w.speed);
        w.backSpeed = get("backSpeed", w.backSpeed);
        w.sideSpeed = get("sideSpeed", w.sideSpeed);
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
        w.flightLift = get("flightLift", w.flightLift);
        w.heelRise = get("heelRise", w.heelRise);
        w.toeOffPitch = deg("toeOffPitchDeg", w.toeOffPitch);
        w.landPitch = deg("landPitchDeg", w.landPitch);
        w.swingPeakAt = get("swingPeakAt", w.swingPeakAt);
        w.widthScale = get("widthScale", w.widthScale);
        w.pelvisTwist = deg("pelvisTwistDeg", w.pelvisTwist);
        w.chestCounter = get("chestCounter", w.chestCounter);
        w.pelvisDrop = get("pelvisDrop", w.pelvisDrop);
        w.flightCarry = get("flightCarry", w.flightCarry);
        w.sideFollow = get("sideFollow", w.sideFollow);
    };
    load("walk", walkSettings_);
    jogSettings_ = walkSettings_;
    load("jog", jogSettings_);
    runSettings_ = jogSettings_;
    load("run", runSettings_);
    control_ = shape_ = walkSettings_;
}

void Fielder::blendShape() {
    // Walk below walking speed, then walk -> jog -> run by the actual speed,
    // measured against each pace's speed in the direction of travel.
    const motion::WalkSettings* gaits[3] = {&walkSettings_, &jogSettings_, &runSettings_};
    const math::Vec2 local = walker_.localVelocity();
    const bool forward = local.y > std::fabs(local.x);
    auto ref = [&](const motion::WalkSettings& g) {
        return forward ? g.speed : std::fabs(local.x) > std::fabs(local.y) ? g.sideSpeed : g.backSpeed;
    };
    const float v = walker_.speed();
    shape_ = runSettings_;
    for (int i = 0; i < 3; ++i) {
        if (v > ref(*gaits[i])) continue;
        if (i == 0) shape_ = walkSettings_;
        else shape_ = motion::blendGait(*gaits[i - 1], *gaits[i],
                                        (v - ref(*gaits[i - 1])) / std::max(ref(*gaits[i]) - ref(*gaits[i - 1]), 0.1f));
        break;
    }
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
    models_.assign(static_cast<size_t>(rig_.numJoints()), math::Mat4(1.0f));
    pelvis_ = current_.locals[static_cast<size_t>(joints_.pelvis)].position;  // pelvis is the root joint
    rig_.localToModel(current_, models_);
    armLength_ = math::distance(modelPos(models_, joints_.armL.start), modelPos(models_, joints_.armL.mid)) +
                 math::distance(modelPos(models_, joints_.armL.mid), modelPos(models_, joints_.armL.end));
    if (joints_.toeL >= 0) {
        rig_.localToModel(current_, models_);
        const math::Vec3 d = modelPos(models_, joints_.toeL) - modelPos(models_, joints_.legL.end);
        ballDistance_ = math::length(math::Vec3(d.x, 0.0f, d.z));
    }
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
    reachError_ = 0.0f;
    const bool reachNow = reaching_ && walker_.speed() < reach_.maxSpeed;
    if (reachNow) reachBody();
    solveIk();
    swingArms(static_cast<float>(dt));
    stabiliseHead(static_cast<float>(dt));
    if (reachNow) reachArm();
    limits_.apply(current_);
}

void Fielder::walk(float dt) {
    // Move and turn the root; planted feet stay where they are in the world.
    control_ = gait_ == motion::Gait::Run ? runSettings_ : gait_ == motion::Gait::Jog ? jogSettings_ : walkSettings_;
    walker_.tick(dt, moveInput_, face_, faceYaw_, control_);
    blendShape();
    const motion::WalkSettings& w = shape_;
    const math::Transform from = root_;
    root_.position += walker_.velocity() * dt;
    root_.rotation = math::fromAxisAngle(math::kUp, walker_.yaw());
    stepper_.rebase(from, root_);

    const math::Vec3 velocity = math::inverse(root_).transformDirection(walker_.velocity());
    // Below walking speed, steps keep the walk's rhythm (full size from the
    // first step and while stopping); faster, they follow the actual speed.
    const float walkSpeed = walkSettings_.speed;
    const float timingSpeed = walker_.speed() > walkSpeed ? walker_.speed() : std::min(walker_.gaitSpeed(), walkSpeed);
    stepper_.tickGait(dt, velocity, motion::gaitTiming(timingSpeed, rig_.height(), w), w.stepHeight,
                      w.settleDistance, gaitStyle());
}

motion::GaitStyle Fielder::gaitStyle() const {
    const motion::WalkSettings& w = shape_;
    motion::GaitStyle s;
    s.flightLift = w.flightLift;
    s.heelRise = w.heelRise;
    s.toeOffPitch = w.toeOffPitch;
    s.landPitch = w.landPitch;
    s.peakAt = w.swingPeakAt;
    s.ballDistance = ballDistance_;
    s.widthScale = w.widthScale;
    s.flightCarry = w.flightCarry;
    return s;
}

void Fielder::addGaitMotion(float dt) {
    const motion::WalkSettings& w = shape_;
    const float amount = math::clamp(walker_.speed() / std::max(w.speed, 0.1f), 0.0f, 1.0f);

    // Bob: see motion::gaitBob. Running also lowers the pelvis so the stance
    // knee flexes (about 40 deg mid-stance). Sway: toward the stance foot (+X is left).
    // Left leg position from the gait: it travels back to front while it
    // swings, front to back while the right swings. Held when standing.
    const int swing = stepper_.swingFoot();
    const float sweep = std::cos(math::kPi * stepper_.swingProgress());
    if (swing == static_cast<int>(motion::Foot::Left)) legPhase_ = -sweep;
    else if (swing == static_cast<int>(motion::Foot::Right)) legPhase_ = sweep;

    const float arc = swing >= 0 ? std::sin(math::kPi * stepper_.swingProgress()) : 0.0f;
    const float side = swing == static_cast<int>(motion::Foot::Right) ? 1.0f : -1.0f;
    // Between steps, hold the value at a step boundary (continuous for walk and run).
    const float bob = motion::gaitBob(swing >= 0 ? stepper_.swingProgress() : 0.0f, w.flightFraction);
    const math::Vec3 offset(side * w.sway * amount * arc, amount * (w.bob * bob - w.pelvisDrop), 0.0f);

    // Lean forward with speed and acceleration, and into turns (about +Z,
    // positive tips the body toward -X, the right).
    const float forward = math::dot(math::inverse(root_).transformDirection(walker_.velocity()), math::kForward);
    // Acceleration steps (to 0 on reaching speed); smoothing it before the
    // lean filter makes the lean ease out instead of kinking.
    const float follow = std::min(1.0f, w.leanRate * dt);
    accel_ += (walker_.acceleration() - accel_) * follow;
    const math::Vec2 want(w.leanPerSpeed * forward + w.leanPerAccel * accel_,
                          -w.turnLean * walker_.yawRate() * walker_.speed());
    lean_ += (math::clamp(want, math::Vec2(-w.maxLean), math::Vec2(w.maxLean)) - lean_) * follow;

    math::Transform& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)];
    pelvis.position = pelvis_ + offset;
    // Pelvis turns toward the forward leg (+X left: a positive turn about +Y
    // takes the left hip back); the chest turns back against it.
    const float twist = -w.pelvisTwist * amount * legPhase_;
    pelvis.rotation = math::fromAxisAngle(math::kRight, lean_.x) * math::fromAxisAngle(math::kForward, lean_.y) *
                      math::fromAxisAngle(math::kUp, twist) * pelvis.rotation;
    if (joints_.chest >= 0) {
        math::Quat& chest = current_.locals[static_cast<size_t>(joints_.chest)].rotation;
        chest = math::fromAxisAngle(math::kUp, -w.chestCounter * twist) * chest;
    }
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
    // Running, both feet are in the air in flight, so the weight follows the
    // gait phase instead (stance foot, handing over to the landing foot).
    float stance = math::clamp(0.5f + 0.5f * (left.foot.y - right.foot.y) / stanceBlendHeight_, 0.0f, 1.0f);
    const float phase = stepper_.rightWeight(shape_.flightFraction);
    if (phase >= 0.0f) stance = math::lerp(stance, phase, motion::runShare(shape_.flightFraction));
    motion::RepositionSettings reposition = reposition_;
    reposition.sideFollow = shape_.sideFollow;  // runners barely shift sideways
    // Moving, the root already travels smoothly; chasing the stance foot
    // (which slides back every step) would make the body lag and surge.
    reposition.forwardFollow = 1.0f - math::clamp(walker_.speed() / std::max(shape_.speed, 0.1f), 0.0f, 1.0f);
    reposition.restZ = restPelvis.z;
    pelvis_ = motion::repositionBody(pelvis_, left, right, rig_.legLength(), stance, reposition, dt);
    current_.locals[static_cast<size_t>(joints_.pelvis)].position = pelvis_;
}

void Fielder::solveIk() {
    const math::Transform toModel = math::inverse(root_);
    auto model = [&](math::Vec3 w) { return toModel.transformPoint(w); };

    rig_.localToModel(current_, models_);
    ik::solveTwoBone(joints_.legL, models_, model(targets_.footL), ik_.kneePole, ik_.soften, current_);
    ik::solveTwoBone(joints_.legR, models_, model(targets_.footR), ik_.kneePole, ik_.soften, current_);
    limits_.apply(current_);

    // Feet keep the stepper's pitch in character space (flat when walking)
    // whatever the shin does; the toes bend to stay down while the heel rises.
    rig_.localToModel(current_, models_);
    const ik::TwoBoneChain* legs[2] = {&joints_.legL, &joints_.legR};
    const int toes[2] = {joints_.toeL, joints_.toeR};
    for (int i = 0; i < 2; ++i) {
        const motion::Foot f = static_cast<motion::Foot>(i);
        math::Quat shin = modelRot(models_, legs[i]->mid);
        current_.locals[static_cast<size_t>(legs[i]->end)].rotation =
            math::normalize(math::inverse(shin) * math::fromAxisAngle(math::kRight, stepper_.footPitch(f)));
        if (toes[i] >= 0)
            current_.locals[static_cast<size_t>(toes[i])].rotation = math::fromAxisAngle(math::kRight, stepper_.toeBend(f));
    }
    limits_.apply(current_);
}

void Fielder::swingArms(float dt) {
    const motion::WalkSettings& w = shape_;

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
    const motion::WalkSettings& w = shape_;

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

void Fielder::rotateInModel(int joint, math::Quat rotation) {
    // Model rotation M = P * L, so a model-space rotation R becomes the local
    // L' = P^-1 * R * P * L.
    const math::Quat parent = modelRot(models_, rig_.parent(joint));
    math::Quat& local = current_.locals[static_cast<size_t>(joint)].rotation;
    local = math::normalize(math::inverse(parent) * rotation * parent * local);
    local = ik::clampRotation(local, limits_.limit(joint));
}

math::Vec3 Fielder::reachGoal() const {
    return math::inverse(root_).transformPoint(reachTarget_) + math::kUp * reach_.wristAboveTarget;
}

void Fielder::reachBody() {
    // Each stage takes only the error the stages before it could not remove:
    // spine (2.2), pelvis (2.3) here; clavicle and arm in reachArm(). Proximal
    // joints are posed before the arm IK (BIOMECHANICS 9.1-9.2).
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;

    // The nearer shoulder's hand reaches.
    rig_.localToModel(current_, models_);
    reachLeft_ = math::distance(modelPos(models_, joints_.armL.start), target) <=
                 math::distance(modelPos(models_, joints_.armR.start), target);
    const int shoulder = reachLeft_ ? joints_.armL.start : joints_.armR.start;

    // Spine: swing the shoulder toward the target about the spine base (law
    // of cosines), split into bend and twist (about the character's up), each
    // shared over spine_01, spine_02 and chest by its weights (9.3: twist
    // mostly thoracic, bend mostly lumbar), every bone clamped to its limit.
    if (joints_.spine[0] >= 0 && joints_.spine[2] >= 0) {
        const float bendShare[3] = {reach_.spineBend.x, reach_.spineBend.y, reach_.spineBend.z};
        const float twistShare[3] = {reach_.spineTwist.x, reach_.spineTwist.y, reach_.spineTwist.z};
        for (int it = 0; it < reach_.spineIterations; ++it) {
            const motion::Swing swing = motion::swingToward(modelPos(models_, joints_.spine[0]),
                                                            modelPos(models_, shoulder), target, armReach);
            if (swing.angle <= 0.0f) break;
            math::Quat bend, twist;
            math::swingTwist(math::fromAxisAngle(swing.axis, swing.angle), math::kUp, bend, twist);
            for (int i = 0; i < 3; ++i) {
                if (joints_.spine[i] < 0) continue;
                const math::Quat id = math::identityQuat();
                rotateInModel(joints_.spine[i],
                              math::slerp(id, bend, bendShare[i]) * math::slerp(id, twist, twistShare[i]));
                rig_.localToModel(current_, models_);
            }
        }
    }

    // Pelvis: tilt forward (hip flexion), turn and side-tilt about the hip
    // centre for what the spine could not reach, within its limits. The
    // pelvis is the root joint, so its local rotation is in character space;
    // the leg IK that runs next keeps the feet planted.
    const motion::Swing swing =
        motion::swingToward(modelPos(models_, joints_.pelvis), modelPos(models_, shoulder), target, armReach);
    if (swing.angle > 0.0f) {
        math::Quat& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)].rotation;
        const math::Quat turn = motion::clampRotationVector(math::fromAxisAngle(swing.axis, swing.angle),
                                                            reach_.pelvisMin, reach_.pelvisMax);
        pelvis = math::normalize(turn * pelvis);
    }
}

void Fielder::reachArm() {
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;
    const ik::TwoBoneChain& arm = reachLeft_ ? joints_.armL : joints_.armR;
    const int clavicle = reachLeft_ ? joints_.clavicleL : joints_.clavicleR;
    rig_.localToModel(current_, models_);

    // Clavicle: protract/elevate so the shoulder moves toward the target.
    if (clavicle >= 0) {
        const motion::Swing swing =
            motion::swingToward(modelPos(models_, clavicle), modelPos(models_, arm.start), target, armReach);
        if (swing.angle > 0.0f) rotateInModel(clavicle, math::fromAxisAngle(swing.axis, swing.angle));
    }

    // Arm: from the rest pose (no walk swing), two-bone IK toward the target,
    // never further than armReach so the elbow stays bent. Elbows point back
    // and out, mirrored for the right arm (+X is left).
    const rig::Pose& rest = rig_.restPose();
    current_.locals[static_cast<size_t>(arm.start)].rotation = rest.locals[static_cast<size_t>(arm.start)].rotation;
    current_.locals[static_cast<size_t>(arm.mid)].rotation = rest.locals[static_cast<size_t>(arm.mid)].rotation;
    rig_.localToModel(current_, models_);
    const math::Vec3 shoulder = modelPos(models_, arm.start);
    const math::Vec3 pole = reachLeft_ ? ik_.elbowPole : math::Vec3(-ik_.elbowPole.x, ik_.elbowPole.y, ik_.elbowPole.z);
    ik::solveTwoBone(arm, models_, motion::clampToReach(shoulder, target, armReach), pole, ik_.soften, current_);

    // Leftover for the next stages (body, step: Steps 2.4-2.5).
    reachError_ = std::max(0.0f, math::distance(shoulder, target) - armReach);
}

void Fielder::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
