#include "agents/fielder.h"

#include "core/config.h"
#include "core/log.h"
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
    // The fielder starts with the ball in the right hand.
    phase_ = Phase::Idle;
    holding_ = true;
    reachLeft_ = bothHands_ = false;
    reach_.armReach = joints.getFloat("reach.armReach", reach_.armReach);
    reach_.wristOffset = joints.getFloat("reach.wristOffset", reach_.wristOffset);
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
    reach_.supportRadius = joints.getFloat("reach.supportRadius", reach_.supportRadius);
    reach_.balanceGain = joints.getFloat("reach.balanceGain", reach_.balanceGain);
    reach_.maxHipShift = joints.getFloat("reach.maxHipShift", reach_.maxHipShift);
    reach_.balancePasses = joints.getInt("reach.balancePasses", reach_.balancePasses);
    reach_.reachTime = joints.getFloat("reach.reachTime", reach_.reachTime);
    reach_.holdTime = joints.getFloat("reach.holdTime", reach_.holdTime);
    reach_.returnTime = joints.getFloat("reach.returnTime", reach_.returnTime);
    reach_.gripTolerance = joints.getFloat("reach.gripTolerance", reach_.gripTolerance);
    reach_.giveUpTime = joints.getFloat("reach.giveUpTime", reach_.giveUpTime);
    reach_.ballRadius = joints.getFloat("reach.ballRadius", reach_.ballRadius);
    reach_.relaxedCurl = joints.getFloat("reach.relaxedCurl", reach_.relaxedCurl);
    reach_.palmCentre = joints.getFloat("reach.palmCentre", reach_.palmCentre);
    reach_.palmThickness = joints.getFloat("reach.palmThickness", reach_.palmThickness);
    reach_.fingerCurl = degs("reach.fingerCurlDeg", reach_.fingerCurl);
    reach_.thumbCurl = degs("reach.thumbCurlDeg", reach_.thumbCurl);
    reach_.maxLunge = joints.getFloat("reach.maxLunge", reach_.maxLunge);
    reach_.lungeShort = joints.getFloat("reach.lungeShort", reach_.lungeShort);
    reach_.stepBeyond = joints.getFloat("reach.stepBeyond", reach_.stepBeyond);
    reach_.lungeDuration = joints.getFloat("reach.lungeDuration", reach_.lungeDuration);
    reach_.lungeHeight = joints.getFloat("reach.lungeHeight", reach_.lungeHeight);
    reach_.elbowPole = joints.getVec3("reach.elbowPole", reach_.elbowPole);
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
    const char* fingerNames[5] = {"index", "middle", "ring", "pinky", "thumb"};
    for (int side = 0; side < 2; ++side)
        for (int f = 0; f < 5; ++f)
            for (int k = 0; k < 3; ++k)
                joints_.fingers[side][f][k] = rig_.findJoint(std::string(fingerNames[f]) + "_0" + std::to_string(k + 1) +
                                                             (side == 0 ? "_l" : "_r"));
    // Curl axes: each finger bone turns toward the palm (palm normal -X on
    // the left, +X on the right) about cross(bone direction, palm normal); each
    // thumb bone turns toward the ball held in front of the palm, about
    // cross(bone direction, ball - bone start). Rest joint frames are aligned
    // with the character, so rest offsets are hand-space directions; the last
    // bone keeps its parent's axis.
    const rig::Pose& rest = rig_.restPose();
    for (int side = 0; side < 2; ++side) {
        const math::Vec3 palm(side == 0 ? -1.0f : 1.0f, 0.0f, 0.0f);
        const math::Vec3 ball = ballSpot(side == 0);
        for (int f = 0; f < 5; ++f) {
            math::Vec3 at(0.0f);  // bone start in hand space
            for (int k = 0; k < 3; ++k) {
                math::Vec3& axis = joints_.curlAxis[side][f][k];
                const int joint = joints_.fingers[side][f][k];
                const int child = k < 2 ? joints_.fingers[side][f][k + 1] : -1;
                if (joint >= 0) at += rest.locals[static_cast<size_t>(joint)].position;
                if (child < 0) {
                    axis = k > 0 ? joints_.curlAxis[side][f][k - 1] : math::Vec3(0.0f, 0.0f, -palm.x);
                    continue;
                }
                const math::Vec3 dir = math::safeNormalize(rest.locals[static_cast<size_t>(child)].position, -math::kUp);
                const math::Vec3 toward = f < 4 ? palm : math::safeNormalize(ball - at, palm);
                axis = math::safeNormalize(math::cross(dir, toward), math::Vec3(0.0f, 0.0f, -palm.x));
            }
        }
    }
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
    // Share of the mass that moves with the pelvis: all but the planted lower
    // legs and feet, and half of each thigh (it turns about the hip).
    float total = 0.0f, planted = 0.0f;
    for (float m : segmentMass_) total += m;
    for (const ik::TwoBoneChain* leg : {&joints_.legL, &joints_.legR})
        planted += 0.5f * segmentMass_[static_cast<size_t>(leg->start)] + segmentMass_[static_cast<size_t>(leg->mid)] +
                   segmentMass_[static_cast<size_t>(leg->end)];
    for (int toe : {joints_.toeL, joints_.toeR})
        if (toe >= 0) planted += segmentMass_[static_cast<size_t>(toe)];
    pelvisMassShare_ = total > 0.0f ? 1.0f - planted / total : 1.0f;
    const ik::TwoBoneChain& arm = joints_.armL;  // both arms weigh the same
    armMassShare_ = total > 0.0f ? (segmentMass_[static_cast<size_t>(arm.start)] + segmentMass_[static_cast<size_t>(arm.mid)] +
                                    segmentMass_[static_cast<size_t>(arm.end)]) / total
                                 : 0.0f;
    models_.assign(static_cast<size_t>(rig_.numJoints()), math::Mat4(1.0f));
    pelvis_ = current_.locals[static_cast<size_t>(joints_.pelvis)].position;  // pelvis is the root joint
    rig_.localToModel(current_, models_);
    restShoulderL_ = modelPos(models_, joints_.armL.start);
    restShoulderR_ = modelPos(models_, joints_.armR.start);
    restWristL_ = modelPos(models_, joints_.armL.end);
    restWristR_ = modelPos(models_, joints_.armR.end);
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
    updatePick(static_cast<float>(dt));
    // While the step-first lunge is in the air the body follows the feet the
    // usual way (repositionBody); the reach starts once the foot lands.
    const bool reachNow = phase_ != Phase::Idle && !(phase_ == Phase::Reach && pickTime_ <= 0.0f);
    if (reachNow) {
        reachSpine();
        reachPelvis();
        reachBody();
    }
    solveIk();
    if (reachNow && !stepper_.stepping()) rebalance();  // a swinging foot is no support
    swingArms(static_cast<float>(dt));
    stabiliseHead(static_cast<float>(dt));
    if (reachNow) {
        reachArm(reachLeft_, goal_);
        if (bothHands_) reachArm(!reachLeft_, otherGoal_);
    }
    // Fingers: closed around the ball once the hand is on it and while it is
    // held (walking and running too), relaxed otherwise.
    const bool closed = holding_ || (phase_ == Phase::Reach && holdTimer_ > 0.0f);
    grip(true, closed && (reachLeft_ || bothHands_) ? 1.0f : reach_.relaxedCurl);
    grip(false, closed && (!reachLeft_ || bothHands_) ? 1.0f : reach_.relaxedCurl);
    limits_.apply(current_);
    if (reachNow) {
        // Measured after the limits, so a clamped arm shows as a miss.
        rig_.localToModel(current_, models_);
        const int hand = reachLeft_ ? joints_.armL.end : joints_.armR.end;
        reachError_ = math::distance(modelPos(models_, hand), goal_);
        finishPick(static_cast<float>(dt));
    }
}

void Fielder::walk(float dt) {
    // Move and turn the root; planted feet stay where they are in the world.
    control_ = gait_ == motion::Gait::Run ? runSettings_ : gait_ == motion::Gait::Jog ? jogSettings_ : walkSettings_;
    // While picking the fielder stands (walk input waits for the return).
    const bool picking = phase_ != Phase::Idle;
    walker_.tick(dt, picking ? math::Vec2(0.0f) : moveInput_, face_ && !picking, faceYaw_, control_);
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
    if (picking) {
        // Only a lunge step moves a foot; back in gait mode after the pick,
        // the gait's settle steps it back to stance (the ideal standing spot).
        motion::StepSettings lunge;
        lunge.stepDuration = reach_.lungeDuration;
        lunge.stepHeight = reach_.lungeHeight;
        stepper_.tick(dt, math::Vec2(0.0f), lunge);
        return;
    }
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

    // Knees point where the pelvis faces: the pole turns with the pelvis's
    // twist about the vertical (not its forward tilt). With a fixed pole, a
    // pelvis turned while bent forward would abduct the thighs past their limit.
    math::Quat pelvisSwing, pelvisTurn;
    math::swingTwist(current_.locals[static_cast<size_t>(joints_.pelvis)].rotation, math::kUp, pelvisSwing, pelvisTurn);
    const math::Vec3 kneePole = pelvisTurn * ik_.kneePole;

    rig_.localToModel(current_, models_);
    ik::solveTwoBone(joints_.legL, models_, model(targets_.footL), kneePole, ik_.soften, current_);
    ik::solveTwoBone(joints_.legR, models_, model(targets_.footR), kneePole, ik_.soften, current_);
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

void Fielder::reachSpine() {
    // Stages run in recruitment order (BIOMECHANICS 9.1), each taking the
    // error the ones before it left: spine, pelvis, body here; clavicle and
    // arm in reachArm(), after the leg IK. One pass each, no retries.
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;
    rig_.localToModel(current_, models_);
    const int shoulder = reachLeft_ ? joints_.armL.start : joints_.armR.start;

    // Swing the shoulder toward the target about the spine base (law of
    // cosines), split into bend and twist (about the character's up), each
    // shared over spine_01, spine_02 and chest by its weights (9.3: twist
    // mostly thoracic, bend mostly lumbar), every bone clamped to its limit.
    if (joints_.spine[0] < 0 || joints_.spine[1] < 0 || joints_.spine[2] < 0) return;
    const float bendShare[3] = {reach_.spineBend.x, reach_.spineBend.y, reach_.spineBend.z};
    const float twistShare[3] = {reach_.spineTwist.x, reach_.spineTwist.y, reach_.spineTwist.z};
    for (int it = 0; it < reach_.spineIterations; ++it) {
        const motion::Swing swing = motion::swingToward(modelPos(models_, joints_.spine[0]),
                                                        modelPos(models_, shoulder), target, armReach);
        math::Quat bend, twist;
        math::swingTwist(math::fromAxisAngle(swing.axis, swing.angle), math::kUp, bend, twist);
        for (int i = 0; i < 3; ++i) {
            const math::Quat id = math::identityQuat();
            rotateInModel(joints_.spine[i], math::slerp(id, bend, bendShare[i]) * math::slerp(id, twist, twistShare[i]));
            rig_.localToModel(current_, models_);
        }
    }
}

void Fielder::reachPelvis() {
    // Tilt forward (hip flexion), turn and side-tilt about the hip centre for
    // what the spine could not reach: the same swing, clamped to the pelvis
    // limits. The pelvis is the root joint, so its rotation is in character space.
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;
    const int shoulder = reachLeft_ ? joints_.armL.start : joints_.armR.start;
    const motion::Swing swing =
        motion::swingToward(modelPos(models_, joints_.pelvis), modelPos(models_, shoulder), target, armReach);
    math::Quat& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)].rotation;
    pelvis = motion::clampRotationVector(math::normalize(math::fromAxisAngle(swing.axis, swing.angle) * pelvis),
                                         reach_.pelvisMin, reach_.pelvisMax);
    rig_.localToModel(current_, models_);
}

void Fielder::reachBody() {
    // RepositionBody driven by the hand (Step 2.4), on top of the foot
    // repositioning (pelvis_ is not changed, so both apply): the body moves
    // toward the target by what is left (body += dir * error), the hips shift
    // back against the centre of mass's overhang (counterbalance, x gain, at
    // most maxHipShift), and the reposition leg clamp keeps both planted feet
    // within reach (no squat limit). rebalance() then corrects the balance
    // exactly after the leg IK.
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;
    const int shoulder = reachLeft_ ? joints_.armL.start : joints_.armR.start;
    math::Vec3& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)].position;

    const math::Vec3 toTarget = target - modelPos(models_, shoulder);
    const float distance = math::length(toTarget);
    const math::Vec3 move = toTarget * (std::max(0.0f, distance - armReach) / std::max(distance, math::kEpsilon));
    // Counterbalance estimate: a pelvis move carries pelvisMassShare_ of the
    // mass; balanceOverhang() gives the pelvis move that cancels the overhang.
    math::Vec3 shift = -balanceOverhang(move) * reach_.balanceGain;
    const float shiftLength = math::length(shift);
    if (shiftLength > reach_.maxHipShift) shift *= reach_.maxHipShift / shiftLength;
    pelvis = clampPelvisToLegs(pelvis + move + shift);
}

void Fielder::rebalance() {
    // Balance (BIOMECHANICS 3, 9.5), measured rather than predicted: a deep
    // squat swings the knees forward, which no estimate sees. Each pass takes
    // the actual centre of mass after the leg IK, moves the pelvis to cancel
    // its overhang beyond the feet, clamps to the legs and solves them again.
    math::Vec3& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)].position;
    for (int i = 0; i < reach_.balancePasses; ++i) {
        rig_.localToModel(current_, models_);
        pelvis = clampPelvisToLegs(pelvis - balanceOverhang(math::Vec3(0.0f)));
        solveIk();
    }
}

math::Vec3 Fielder::balanceOverhang(math::Vec3 pelvisMove) {
    // Centre of mass (models_ current) plus the reaching arm, which is posed
    // last: its mass will sit about midway between the shoulder and its reach
    // point. A pelvis move shifts it by pelvisMassShare_ of the move.
    const math::Vec3 target = reachGoal();
    const float armReach = reach_.armReach * armLength_;
    const ik::TwoBoneChain& arm = reachLeft_ ? joints_.armL : joints_.armR;
    const math::Vec3 shoulder = modelPos(models_, arm.start);
    const math::Vec3 armNow = 0.5f * (shoulder + modelPos(models_, arm.end));
    const math::Vec3 armThen = 0.5f * (shoulder + motion::clampToReach(shoulder, target, armReach));
    const math::Vec3 com = modelCentreOfMass() + (armThen - armNow) * armMassShare_ + pelvisMove * pelvisMassShare_;

    // Base of support: capsule around the foot centres (ankle to mid-foot).
    const math::Transform toModel = math::inverse(root_);
    const math::Vec3 half = 0.5f * math::kForward * ballDistance_;
    const math::Vec3 footL = toModel.transformPoint(targets_.footL) + half;
    const math::Vec3 footR = toModel.transformPoint(targets_.footR) + half;
    const math::Vec2 e = motion::supportExcess(math::Vec2(com.x, com.z), math::Vec2(footL.x, footL.z),
                                               math::Vec2(footR.x, footR.z), reach_.supportRadius);
    return math::Vec3(e.x, 0.0f, e.y) / pelvisMassShare_;
}

math::Vec3 Fielder::clampPelvisToLegs(math::Vec3 pelvis) {
    // The reposition leg clamp with both feet planted and no squat limit;
    // legs stay within the IK soften ratio so the ankles reach their targets.
    motion::RepositionSettings legs = reposition_;
    legs.minReach = 0.0f;
    legs.maxReach = std::min(reposition_.maxReach, ik_.soften);
    const math::Transform toModel = math::inverse(root_);
    const math::Vec3 now = current_.locals[static_cast<size_t>(joints_.pelvis)].position;
    const motion::LegSupport left{modelPos(models_, joints_.legL.start) - now, toModel.transformPoint(targets_.footL)};
    const motion::LegSupport right{modelPos(models_, joints_.legR.start) - now, toModel.transformPoint(targets_.footR)};
    return motion::clampToLegs(pelvis, left, right, rig_.legLength(), legs);
}

math::Vec3 Fielder::modelCentreOfMass() {
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

math::Vec3 Fielder::centreOfMass() {
    rig_.localToModel(current_, models_);
    return root_.transformPoint(modelCentreOfMass());
}

void Fielder::reachArm(bool left, math::Vec3 goal) {
    const float armReach = reach_.armReach * armLength_;
    const ik::TwoBoneChain& arm = left ? joints_.armL : joints_.armR;
    const int clavicle = left ? joints_.clavicleL : joints_.clavicleR;
    rig_.localToModel(current_, models_);

    // Clavicle: protract/elevate so the shoulder moves toward the goal.
    if (clavicle >= 0) {
        const motion::Swing swing =
            motion::swingToward(modelPos(models_, clavicle), modelPos(models_, arm.start), goal, armReach);
        rotateInModel(clavicle, math::fromAxisAngle(swing.axis, swing.angle));
    }

    // Arm: from the rest pose (no walk swing), two-bone IK toward the goal,
    // never further than armReach so the elbow stays bent. The elbow points
    // out, down and back (reach.elbowPole, mirrored for the right arm; +X is
    // left), made perpendicular to the shoulder-to-goal line so it bends away
    // from the body whatever the goal's direction.
    const rig::Pose& rest = rig_.restPose();
    current_.locals[static_cast<size_t>(arm.start)].rotation = rest.locals[static_cast<size_t>(arm.start)].rotation;
    current_.locals[static_cast<size_t>(arm.mid)].rotation = rest.locals[static_cast<size_t>(arm.mid)].rotation;
    rig_.localToModel(current_, models_);
    const math::Vec3 shoulder = modelPos(models_, arm.start);
    const math::Vec3 dir = math::safeNormalize(goal - shoulder, -math::kUp);
    const math::Vec3 side(left ? reach_.elbowPole.x : -reach_.elbowPole.x, reach_.elbowPole.y, reach_.elbowPole.z);
    const math::Vec3 pole = math::safeNormalize(side - dir * math::dot(side, dir), side);
    ik::solveTwoBone(arm, models_, motion::clampToReach(shoulder, goal, armReach), pole, ik_.soften, current_);

    // Palm toward the ball while reaching: turn the hand so its palm normal
    // points from the palm centre at the ball (within the wrist's limits).
    if (phase_ != Phase::Reach) return;
    rig_.localToModel(current_, models_);
    const math::Mat4& hand = models_[static_cast<size_t>(arm.end)];
    const math::Vec3 palm = math::Vec3(hand * math::Vec4(0.0f, -reach_.palmCentre, 0.0f, 1.0f));
    const math::Vec3 normal = modelRot(models_, arm.end) * math::Vec3(left ? -1.0f : 1.0f, 0.0f, 0.0f);
    const math::Vec3 ball = math::inverse(root_).transformPoint(reachTarget_);
    rotateInModel(arm.end, math::fromTo(normal, math::safeNormalize(ball - palm, normal)));
}

void Fielder::grip(bool left, float amount) {
    // Each finger and thumb bone turns toward the palm about its curl axis.
    const int side = left ? 0 : 1;
    const float curl[3] = {reach_.fingerCurl.x, reach_.fingerCurl.y, reach_.fingerCurl.z};
    const float thumb[3] = {reach_.thumbCurl.x, reach_.thumbCurl.y, reach_.thumbCurl.z};
    for (int f = 0; f < 5; ++f) {
        for (int k = 0; k < 3; ++k) {
            const int j = joints_.fingers[side][f][k];
            if (j < 0) continue;
            current_.locals[static_cast<size_t>(j)].rotation =
                math::fromAxisAngle(joints_.curlAxis[side][f][k], (f < 4 ? curl[k] : thumb[k]) * amount);
        }
    }
}

void Fielder::startPick() {
    if (holding_) return;  // already has it
    phase_ = Phase::Reach;
    holdTimer_ = stuckTimer_ = 0.0f;
    lunged_ = false;
}

void Fielder::clearReach() {
    if (phase_ == Phase::Reach) {
        carryFrom_ = goal_;
        phase_ = Phase::Return;
    }
}

void Fielder::updatePick(float dt) {
    // Progress eases in and out (smoothstep), so the hand leads and the body,
    // driven by the same goal through every stage, follows smoothly.
    if (phase_ == Phase::Idle) return;
    // Step first: a ball beyond the planted reach (horizontally, from the
    // middle of the feet) gets the lunge before the reach starts; the reach
    // waits until the foot lands, then the hand and body go from the new stance.
    const math::Vec3 ballNow = math::inverse(root_).transformPoint(reachTarget_);
    if (phase_ == Phase::Reach && pickTime_ <= 0.0f && !lunged_ && reach_.maxLunge > 0.0f) {
        const math::Vec3 mid = 0.5f * (stepper_.foot(motion::Foot::Left) + stepper_.foot(motion::Foot::Right));
        if (math::length(math::Vec2(ballNow.x - mid.x, ballNow.z - mid.z)) > reach_.stepBeyond) lungeToward(ballNow);
    }
    if (phase_ == Phase::Reach && stepper_.stepping()) return;  // goals from the last tick hold
    if (phase_ == Phase::Reach) pickTime_ = std::min(1.0f, pickTime_ + dt / std::max(reach_.reachTime, 0.05f));
    else pickTime_ = std::max(0.0f, pickTime_ - dt / std::max(reach_.returnTime, 0.05f));
    if (phase_ == Phase::Return && pickTime_ <= 0.0f) {
        phase_ = Phase::Idle;
        return;
    }
    const float s = pickTime_ * pickTime_ * (3.0f - 2.0f * pickTime_);

    // Which hand: the one on the ball's side (rest shoulders, stable while
    // solving), or the chosen one; both hands pick from either side of the ball.
    const math::Vec3 ball = math::inverse(root_).transformPoint(reachTarget_);
    if (phase_ == Phase::Reach) {
        reachLeft_ = handMode_ == HandMode::Left ||
                     (handMode_ != HandMode::Right && math::distance(restShoulderL_, ball) <= math::distance(restShoulderR_, ball));
        bothHands_ = handMode_ == HandMode::Both;
    }
    // Wrist goal on the ball: back from it toward the shoulder (palm on the
    // ball); with both hands, each a ball radius to its own side (+X is left).
    auto onBall = [&](bool left) {
        const math::Vec3 shoulder = left ? restShoulderL_ : restShoulderR_;
        const math::Vec3 side = bothHands_ ? math::kRight * (left ? reach_.ballRadius : -reach_.ballRadius) : math::Vec3(0.0f);
        return ball + side + math::safeNormalize(shoulder - ball, math::kUp) * reach_.wristOffset;
    };
    auto rest = [&](bool left) { return left ? restWristL_ : restWristR_; };
    const math::Vec3 end = phase_ == Phase::Reach ? onBall(reachLeft_) : carryFrom_;
    goal_ = math::lerp(rest(reachLeft_), end, s);
    const math::Vec3 otherEnd = phase_ == Phase::Reach ? onBall(!reachLeft_) : carryFrom_ - onBall(reachLeft_) + onBall(!reachLeft_);
    otherGoal_ = math::lerp(rest(!reachLeft_), otherEnd, s);
}

void Fielder::finishPick(float dt) {
    if (phase_ != Phase::Reach || pickTime_ < 1.0f) return;
    if (reachError_ <= reach_.gripTolerance) {
        // On the ball: grip, then carry it back.
        holdTimer_ += dt;
        if (holdTimer_ >= reach_.holdTime) {
            holding_ = true;
            carryFrom_ = goal_;
            phase_ = Phase::Return;
        }
        return;
    }
    if (stepper_.stepping()) return;
    if (!lunged_ && reach_.maxLunge > 0.0f) {
        // Fallback: the step-first estimate (stepBeyond) was short.
        lungeToward(math::inverse(root_).transformPoint(reachTarget_));
        return;
    }
    // Still out of reach after the lunge: hold the stretch briefly, then let
    // go (this is where the dive will start).
    stuckTimer_ += dt;
    if (stuckTimer_ >= reach_.giveUpTime) {
        LOG_INFO("reach: ball out of reach by {:.2f} m (dive case)", reachError_);
        clearReach();
    }
}

void Fielder::lungeToward(math::Vec3 ball) {
    // The foot nearer the ball steps to land beside it (BIOMECHANICS 8: low
    // pickups put the foot by the ball), lungeShort short of it, at most maxLunge.
    lunged_ = true;
    const motion::Foot foot = math::distance(stepper_.foot(motion::Foot::Left), ball) <=
                                      math::distance(stepper_.foot(motion::Foot::Right), ball)
                                  ? motion::Foot::Left
                                  : motion::Foot::Right;
    const math::Vec3 from = stepper_.foot(foot);
    const math::Vec3 flat(ball.x - from.x, 0.0f, ball.z - from.z);
    const float length = math::clamp(math::length(flat) - reach_.lungeShort, 0.0f, reach_.maxLunge);
    stepper_.stepTo(foot, from + math::safeNormalize(flat, math::kForward) * length);
}

math::Vec3 Fielder::ballSpot(bool left) const {
    // Half the palm down the hand (-Y), out along the palm normal.
    return math::Vec3(0.0f, -reach_.palmCentre, 0.0f) +
           math::Vec3(left ? -1.0f : 1.0f, 0.0f, 0.0f) * (reach_.ballRadius + reach_.palmThickness);
}

math::Vec3 Fielder::ballInHand(const std::vector<math::Mat4>& models, const math::Transform& root) const {
    // In front of the holding palm; with both hands, between the two palm centres.
    auto at = [&](bool left, math::Vec3 local) {
        const ik::TwoBoneChain& arm = left ? joints_.armL : joints_.armR;
        return math::Vec3(models[static_cast<size_t>(arm.end)] * math::Vec4(local, 1.0f));
    };
    const math::Vec3 centre(0.0f, -reach_.palmCentre, 0.0f);
    const math::Vec3 p = bothHands_ ? 0.5f * (at(true, centre) + at(false, centre)) : at(reachLeft_, ballSpot(reachLeft_));
    return root.transformPoint(p);
}

void Fielder::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
