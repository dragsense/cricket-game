#include "agents/human.h"

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

bool Human::init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion) {
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
    reposition_.maxReach = joints.getFloat("reposition.maxReach", reposition_.maxReach);
    reposition_.followRate = joints.getFloat("reposition.followRate", reposition_.followRate);
    reposition_.softBand = joints.getFloat("reposition.softBand", reposition_.softBand);
    reposition_.damping = joints.getFloat("reposition.damping", reposition_.damping);
    reposition_.stanceDrop = math::radians(joints.getFloat("reposition.stanceDropDeg", math::degrees(reposition_.stanceDrop)));
    stanceBlendHeight_ = joints.getFloat("reposition.stanceBlendHeight", stanceBlendHeight_);
    bodyRepositionSettings_.passes = joints.getInt("bodyReposition.passes", bodyRepositionSettings_.passes);
    armLegRatio_ = joints.getFloat("bodyReposition.armLegRatio", armLegRatio_);
    bodyRepositionSettings_.legReach = joints.getFloat("bodyReposition.legReach", bodyRepositionSettings_.legReach);
    {
        motion::BodyRepositionSettings& b = bodyRepositionSettings_;
        b.dorsiflexion = math::radians(joints.getFloat("bodyReposition.dorsiflexionDeg", math::degrees(b.dorsiflexion)));
        b.plantarflexion = math::radians(joints.getFloat("bodyReposition.plantarflexionDeg", math::degrees(b.plantarflexion)));
        b.pushOff = math::radians(joints.getFloat("bodyReposition.pushOffDeg", math::degrees(b.pushOff)));
        b.rollPasses = joints.getInt("bodyReposition.rollPasses", b.rollPasses);
    }
    // The fielder starts with the ball in the right hand, picking with it.
    phase_ = Phase::Idle;
    holding_ = true;
    reachLeft_ = false;
    handMode_ = HandMode::Auto;
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
    reach_.fittsA = joints.getFloat("reach.fittsA", reach_.fittsA);
    reach_.fittsB = joints.getFloat("reach.fittsB", reach_.fittsB);
    reach_.fittsWidth = joints.getFloat("reach.fittsWidth", reach_.fittsWidth);
    reach_.gripTolerance = joints.getFloat("reach.gripTolerance", reach_.gripTolerance);
    reach_.ballRadius = joints.getFloat("reach.ballRadius", reach_.ballRadius);
    reach_.relaxedCurl = joints.getFloat("reach.relaxedCurl", reach_.relaxedCurl);
    reach_.palmCentre = joints.getFloat("reach.palmCentre", reach_.palmCentre);
    reach_.palmThickness = joints.getFloat("reach.palmThickness", reach_.palmThickness);
    reach_.fingerCurl = degs("reach.fingerCurlDeg", reach_.fingerCurl);
    reach_.thumbCurl = degs("reach.thumbCurlDeg", reach_.thumbCurl);
    reach_.maxLunge = joints.getFloat("reach.maxLunge", reach_.maxLunge);
    reach_.lungeShort = joints.getFloat("reach.lungeShort", reach_.lungeShort);
    reach_.lungeDuration = joints.getFloat("reach.lungeDuration", reach_.lungeDuration);
    reach_.lungeHeight = joints.getFloat("reach.lungeHeight", reach_.lungeHeight);
    reach_.elbowPole = joints.getVec3("reach.elbowPole", reach_.elbowPole);
    return setHeight(height);
}

void Human::loadLocomotion(const core::Config& c) {
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
    load("walk", walkSettings_);
    jogSettings_ = walkSettings_;
    load("jog", jogSettings_);
    runSettings_ = jogSettings_;  // the jog block is the run's base (no jog pace of its own)
    load("run", runSettings_);
    postureLean_ = math::radians(c.getFloat("walk.postureLeanDeg", math::degrees(postureLean_)));
    stepSettings_.stepLength = c.getFloat("step.stepLength", stepSettings_.stepLength);
    stepSettings_.backStepLength = c.getFloat("step.backStepLength", stepSettings_.backStepLength);
    stepSettings_.sideStepLength = c.getFloat("step.sideStepLength", stepSettings_.sideStepLength);
    stepSettings_.stepDuration = c.getFloat("step.stepDuration", stepSettings_.stepDuration);
    stepSettings_.stepHeight = c.getFloat("step.stepHeight", stepSettings_.stepHeight);
    stepSettings_.peakAt = c.getFloat("step.swingPeakAt", stepSettings_.peakAt);
    stepSettings_.liftAt = c.getFloat("step.liftAt", stepSettings_.liftAt);
    auto get = [&](const char* key, float v) { return c.getFloat(std::string("jump.") + key, v); };
    auto deg = [&](const char* key, float radians) { return math::radians(get(key, math::degrees(radians))); };
    jump_.crouchDepth = get("crouchDepth", jump_.crouchDepth);
    jump_.loadTime = get("loadTime", jump_.loadTime);
    jump_.minLoad = get("minLoad", jump_.minLoad);
    jump_.moveLoad = get("moveLoad", jump_.moveLoad);
    jump_.crouchLean = deg("crouchLeanDeg", jump_.crouchLean);
    jump_.pushTime = get("pushTime", jump_.pushTime);
    jump_.movePushTime = get("movePushTime", jump_.movePushTime);
    jump_.moveReach = get("moveReach", jump_.moveReach);
    jump_.moveHeight = get("moveHeight", jump_.moveHeight);
    jump_.moveKnee = get("moveKnee", jump_.moveKnee);
    jump_.moveTrail = get("moveTrail", jump_.moveTrail);
    jump_.pushLift = get("pushLift", jump_.pushLift);
    jump_.height = get("height", jump_.height);
    jump_.forwardDistance = get("forwardDistance", jump_.forwardDistance);
    jump_.backDistance = get("backDistance", jump_.backDistance);
    jump_.sideDistance = get("sideDistance", jump_.sideDistance);
    jump_.tuck = get("tuck", jump_.tuck);
    jump_.lean = deg("leanDeg", jump_.lean);
    jump_.landDepth = get("landDepth", jump_.landDepth);
    jump_.landTime = get("landTime", jump_.landTime);
    jump_.recoverTime = get("recoverTime", jump_.recoverTime);
    jump_.armBack = deg("armBackDeg", jump_.armBack);
    jump_.armBackForward = deg("armBackForwardDeg", jump_.armBackForward);
    jump_.armUp = deg("armUpDeg", jump_.armUp);
    jump_.armUpForward = deg("armUpForwardDeg", jump_.armUpForward);
    jump_.armUpBack = deg("armUpBackDeg", jump_.armUpBack);
    jump_.armUpSide = deg("armUpSideDeg", jump_.armUpSide);
    jump_.sideOut = deg("sideOutDeg", jump_.sideOut);
    jump_.acrossFlex = deg("acrossFlexDeg", jump_.acrossFlex);
    jump_.across = deg("acrossDeg", jump_.across);
    jump_.flightFall = get("flightFall", jump_.flightFall);
    jump_.landFlex = deg("landFlexDeg", jump_.landFlex);
    jump_.landSideOut = deg("landSideOutDeg", jump_.landSideOut);
    jump_.landSideFlex = deg("landSideFlexDeg", jump_.landSideFlex);
    jump_.elbowLoad = deg("elbowLoadDeg", jump_.elbowLoad);
    jump_.elbowTop = deg("elbowTopDeg", jump_.elbowTop);
    jump_.elbowLand = deg("elbowLandDeg", jump_.elbowLand);
    jump_.moveDrive = deg("moveDriveDeg", jump_.moveDrive);
    jump_.moveBack = deg("moveBackDeg", jump_.moveBack);
    jump_.moveMin = get("moveMin", jump_.moveMin);
    jump_.elbowMove = deg("elbowMoveDeg", jump_.elbowMove);
    control_ = shape_ = walkSettings_;
}

void Human::blendShape() {
    // Walk below walking speed, then walk -> jog -> run by the actual speed,
    // measured against each pace's speed in the direction of travel.
    const motion::WalkSettings* gaits[2] = {&walkSettings_, &runSettings_};
    const math::Vec2 local = walker_.localVelocity();
    const bool forward = local.y > std::fabs(local.x);
    auto ref = [&](const motion::WalkSettings& g) {
        return forward ? g.speed : std::fabs(local.x) > std::fabs(local.y) ? g.sideSpeed : g.backSpeed;
    };
    const float v = walker_.speed();
    shape_ = runSettings_;
    for (int i = 0; i < 2; ++i) {
        if (v > ref(*gaits[i])) continue;
        if (i == 0) shape_ = walkSettings_;
        else shape_ = motion::blendGait(*gaits[i - 1], *gaits[i],
                                        (v - ref(*gaits[i - 1])) / std::max(ref(*gaits[i]) - ref(*gaits[i - 1]), 0.1f));
        break;
    }
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
    body_ = motion::BodyState{};
    body_.position = current_.locals[static_cast<size_t>(joints_.pelvis)].position;  // pelvis is the root joint
    rig_.localToModel(current_, models_);
    restShoulderL_ = modelPos(models_, joints_.armL.start);
    restShoulderR_ = modelPos(models_, joints_.armR.start);
    restChest_ = modelPos(models_, joints_.spine[2] >= 0 ? joints_.spine[2] : joints_.pelvis);
    restWristL_ = modelPos(models_, joints_.armL.end);
    restWristR_ = modelPos(models_, joints_.armR.end);
    liveShoulderL_ = restShoulderL_;
    liveShoulderR_ = restShoulderR_;
    if (joints_.spine[2] >= 0) {
        // Shoulders in the chest's frame, so the bowling circles can follow the trunk.
        const math::Mat4 toChest = math::inverse(models_[static_cast<size_t>(joints_.spine[2])]);
        chestShoulderL_ = math::Vec3(toChest * math::Vec4(restShoulderL_, 1.0f));
        chestShoulderR_ = math::Vec3(toChest * math::Vec4(restShoulderR_, 1.0f));
    }
    // Deepest crouch: the leg with the knee bent to its joint limit.
    {
        const float thigh = math::distance(modelPos(models_, joints_.legL.start), modelPos(models_, joints_.legL.mid));
        const float shin = math::distance(modelPos(models_, joints_.legL.mid), modelPos(models_, joints_.legL.end));
        reposition_.minReach = motion::bentReach(thigh, shin, limits_.limit(joints_.legL.mid).twistMax) / rig_.legLength();
    }
    armLength_ = math::distance(modelPos(models_, joints_.armL.start), modelPos(models_, joints_.armL.mid)) +
                 math::distance(modelPos(models_, joints_.armL.mid), modelPos(models_, joints_.armL.end));
    if (joints_.toeL >= 0) {
        rig_.localToModel(current_, models_);
        const math::Vec3 d = modelPos(models_, joints_.toeL) - modelPos(models_, joints_.legL.end);
        ballDistance_ = math::length(math::Vec3(d.x, 0.0f, d.z));
        ballDrop_ = -d.y;
    }
    {
        motion::BodyRepositionJoints sj;
        sj.pelvis = joints_.pelvis;
        sj.legs[0] = joints_.legL;
        sj.legs[1] = joints_.legR;
        sj.toes[0] = joints_.toeL;
        sj.toes[1] = joints_.toeR;
        // Sole contacts from the skeleton's shoe (foot box, toe joint), in
        // metres: heel = the box's back edge, ball = under the toe joint.
        for (int i = 0; i < 2; ++i) {
            const int foot = def_.find(i == 0 ? "foot_l" : "foot_r"), toe = def_.find(i == 0 ? "toe_l" : "toe_r");
            if (foot < 0) continue;
            const rig::BoneVisual& v = def_.bones[static_cast<size_t>(foot)].visual;
            const float h = rig_.height(), sole = (v.center.y - 0.5f * v.size.y) * h;
            sj.heel[i] = math::Vec3(0.0f, sole, (v.center.z - 0.5f * v.size.z) * h);
            const float ahead = toe >= 0 ? def_.bones[static_cast<size_t>(toe)].offset.z * h : 0.0f;
            sj.ball[i] = math::Vec3(0.0f, sole, ahead);
        }
        for (int i = 0; i < 3; ++i) sj.spine[i] = joints_.spine[i];
        sj.clavicles[0] = joints_.clavicleL;
        sj.clavicles[1] = joints_.clavicleR;
        sj.arms[0] = joints_.armL;
        sj.arms[1] = joints_.armR;
        bodyReposition_.init(rig_, limits_, sj);
    }
    plantFeet();
    return true;
}

void Human::placeAt(math::Vec3 position, float yaw) {
    root_.position = position;
    root_.rotation = math::fromAxisAngle(math::kUp, yaw);
    previousRoot_ = root_;
    walker_.reset(yaw);
    jumper_ = motion::JumpController{};
    jumpQueued_ = followThrough_ = landTurn_ = autoDeliver_ = checkHeld_ = bounding_ = false;
    carrySpeed_ = frontLift_ = 0.0f;
    plantFeet();
}

void Human::autoDeliver(float releaseAngle, float releaseTilt) {
    if (phase_ != Phase::Bowl) return;
    autoDeliver_ = true;
    autoReleaseAngle_ = releaseAngle;
    autoReleaseTilt_ = releaseTilt;
    // The bound: the running goes on as it is; the back foot's step under
    // way is stretched (longer and higher), and the front foot's next one
    // (the delivery stride) too. The hand reaches the action point, then
    // goes down to the release circle (runBackTime) as the back foot lands.
    stepper_.stretchSwing(bowl_.boundStretch);
    stepper_.stretchNext(bowl_.strideStretch);
    bounding_ = true;
    carrySpeed_ = walker_.speed();
    autoCoilTime_ = std::max(stepper_.swingTimeLeft() - bowl_.runBackTime, 0.05f);
    // The body stays facing the batter; it turns once the hand starts down
    // from the action point (Back stage).
    bowlYaw_ = bowlBaseYaw_;
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
    if (checkHeld_) return;  // frozen at a check stop
    current_.locals = rig_.restPose().locals;
    // Jump (not picking): standing, held J crouches; moving, it goes at once.
    const bool moving = walker_.speed() >= 0.2f;
    if (jumpHeld_ && moving && !jumper_.active()) jumpQueued_ = true;  // a tap is kept until the foot is ready
    if (!moving || phase_ != Phase::Idle) jumpQueued_ = false;
    if ((jumpHeld_ || jumpQueued_) && phase_ == Phase::Idle && !jumper_.active()) {
        jumpMoving_ = moving;
        // Moving, the swinging leg is the knee that drives up.
        if (jumpMoving_ || !stepper_.stepping())
            jumper_.start(jumpMoving_, stepper_.swingFoot() == static_cast<int>(motion::Foot::Left),
                          walker_.speed() / std::max(runSettings_.speed, 0.1f));
    }
    // Moving: the dip holds until the take-off foot (if set) has just planted
    // (the other leg early in its swing); the push then goes off it (walk()
    // freezes the feet from the push on).
    const bool footReady = jumpFoot_ < 0 || (stepper_.stepping() && stepper_.swingFoot() != jumpFoot_ &&
                                             stepper_.swingProgress() < 0.35f);
    if (jumpMoving_ && jumper_.loading() && (footReady || !moving)) {
        jumper_.release(moveInput_, jump_.moveLoad);
        jumpQueued_ = false;
    }
    if (!jumpHeld_ && !(jumpMoving_ && jumper_.loading())) jumper_.release(moveInput_);
    const bool wasAirborne = jumper_.airborne();
    jumper_.tick(static_cast<float>(dt), jump_);
    if (wasAirborne && !jumper_.airborne() && phase_ == Phase::Bowl) {
        // Landing into the bowling action on the back foot (the bowling
        // arm's side); the front foot stays up until the hand leaves the
        // action point, then strides (downBowl). The body keeps moving over
        // the planted back foot (slowing); the run's speed is handed back at
        // the release.
        carrySpeed_ = walker_.speed();
        math::Vec3 left = stepper_.foot(motion::Foot::Left), right = stepper_.foot(motion::Foot::Right);
        math::Vec3& front = reachLeft_ ? right : left;
        const math::Vec3& back = reachLeft_ ? left : right;
        frontLift_ = std::max(front.y - back.y, jump_.tuck);
        front.y = back.y;
        // The bound is just a running step: the landing foot planted last, the
        // order goes on (the stride, then the back leg, then left, right ...).
        stepper_.place(motion::Foot::Left, left);
        stepper_.place(motion::Foot::Right, right);
        stepper_.landed(reachLeft_ ? motion::Foot::Left : motion::Foot::Right);
        // Back-foot contact: the front leg still in the air; the body turns
        // with the hand on its way down (Back stage).
        landTurn_ = true;
        followThrough_ = false;
        checkHeld_ = checkStops_ && autoDeliver_;
        frontStopped_ = false;
    }
    // Single steps: standing still (not walking, picking or jumping), a step
    // input takes steps until the fielder walks or acts.
    const bool standing = (phase_ == Phase::Idle || (phase_ == Phase::Pose && !poseHand_)) && walker_.speed() < 0.2f &&
                          math::length2(moveInput_) < 1e-6f && !jumper_.active();
    if (!standing) stepWalk_ = false;
    else if (math::length2(stepInput_) > 1e-6f) stepWalk_ = true;
    walk(static_cast<float>(dt));
    if (bounding_ && stepper_.swingFoot() != static_cast<int>(reachLeft_ ? motion::Foot::Left : motion::Foot::Right)) {
        // Back-foot contact (the long step down): the running goes on.
        bounding_ = false;
        landTurn_ = true;
        followThrough_ = false;
        checkHeld_ = checkStops_ && autoDeliver_;
    }
    if (jumpMoving_ && jumpFoot_ >= 0 &&
        (jumper_.stage() == motion::JumpStage::Push || jumper_.airborne())) {
        // Bound off the take-off foot: the other (swing) leg drives on ahead
        // through the push and the flight and comes down there.
        const motion::Foot drive = jumpFoot_ == static_cast<int>(motion::Foot::Left) ? motion::Foot::Right : motion::Foot::Left;
        math::Vec3 p = stepper_.foot(drive);
        // Feet are world-fixed in the bound (walk()): the swing foot travels
        // to its landing spot, arriving as the flight ends.
        // Its own path: the knee drives up at the push (the foot lifted high),
        // then the foot reaches forward and comes down to land.
        const float knee = jump_.moveKnee * rig_.legLength();
        const float left = jumper_.airborne() ? jumper_.timeToLand() : 0.07f;
        const float follow = std::min(1.0f, static_cast<float>(dt) / std::max(left, static_cast<float>(dt)));
        p.z += (jump_.moveReach * rig_.legLength() - p.z) * follow;
        if (jumper_.airborne()) {
            const float s = 1.0f - jumper_.timeToLand() / std::max(jumper_.flightTime(), 0.01f);
            p.y = knee * std::cos(0.5f * math::kPi * s);
        } else {
            p.y += (knee - p.y) * std::min(1.0f, static_cast<float>(dt) * 15.0f);
        }
        stepper_.place(drive, p);
        if (jumper_.airborne()) {
            // The take-off foot stays where it left the ground and the body
            // moves away from it, so the leg stretches out straight behind
            // (the foot rising only as the leg's length needs; beyond the
            // trail it is drawn along). After the landing it swings through
            // past the landing leg into the stride.
            const motion::Foot push = static_cast<motion::Foot>(jumpFoot_);
            math::Vec3 q = stepper_.foot(push);
            const float leg = rig_.legLength() * 0.98f;
            q.z = std::max(q.z, -jump_.moveTrail * rig_.legLength());
            q.y = rig_.legLength() + jumper_.bodyY() - std::sqrt(std::max(leg * leg - q.z * q.z, 0.0f));  // hip rises with the body
            stepper_.place(push, q);
        }
    }
    targets_.footL = root_.transformPoint(stepper_.foot(motion::Foot::Left));
    targets_.footR = root_.transformPoint(stepper_.foot(motion::Foot::Right));
    // Feet stand on the world's real surface under them (Jolt ground query:
    // the field, later a step or a slope), not on an assumed flat ground.
    if (physics_) {
        const float base = physics_->groundHeightAt(root_.position);
        targets_.footL.y += physics_->groundHeightAt(targets_.footL) - base;
        targets_.footR.y += physics_->groundHeightAt(targets_.footR) - base;
    }
    // After a run-up landing the front foot is held up, coming down over its
    // stride (before the body settles, so the body stands on the landed foot).
    if (phase_ != Phase::Bowl || (strideFoot_ >= 0 && !stepper_.stepping())) frontLift_ = 0.0f;
    if (frontLift_ > 0.0f) {
        const motion::Foot front = reachLeft_ ? motion::Foot::Right : motion::Foot::Left;
        const bool striding = strideFoot_ == static_cast<int>(front) && stepper_.stepping();
        (front == motion::Foot::Left ? targets_.footL : targets_.footR).y +=
            frontLift_ * (striding ? 1.0f - stepper_.swingProgress() : 1.0f);
    }
    // Limb test with a placed foot: one solve from the body's real constraints
    // (motion::BodyReposition) does the body and legs instead.
    // Single steps and walking too: the swinging foot reaches, the other
    // carries the body. (Jogging, running, jumping, picking and bowling stay
    // on the backup repositioning for now.)
    const bool walking = phase_ == Phase::Idle && !jumper_.active() && !followThrough_;
    if (walking && stepper_.stepping()) stepPlanted_ = 1 - stepper_.swingFoot();
    // The limb test's hand target too (motion::BodyReposition; picking,
    // throwing and bowling stay on the reach* functions for now).
    const bool posedHand = phase_ == Phase::Pose && poseHand_;
    const bool stance = stepWalk_ || walking || posedHand || (phase_ == Phase::Pose && posedFoot_ >= 0);
    if (!stance) repositionBody(static_cast<float>(dt));
    // In the air the feet leave the ground (after the body settles over them).
    // (Not in a bound: there each leg has its own path, set above.)
    if (!(jumpMoving_ && jumpFoot_ >= 0)) {
        targets_.footL.y += jumper_.feetY();
        targets_.footR.y += jumper_.feetY();
    }
    addGaitMotion(static_cast<float>(dt));
    reachError_ = 0.0f;
    updatePick(static_cast<float>(dt));
    updateThrow(static_cast<float>(dt));
    updateBowl(static_cast<float>(dt));
    if (phase_ == Phase::Pose && poseHand_) goal_ = math::inverse(root_).transformPoint(poseGoal_);
    // While the step-first lunge is in the air the body follows the feet the
    // usual way (repositionBody); the reach starts once the foot lands.
    const bool reachNow = phase_ == Phase::Return || phase_ == Phase::Throw || phase_ == Phase::Bowl ||
                          (phase_ == Phase::Reach && pickTime_ > 0.0f);
    if (reachNow) {
        twistChest();
        if (phase_ != Phase::Bowl && phase_ != Phase::Throw) {  // throw, bowl: no reverse IK, the arm IK alone takes the hand
            reachSpine();
            reachPelvis();
            reachBody();
        }
    }
    if (!stance) solveIk();
    if (reachNow && !stepper_.stepping()) rebalance();  // a swinging foot is no support
    swingArms(static_cast<float>(dt));
    if (stance) repositionFromFeet();  // after the arm swing: a hand target's arm wins
    stabiliseHead(static_cast<float>(dt));
    if (reachNow) {
        reachArm(reachLeft_, goal_);
        if (phase_ == Phase::Bowl && bowlStage_ != BowlStage::Circle) reachArm(!reachLeft_, frontGoal_);
    }
    // Fingers: closed around the ball while it is held (walking and running
    // too), relaxed otherwise.
    const bool closed = holding_;
    grip(true, closed && reachLeft_ ? 1.0f : reach_.relaxedCurl);
    grip(false, closed && !reachLeft_ ? 1.0f : reach_.relaxedCurl);
    limits_.apply(current_);
    if (reachNow) {
        // Measured after the limits, so a clamped arm shows as a miss.
        rig_.localToModel(current_, models_);
        const int hand = reachLeft_ ? joints_.armL.end : joints_.armR.end;
        reachError_ = math::distance(modelPos(models_, hand), goal_);
        const math::Vec3 ball =
            root_.transformPoint(math::Vec3(models_[static_cast<size_t>(hand)] * math::Vec4(ballSpot(reachLeft_), 1.0f)));
        if (released_) releasePosition_ = ball;
        finishPick(static_cast<float>(dt));
    }
    // The body's mass: the feet on the ground hold it for the next physics
    // step (the world steps after the fielders).
    if (physics_ && humanBody_.ready()) {
        const math::Vec3 ankles[2] = {targets_.footL, targets_.footR};
        const bool down[2] = {!stepper_.inAir(motion::Foot::Left), !stepper_.inAir(motion::Foot::Right)};
        humanBody_.setFeet(*physics_, root_.position + math::kUp * restComHeight_, ankles, down,
                             static_cast<float>(dt));
    }
    // Shoulders carried by the chest (not the clavicle) while bowling, for
    // the bowling circles next tick; at rest otherwise.
    if (reachNow && phase_ == Phase::Bowl && joints_.spine[2] >= 0) {
        const math::Mat4& chest = models_[static_cast<size_t>(joints_.spine[2])];
        liveShoulderL_ = math::Vec3(chest * math::Vec4(chestShoulderL_, 1.0f));
        liveShoulderR_ = math::Vec3(chest * math::Vec4(chestShoulderR_, 1.0f));
    } else {
        liveShoulderL_ = restShoulderL_;
        liveShoulderR_ = restShoulderR_;
    }
}

void Human::walk(float dt) {
    // Move and turn the root; planted feet stay where they are in the world.
    control_ = gait_ == motion::Gait::Run ? runSettings_ : gait_ == motion::Gait::Jog ? jogSettings_ : walkSettings_;
    // A run-up delivery keeps running whatever the gait keys say.
    if (autoDeliver_) control_ = runSettings_;
    if (jumper_.active() &&
        (!jumpMoving_ || jumper_.airborne() || jumper_.stage() == motion::JumpStage::Push)) {
        // Jumping: the root carries the flight's travel and the feet go with
        // it (character space), landing where the body comes down. A moving
        // jump keeps the walker's speed (not ticked) and its gait (frozen
        // mid-stride); before take-off and after landing it walks as usual.
        if (!jumpMoving_) walker_.tick(dt, math::Vec2(0.0f), false, 0.0f, control_);
        if (jumpMoving_ && autoDeliver_) {
            // The bound of a run-up delivery: the body turns side-on in the
            // air (run turn rate), the flight keeping its world velocity.
            const math::Vec3 world = walker_.velocity();
            walker_.tick(dt, math::Vec2(0.0f), true, bowlYaw_, runSettings_);
            const math::Vec3 local = math::inverse(math::fromAxisAngle(math::kUp, walker_.yaw())) * world;
            walker_.setLocalVelocity(math::Vec2(local.x, local.z));
            root_.rotation = math::fromAxisAngle(math::kUp, walker_.yaw());
        }
        blendShape();
        const math::Transform from = root_;
        const math::Vec2 v = jumper_.velocity();
        root_.position += root_.rotation * math::Vec3(v.x, 0.0f, v.y) * dt;
        if (jumpMoving_) root_.position += walker_.velocity() * dt;
        // Pushing off, the take-off foot stays planted (the body moves over it).
        // In a bound the feet stay where they are in the world too (tick()
        // swings them), so they do not ride along with the body.
        if (jumper_.stage() == motion::JumpStage::Push || (jumpMoving_ && jumpFoot_ >= 0)) stepper_.rebase(from, root_);
        return;
    }
    // While picking the fielder stands (walk input waits for the return).
    // Bowling turns the whole player to bowlYaw_ (side-on, then straight).
    if (phase_ == Phase::Bowl && !holding_ && carrySpeed_ > 0.0f) {
        // Released after a run-up: the bowler runs on, straight (the facing
        // before the action), picking up from the braced front leg at a
        // share of the run-up's pace, then slowing (run-on below).
        followShare_ = bowl_.runOnPace * carrySpeed_ / std::max(runSettings_.speed, 0.1f);
        carrySpeed_ = 0.0f;
        followThrough_ = true;
        followTime_ = 0.0f;
        followSwing_ = -1;
        // From here on it is plain running (the back leg is already swinging
        // through, started at front-foot contact).
        jumper_ = motion::JumpController{};
        frontLift_ = 0.0f;
        // Running at once (not picking up from standing still): with no
        // speed the steps would settle the feet back under the body.
        const float a = bowlBaseYaw_ - walker_.yaw();
        walker_.setLocalVelocity(math::Vec2(std::sin(a), std::cos(a)) * (followShare_ * runSettings_.speed));
    }
    if (phase_ != Phase::Bowl) {
        carrySpeed_ = 0.0f;
        landTurn_ = autoDeliver_ = false;
    }
    // Run-on: the usual running gait straight on toward the batter, the pace
    // easing off over runOnTime (it outlasts the arm's follow-through).
    math::Vec2 runOn(0.0f);
    if (followThrough_) {
        followTime_ += dt;
        // Check stop: after every run-on step (as the next one begins).
        if (checkStops_ && stepper_.stepping()) {
            if (followSwing_ >= 0 && stepper_.swingFoot() != followSwing_) checkHeld_ = true;
            followSwing_ = stepper_.swingFoot();
        }
        // Plain running on for runOnHold of runOnTime, then the input stops
        // and the walker slows to a stop the usual way (steps get slower).
        const bool running = followTime_ < bowl_.runOnHold * bowl_.runOnTime;
        const float a = bowlBaseYaw_ - walker_.yaw();
        runOn = running ? math::Vec2(std::sin(a), std::cos(a)) * followShare_ : math::Vec2(0.0f);
        control_ = runSettings_;
        // Ends only once stopped and the arm is back (the legs never drop
        // into the standing reach steps while still moving).
        if (!running && walker_.speed() < 0.2f && phase_ == Phase::Idle) followThrough_ = false;
    }
    const bool picking = phase_ != Phase::Idle;
    const bool bowlTurn = phase_ == Phase::Bowl && bowlStage_ != BowlStage::Circle;
    // The run carries on through the bound and the stride to the release (the body keeps moving over the feet).
    const float ahead = bowlBaseYaw_ - walker_.yaw();
    const math::Vec2 bound = autoDeliver_ && holding_ ? math::Vec2(std::sin(ahead), std::cos(ahead)) : math::Vec2(0.0f);
    walker_.tick(dt, followThrough_ ? runOn : picking ? bound : moveInput_,
                 bowlTurn || (face_ && !picking), bowlTurn ? bowlYaw_ : faceYaw_, control_);
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
    // Run-up delivery: the feet keep running straight at the batter
    // (stance along the run line) whatever the body's turn.
    stepper_.setStanceTurn(autoDeliver_ || followThrough_ ? bowlBaseYaw_ - walker_.yaw() : 0.0f);
    // Single steps: the stepper picks the foot (the leading side's foot, else
    // alternating) and swings it to its spot; the body follows from the
    // body reposition. The root (and the camera with it) stays between the
    // feet every tick, so it glides along with the swinging foot.
    if (stepWalk_) {
        stepper_.tick(dt, stepInput_, stepSettings_);
        if (stepper_.stepping()) stepPlanted_ = 1 - stepper_.swingFoot();
        const math::Transform before = root_;
        const math::Vec3 mid = root_.transformPoint(0.5f * (stepper_.foot(motion::Foot::Left) + stepper_.foot(motion::Foot::Right)));
        root_.position.x = mid.x;
        root_.position.z = mid.z;
        stepper_.rebase(before, root_);
        return;
    }
    if (picking && !followThrough_ && !autoDeliver_ && !(bowlTurn && bowlStage_ == BowlStage::Back && strideWaiting_ && !autoDeliver_)) {  // turning side-on, the feet step round
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

motion::GaitStyle Human::gaitStyle() const {
    const motion::WalkSettings& w = shape_;
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

void Human::addGaitMotion(float dt) {
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
    const math::Vec3 offset(side * w.sway * amount * arc, amount * (w.bob * bob - w.pelvisDrop) + jumper_.bodyY(), 0.0f);

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
    pelvis.position = body_.position + offset;
    // Pelvis turns toward the forward leg (+X left: a positive turn about +Y
    // takes the left hip back); the chest turns back against it.
    const float twist = -w.pelvisTwist * amount * legPhase_;
    const math::Vec2 lean = lean_ + jumper_.lean();
    pelvis.rotation = math::fromAxisAngle(math::kRight, lean.x) * math::fromAxisAngle(math::kForward, lean.y) *
                      math::fromAxisAngle(math::kUp, twist) * pelvis.rotation;
    if (joints_.chest >= 0) {
        math::Quat& chest = current_.locals[static_cast<size_t>(joints_.chest)].rotation;
        chest = math::fromAxisAngle(math::kUp, -w.chestCounter * twist) * chest;
    }
}

void Human::repositionBody(float dt) {
    // Hip offsets from the rest pose; the pelvis is the root joint, so its
    // local position is in character space.
    rig_.localToModel(rig_.restPose(), models_);
    const math::Vec3 restPelvis = modelPos(models_, joints_.pelvis);
    const math::Transform toModel = math::inverse(root_);
    motion::LegSupport left{modelPos(models_, joints_.legL.start) - restPelvis, toModel.transformPoint(targets_.footL)};
    motion::LegSupport right{modelPos(models_, joints_.legR.start) - restPelvis, toModel.transformPoint(targets_.footR)};

    // The lower foot carries the body; even weight when both are level.
    const float stance = math::clamp(0.5f + 0.5f * (left.foot.y - right.foot.y) / stanceBlendHeight_, 0.0f, 1.0f);
    motion::RepositionSettings reposition = reposition_;
    reposition.sideFollow = shape_.sideFollow;  // runners barely shift sideways
    // Moving, the root already travels smoothly; chasing the stance foot
    // (which slides back every step) would make the body lag and surge.
    reposition.forwardFollow = 1.0f - math::clamp(walker_.speed() / std::max(shape_.speed, 0.1f), 0.0f, 1.0f);
    reposition.restZ = restPelvis.z;
    // Standing rules (straight-leg height, hip drop); moving, the gait's own
    // pelvis motion (addGaitMotion) takes over.
    reposition.standShare = reposition.forwardFollow;

    motion::repositionBody(body_, left, right, rig_.legLength(), stance, reposition, dt);
    math::Transform& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)];
    pelvis.position = body_.position;
    pelvis.rotation = body_.rotation() * pelvis.rotation;
}

void Human::repositionFromFeet() {
    // The body follows its targets (motion::BodyReposition): the placed (or
    // stepping) foot moves, the other stands; the limb test's hand target.
    // Targets are the player's intent: each gets as near as the body allows.
    const math::Transform toModel = math::inverse(root_);
    const bool posed = phase_ == Phase::Pose && !poseHand_ && posedFoot_ >= 0;
    motion::BodyTargets t;
    t.planted = posed && !stepWalk_ ? 1 - posedFoot_ : stepPlanted_;
    t.plantedAnkle = toModel.transformPoint(t.planted == 0 ? targets_.footL : targets_.footR);
    t.movingAnkle = toModel.transformPoint(t.planted == 1 ? targets_.footL : targets_.footR);
    t.down[0] = !stepper_.inAir(motion::Foot::Left);
    t.down[1] = !stepper_.inAir(motion::Foot::Right);
    // The body's height comes from its mass on the legs (physics), not from
    // a standing height.
    if (physics_ && humanBody_.ready())
        t.bodyDrop = root_.position.y + restComHeight_ - humanBody_.height(*physics_);
    t.push[0] = stepper_.pushOff(motion::Foot::Left);
    t.push[1] = stepper_.pushOff(motion::Foot::Right);
    // Chest target: the walking posture holds the trunk slightly forward
    // (postureLeanDeg), easing in from standing up to walking speed. The lean
    // of speeding up and slowing down comes later from forces. Positive about
    // +X tips the chest forward.
    {
        const float walking = math::clamp(walker_.speed() / std::max(walkSettings_.speed, 0.1f), 0.0f, 1.0f);
        t.chest = true;
        t.chestRot = math::fromAxisAngle(math::kRight, postureLean_ * walking);
    }
    // Hand targets: each hand swings on an arc round its shoulder opposite
    // the other leg, by that leg's thigh angle (forward as the other foot
    // steps forward; hanging with the feet together): armLegRatio of it.
    // Positive about +X takes a hanging hand back. The limb test's hand
    // target replaces its hand's.
    rig_.localToModel(current_, models_);
    auto thigh = [&](int hip, math::Vec3 ankle) {  // forward angle of the hip-to-ankle line (rad)
        const math::Vec3 h = modelPos(models_, hip);
        return std::atan2(ankle.z - h.z, std::max(h.y - ankle.y, 1e-3f));
    };
    const math::Vec3 ankles[2] = {t.planted == 0 ? t.plantedAnkle : t.movingAnkle, t.planted == 1 ? t.plantedAnkle : t.movingAnkle};
    const float legAngle[2] = {thigh(joints_.legL.start, ankles[0]), thigh(joints_.legR.start, ankles[1])};
    auto swing = [&](int shoulder, math::Vec3 restShoulder, math::Vec3 restWrist, float otherLeg) {
        const float angle = -armLegRatio_ * otherLeg;
        return modelPos(models_, shoulder) + math::fromAxisAngle(math::kRight, angle) * (restWrist - restShoulder);
    };
    t.hand[0] = t.hand[1] = true;
    t.handGoal[0] = swing(joints_.armL.start, restShoulderL_, restWristL_, legAngle[1]);
    t.handGoal[1] = swing(joints_.armR.start, restShoulderR_, restWristR_, legAngle[0]);
    if (phase_ == Phase::Pose && poseHand_) t.handGoal[reachLeft_ ? 0 : 1] = goal_;
    bodyRepositionSettings_.kneePole = ik_.kneePole;
    bodyRepositionSettings_.elbowPole = reach_.elbowPole;
    bodyRepositionSettings_.pelvisMin = reach_.pelvisMin;
    bodyRepositionSettings_.pelvisMax = reach_.pelvisMax;
    bodyReposition_.solve(current_, models_, t, bodyRepositionSettings_);
    // Leaving the test, the usual repositioning carries on from here.
    body_ = motion::BodyState{};
    body_.position = current_.locals[static_cast<size_t>(joints_.pelvis)].position;
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
        // While bowling, the striding foot turns to point straight (the facing
        // before the action) while it is in the air, and stays so.
        const float straighten = stepper_.stepping() ? stepper_.swingProgress() : 1.0f;
        const float yaw = phase_ == Phase::Bowl && strideFoot_ == i ? (bowlBaseYaw_ - walker_.yaw()) * straighten : 0.0f;

        const bool planted = !(stepper_.stepping() && stepper_.swingFoot() == i);
        const math::Quat flat = math::fromAxisAngle(math::kUp, yaw) * math::fromAxisAngle(math::kRight, stepper_.footPitch(f));
        const math::Quat limited = shin * ik::clampRotation(math::normalize(math::inverse(shin) * flat), limits_.limit(legs[i]->end));
        auto toesDown = [](math::Quat q) { return std::asin(math::clamp(-(q * math::kForward).y, -1.0f, 1.0f)); };
        const float tip = planted ? toesDown(limited) - toesDown(flat) : 0.0f;
        const float heel = std::max(tip, 0.0f), toesUp = std::max(-tip, 0.0f);
        heelLift_[i] += (heel - heelLift_[i]) * std::min(1.0f, kHeelRate * tickSeconds_);

        const float pitch = std::max(stepper_.footPitch(f), heelLift_[i]) - toesUp;
        current_.locals[static_cast<size_t>(legs[i]->end)].rotation = math::normalize(
            math::inverse(shin) * math::fromAxisAngle(math::kUp, yaw) * math::fromAxisAngle(math::kRight, pitch));
        if (toes[i] >= 0)
            current_.locals[static_cast<size_t>(toes[i])].rotation =
                math::fromAxisAngle(math::kRight, std::min(stepper_.toeBend(f), -heelLift_[i]));  // toes stay flat
    }
    limits_.apply(current_);
}

void Human::swingArms(float dt) {
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
    if (!jumper_.active()) return;
    // Jumping: each arm's pose from the jump (flexion forward-up is -X; out
    // to the side is +Z for the left arm, -Z for the right), blended over the
    // walk's swing by the jump's arm weight.
    const float weight = jumper_.armWeight();
    for (int i = 0; i < 2; ++i) {
        const bool left = i == 0;
        const ik::TwoBoneChain& chain = left ? joints_.armL : joints_.armR;
        const motion::JumpArm& a = jumper_.arm(left);
        math::Quat& shoulder = current_.locals[static_cast<size_t>(chain.start)].rotation;
        math::Quat& elbow = current_.locals[static_cast<size_t>(chain.mid)].rotation;
        const math::Quat want = math::fromAxisAngle(math::kForward, left ? a.abduct : -a.abduct) *
                                math::fromAxisAngle(math::kRight, -a.flex);
        shoulder = math::normalize(math::slerp(shoulder, want, weight));
        elbow = math::normalize(math::slerp(elbow, math::fromAxisAngle(math::kRight, -a.elbow), weight));
    }
}

void Human::stabiliseHead(float dt) {
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

void Human::rotateInModel(int joint, math::Quat rotation) {
    // Model rotation M = P * L, so a model-space rotation R becomes the local
    // L' = P^-1 * R * P * L.
    const math::Quat parent = modelRot(models_, rig_.parent(joint));
    math::Quat& local = current_.locals[static_cast<size_t>(joint)].rotation;
    local = math::normalize(math::inverse(parent) * rotation * parent * local);
    local = ik::clampRotation(local, limits_.limit(joint));
}

void Human::reachSpine() {
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

void Human::reachPelvis() {
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

void Human::reachBody() {
    // RepositionBody driven by the hand (Step 2.4), on top of the foot
    // repositioning (body_ is not changed, so both apply): the body moves
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

void Human::rebalance() {
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

math::Vec3 Human::balanceOverhang(math::Vec3 pelvisMove) {
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

math::Vec3 Human::clampPelvisToLegs(math::Vec3 pelvis) {
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
    physics_ = &world;
    humanBody_ = HumanBody{};
    if (jointsConfig_) {
        HumanBodySettings& s = humanBodySettings_;
        s.legStiffness = jointsConfig_->getFloat("humanBody.legStiffness", s.legStiffness);
        s.legDamping = jointsConfig_->getFloat("humanBody.legDamping", s.legDamping);
    }
    // At rest: the centre of mass over the root, each leg's length from it to
    // its ankle (the spring's rest length).
    rig_.localToModel(rig_.restPose(), models_);
    const math::Vec3 com = modelCentreOfMass();
    restComHeight_ = com.y;
    const float rest[2] = {math::distance(com, modelPos(models_, joints_.legL.end)),
                           math::distance(com, modelPos(models_, joints_.legR.end))};
    if (!humanBody_.init(world, def_.bodyMass, root_.transformPoint(com), rest, humanBodySettings_)) {
        LOG_ERROR("fielder: no room for the body in the physics world");
        physics_ = nullptr;
        return false;
    }
    return true;
}

math::Vec3 Human::centreOfMass() {
    rig_.localToModel(current_, models_);
    return root_.transformPoint(modelCentreOfMass());
}

void Human::reachArm(bool left, math::Vec3 goal) {
    // Bowling keeps the arm nearly straight: the reach limit is the circle's
    // radius and the IK does not soften toward full extension.
    const bool bowl = phase_ == Phase::Bowl;
    const float armReach = bowl ? bowlRadius() : reach_.armReach * armLength_;
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
    // Bowling, the front (non-bowling) arm: its hand is kept frontClear out
    // from the chest's vertical axis (it never goes inside the body) and its
    // elbow points out to its own side and down, so the forearm stays
    // outside the chest when the hand comes across.
    const bool front = bowl && left != reachLeft_;
    if (front) {
        const math::Vec3 chest = modelPos(models_, joints_.spine[2] >= 0 ? joints_.spine[2] : joints_.pelvis);
        math::Vec3 off(goal.x - chest.x, 0.0f, goal.z - chest.z);
        const float d = math::length(off);
        if (d < bowl_.frontClear) {
            off = d > 0.001f ? off / d : math::kForward;
            goal.x = chest.x + off.x * bowl_.frontClear;
            goal.z = chest.z + off.z * bowl_.frontClear;
        }
    }
    const math::Vec3 dir = math::safeNormalize(goal - shoulder, -math::kUp);
    math::Vec3 side(left ? reach_.elbowPole.x : -reach_.elbowPole.x, reach_.elbowPole.y, reach_.elbowPole.z);
    if (front) side = math::Vec3(left ? 1.0f : -1.0f, -0.5f, 0.0f);  // +X is left
    // Bowling with the yellow circle turned inward: the elbow hint moves
    // toward outward-up on its own side (same out side as reach.elbowPole),
    // by the share elbowSwivel_ (the bowling arm only).
    if (bowl && !front && bowlStage_ != BowlStage::Circle && elbowSwivel_ > 0.0f) {
        const math::Vec3 outUp(left ? 1.0f : -1.0f, 1.0f, 0.0f);  // +X is left
        side = math::lerp(side, outUp, elbowSwivel_);
    }
    const math::Vec3 pole = math::safeNormalize(side - dir * math::dot(side, dir), side);
    ik::solveTwoBone(arm, models_, motion::clampToReach(shoulder, goal, armReach), pole, bowl ? 1.0f : ik_.soften, current_);

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

void Human::grip(bool left, float amount) {
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

void Human::startPick(bool step) {
    if (jumper_.active()) return;
    // At once, whatever the ball is doing: the hand on the ball's side
    // (rest shoulders), unless a fixed one is set.
    if (holding_ || phase_ != Phase::Idle) return;
    const math::Vec3 ball = math::inverse(root_).transformPoint(reachTarget_);
    reachLeft_ = handMode_ == HandMode::Left ||
                 (handMode_ != HandMode::Right && math::distance(restShoulderL_, ball) <= math::distance(restShoulderR_, ball));
    pickDuration_ = handTime(math::distance(reachLeft_ ? restWristL_ : restWristR_, ball));
    phase_ = Phase::Reach;
    pickTime_ = 0.0f;
    if (step && reach_.maxLunge > 0.0f) lungeToward(ball);  // the reach waits for the foot to land
}

void Human::clearReach() {
    if (phase_ == Phase::Reach) startReturn();
}

float Human::handTime(float distance) const {
    return motion::movementTime(distance, reach_.fittsWidth, reach_.fittsA, reach_.fittsB);
}

void Human::startReturn() {
    carryFrom_ = goal_;
    returnDuration_ = handTime(math::distance(carryFrom_, reachLeft_ ? restWristL_ : restWristR_));
    phase_ = Phase::Return;
}

motion::ThrowArc Human::slotArc() const {
    return motion::makeThrowArc(restChest_, math::identityQuat(), reachLeft_, throw_.slotRadiusFactor * armLength_);
}

motion::ThrowArc Human::chestArc(float slotAngle, float tilt) const {
    // Around the throwing shoulder, radius swingRadiusFactor x arm length,
    // toward the slot point (angle 0), in the plane of
    // the slot point and the facing direction (horizontal for a sidearm slot,
    // upright for over- and underarm) turned by `tilt` about the chest-to-slot
    // line. The trunk's current turn carries the shoulder, and so the circle,
    // round the chest; the circle's angles stay those of the hand.
    const motion::ThrowArc swing = motion::makeSwingArc(slotArc(), slotAngle, tilt);
    const math::Vec3 shoulder = reachLeft_ ? restShoulderL_ : restShoulderR_;
    const math::Vec3 toHand = slotArc().point(slotAngle) - shoulder;
    motion::ThrowArc arc = swing;
    arc.radius = throw_.swingRadiusFactor * armLength_;
    arc.side = math::safeNormalize(toHand, swing.side);
    arc.up = math::safeNormalize(swing.up - arc.side * math::dot(swing.up, arc.side), swing.up);
    arc.normal = math::cross(arc.side, arc.up);
    arc.centre = restChest_ + math::fromAxisAngle(twistAxis_, chestTwist_ * throw_.turnShare) * (shoulder - restChest_);
    return arc;
}

math::Vec3 Human::heldHand(float twist) const {
    // The hand stays where the slot click put it relative to the trunk: the
    // trunk's turn (turnShare of the throw angle, about twistAxis_ through
    // the chest) carries it, as it carries the shoulder.
    return restChest_ + math::fromAxisAngle(twistAxis_, twist * throw_.turnShare) * (slotArc().point(throwSlot_) - restChest_);
}

void Human::beginThrow(float slotAngle) {
    bowlTwist_ = false;
    chestLean_ = 0.0f;
    if (!holding_ || (phase_ != Phase::Idle && phase_ != Phase::Return)) return;
    throwSlot_ = slotAngle;
    throwTilt_ = 0.0f;
    throwBack_ = 0.0f;
    throwStage_ = ThrowStage::Ready;
    throwTime_ = 0.0f;
    twistFrom_ = chestTwist_;
    released_ = false;
    throwFrom_ = phase_ == Phase::Return ? goal_ : (reachLeft_ ? restWristL_ : restWristR_);
    goal_ = throwFrom_;
    phase_ = Phase::Throw;
}

void Human::windThrow(float tilt, float backAngle) {
    if (phase_ != Phase::Throw || throwStage_ == ThrowStage::Swing) return;
    throwTilt_ = tilt;
    throwBack_ = math::clamp(backAngle, -math::kTwoPi, 0.0f);
    twistAxis_ = chestArc(throwSlot_, tilt).normal;
    throwStage_ = ThrowStage::Wind;
    throwTime_ = 0.0f;
    twistFrom_ = chestTwist_;
}

void Human::tiltThrow(float tilt) {
    if (phase_ != Phase::Throw || throwStage_ == ThrowStage::Swing) return;
    throwTilt_ = tilt;
    twistAxis_ = chestArc(throwSlot_, tilt).normal;
}

void Human::releaseThrow(float releaseAngle) {
    if (phase_ != Phase::Throw || throwStage_ != ThrowStage::Wind) return;
    releaseAngle_ = math::clamp(releaseAngle, throwBack_ + 0.1f, throwBack_ + math::kTwoPi);
    throwStage_ = ThrowStage::Swing;
    throwTime_ = 0.0f;
    twistFrom_ = chestTwist_;
    throwHeld_ = letGo_ = false;
    // Stride (BIOMECHANICS 10.4: stride length goes with throw speed): with
    // enough force the foot opposite the throwing hand steps along the
    // throw's direction on the ground, longer the harder; the swing starts
    // when it lands (front foot contact, then the trunk turns).
    const float force = math::clamp(-throwBack_ / std::max(throw_.maxBack, 0.01f), 0.0f, 1.0f);
    strideWaiting_ = false;
    if (force >= throw_.strideForce && throw_.strideForce < 1.0f) {
        const motion::Foot foot = reachLeft_ ? motion::Foot::Right : motion::Foot::Left;
        const math::Vec3 along = chestArc(throwSlot_, throwTilt_).tangent(releaseAngle_);
        const math::Vec3 flat = math::safeNormalize(math::Vec3(along.x, 0.0f, along.z), math::kForward);
        const float share = (force - throw_.strideForce) / (1.0f - throw_.strideForce);
        strideWaiting_ = stepper_.stepTo(foot, stepper_.foot(foot) + flat * math::lerp(throw_.minStride, throw_.maxStride, share));
    }
}

void Human::cancelThrow() {
    if (phase_ == Phase::Throw && throwStage_ != ThrowStage::Swing) startReturn();
}

void Human::updateThrow(float dt) {
    // The trunk drives the throw (BIOMECHANICS 10.2, proximal to distal):
    // the hand goes to the slot point and holds; the chest winds back to the
    // force point carrying the hand with it (its place on the trunk does not
    // change) and holds; then forward speeding up (s = t^2), the hand
    // blending out onto the throw circle (around the shoulder) and on along
    // it to the release point while it pushes out along the throw, on past it slowing down; then the hand returns and the
    // chest turns back square. Character space.
    if (phase_ != Phase::Throw) {
        chestTwist_ -= chestTwist_ * std::min(1.0f, dt * throw_.untwistRate);
        chestLean_ -= chestLean_ * std::min(1.0f, dt * throw_.untwistRate);
        return;
    }
    throwTime_ += dt;
    const motion::ThrowArc arc = chestArc(throwSlot_, throwTilt_);
    auto smooth = [](float u) { return u * u * (3.0f - 2.0f * u); };
    float t = throwTime_;
    if (throwStage_ == ThrowStage::Ready) {
        const float u = smooth(std::min(1.0f, t / std::max(throw_.readyTime, 0.01f)));
        chestTwist_ = twistFrom_ * (1.0f - u);
        goal_ = math::lerp(throwFrom_, slotArc().point(throwSlot_), u);
        return;
    }
    if (throwStage_ == ThrowStage::Wind) {
        // The hand keeps its place on the trunk as the chest winds back.
        const float u = smooth(std::min(1.0f, t / std::max(throw_.windTime, 0.01f)));
        chestTwist_ = math::lerp(twistFrom_, throwBack_, u);
        goal_ = heldHand(chestTwist_);
        return;
    }
    if (strideWaiting_) {
        // The chest stays wound up while the stride foot is in the air.
        throwTime_ = 0.0f;
        strideWaiting_ = stepper_.stepping();
        goal_ = heldHand(chestTwist_);
        return;
    }
    const float swing = std::max(throw_.swingTime, 0.01f), follow = std::max(throw_.followTime, 0.01f);
    const math::Vec3 pushDir = arc.tangent(releaseAngle_);
    if (t < swing) {
        // Forward with the trunk while blending out onto the throw circle, on
        // it by blendShare of the way to the release; then on along it.
        const float u = t / swing;
        chestTwist_ = math::lerp(twistFrom_, releaseAngle_, u * u);
        const float onOuter = smooth(std::min(1.0f, u * u / std::max(throw_.blendShare, 0.01f)));
        goal_ = math::lerp(heldHand(chestTwist_), arc.point(chestTwist_), onOuter) + pushDir * (throw_.push * u * u);
    } else if (holding_ && throw_.pauseAtRelease && !letGo_) {
        // Test: wait at the release point with the ball.
        throwHeld_ = true;
        throwTime_ = swing;
        chestTwist_ = releaseAngle_;
        goal_ = arc.point(releaseAngle_) + pushDir * throw_.push;
    } else if ((t -= swing) < follow) {
        throwHeld_ = false;
        if (holding_) {
            // The ball leaves at the release point along the circle; its
            // world position is taken from the solved hand (tick()). The
            // farther the wind-up, the faster.
            holding_ = false;
            released_ = true;
            const float force = math::clamp(-throwBack_ / std::max(throw_.maxBack, 0.01f), 0.0f, 1.0f);
            releaseVelocity_ = root_.transformDirection(pushDir) * math::lerp(throw_.minSpeed, throw_.maxSpeed, force);
        }
        const float u = 1.0f - t / follow;
        chestTwist_ = releaseAngle_ + throw_.followAngle * (1.0f - u * u);
        goal_ = arc.point(chestTwist_) + pushDir * (throw_.push * u);
    } else {
        startReturn();
        pickTime_ = 1.0f;
    }
}

void Human::twistChest() {
    // The trunk turns turnShare of the throw angle about twistAxis_ (the arm
    // swings round the shoulder for the rest): pelvisShare on the pelvis (clamped to
    // its limits), the rest over spine_01, spine_02 and chest by the reach
    // twist weights (9.3: mostly thoracic), each clamped to its limit.
    // Bowling turns the chest by chestLean_, chest bone first; what its
    // limit leaves goes to spine_02, then spine_01 (the hips stay; the whole
    // player turns for the rest, see walk()).
    if (joints_.spine[0] < 0 || joints_.spine[1] < 0 || joints_.spine[2] < 0) return;
    if (bowlTwist_) {
        float left = chestLean_;
        for (int i = 2; i >= 0 && std::fabs(left) >= 1e-4f; --i) {
            const int joint = joints_.spine[i];
            const math::Quat before = current_.locals[static_cast<size_t>(joint)].rotation;
            rotateInModel(joint, math::fromAxisAngle(leanAxis_, left));
            // What the limit let through: the turn about the vertical between before and after.
            const math::Quat applied = current_.locals[static_cast<size_t>(joint)].rotation * math::inverse(before);
            const math::Vec3 axis(applied.x, applied.y, applied.z);
            left -= 2.0f * std::atan2(math::dot(axis, math::Vec3(0.0f, 1.0f, 0.0f)), applied.w) *
                    (leanAxis_.y < 0.0f ? -1.0f : 1.0f);
            rig_.localToModel(current_, models_);
        }
        return;
    }
    if (std::fabs(chestTwist_) < 1e-4f) return;
    const float pelvisTurn = chestTwist_ * throw_.turnShare * throw_.pelvisShare;
    math::Quat& pelvis = current_.locals[static_cast<size_t>(joints_.pelvis)].rotation;
    pelvis = motion::clampRotationVector(
        math::normalize(math::fromAxisAngle(twistAxis_, pelvisTurn) * pelvis),
        reach_.pelvisMin,
        reach_.pelvisMax);
    rig_.localToModel(current_, models_);
    const float share[3] = {reach_.spineTwist.x, reach_.spineTwist.y, reach_.spineTwist.z};
    const float total = std::max(share[0] + share[1] + share[2], 1e-3f);
    const float spine = chestTwist_ * throw_.turnShare * (1.0f - throw_.pelvisShare);
    for (int i = 0; i < 3; ++i) {
        rotateInModel(joints_.spine[i], math::fromAxisAngle(twistAxis_, spine * share[i] / total));
        rig_.localToModel(current_, models_);
    }
}

void Human::startBowl(float tilt, float releaseAngle) {
    if (!holding_ || (phase_ != Phase::Idle && phase_ != Phase::Return)) return;
    throwTilt_ = tilt;
    // Release between the horizontal behind and the horizontal in front.
    releaseAngle_ = math::clamp(releaseAngle, -math::kHalfPi + 0.1f, math::kHalfPi);
    throwTime_ = 0.0f;
    released_ = false;
    throwFrom_ = phase_ == Phase::Return ? goal_ : (reachLeft_ ? restWristL_ : restWristR_);
    bowlStage_ = BowlStage::Circle;
    phase_ = Phase::Bowl;
}

motion::ThrowArc Human::bowlArc(float tilt) const {
    // The bowling circle around the rest shoulder, square to the batter,
    // leaned by `tilt` out to the bowling-arm side; it stays put while the
    // body moves.
    return motion::makeBowlingArc(shoulderNow(reachLeft_) + releaseShift_, math::identityQuat(), reachLeft_,
                                  bowlRadius(), tilt);
}

math::Vec3 Human::arcBetween(math::Vec3 from, math::Vec3 to, float u) const {
    return arcAround(shoulderNow(reachLeft_), from, to, u);
}

math::Vec3 Human::frontPathPoint(math::Vec3 point, float gap, float u) const {
    // From the front hand at rest round its own shoulder (via the front) to
    // the end of the bowling hand's way, kept apart by frontGap per axis.
    const math::Vec3 shoulder = shoulderNow(!reachLeft_);
    const math::Vec3 end = actionPathPoint(point, gap, 1.0f);
    return arcAround(shoulder, restHandNow(!reachLeft_), end - (end - shoulder) * bowl_.frontGap, u, true);
}

math::Vec3 Human::arcAround(math::Vec3 shoulder, math::Vec3 from, math::Vec3 to, float u, bool viaFront) {
    // In the plane of the shoulder, `from` and `to`: round the shoulder the
    // short way (with `viaFront`, the other way if the short way would set
    // off backward), the radius going from from's to to's.
    const math::Vec3 f = from - shoulder, d = to - shoulder;
    const math::Vec3 e0 = math::safeNormalize(f, -math::kUp);
    const math::Vec3 e1 = math::safeNormalize(d - e0 * math::dot(d, e0), math::kForward);
    float total = std::atan2(math::dot(d, e1), math::dot(d, e0));  // 0..pi toward e1
    if (viaFront && math::dot(e1, math::kForward) < 0.0f) total -= math::kTwoPi;
    const float a = total * u;
    const float r = math::lerp(math::length(f), math::length(d), u);
    return shoulder + (e0 * std::cos(a) + e1 * std::sin(a)) * r;
}

math::Vec3 Human::shoulderNow(bool left) const { return left ? liveShoulderL_ : liveShoulderR_; }

math::Vec3 Human::restHandNow(bool left) const {
    return (left ? restWristL_ : restWristR_) + shoulderNow(left) - (left ? restShoulderL_ : restShoulderR_);
}

math::Vec3 Human::bottomPoint() const {
    return (shoulderNow(reachLeft_)) - math::kUp * bowlRadius();
}

math::Vec3 Human::frontDownPoint(math::Vec3 point, float gap, float u) const {
    return frontPathPoint(point, gap, 1.0f - u);  // back down the same way
}

math::Vec3 Human::actionPathPoint(math::Vec3 point, float gap, float u) const {
    // From the resting hand round the shoulder to `gap` beyond the point.
    const math::Vec3 shoulder = shoulderNow(reachLeft_);
    const math::Vec3 d = point - shoulder;
    return arcAround(shoulder, restHandNow(reachLeft_), point + math::safeNormalize(d, math::kUp) * gap, u, true);
}

motion::ThrowArc Human::actionCircle(float turn, float radius) const {
    // +X is left: the bowling-arm side is +X for the left arm, -X for the
    // right; the sign makes + turn go toward it.
    const float side = reachLeft_ ? 1.0f : -1.0f;
    motion::ThrowArc arc;
    arc.centre = shoulderNow(reachLeft_);
    arc.side = -math::kUp;
    arc.up = math::fromAxisAngle(math::kUp, side * turn) * math::kForward;
    arc.normal = math::cross(arc.side, arc.up);
    arc.radius = radius;
    return arc;
}

void Human::setActionPoint(math::Vec3 point) {
    // The farther behind the shoulder, the more pace.
    actionOffset_ = point - shoulderNow(reachLeft_);
    const math::Vec3 d = math::safeNormalize(point - (shoulderNow(reachLeft_)), math::kUp);
    bowlPace_ = math::clamp(0.5f * (1.0f - math::dot(d, math::kForward)), 0.0f, 1.0f);
}

void Human::actionBowl(math::Vec3 point, float gap, float turn) {
    if (!holding_ || (phase_ != Phase::Idle && phase_ != Phase::Return)) return;
    // + turn is outward (bowling-arm side): no swivel.
    elbowSwivel_ = math::clamp(-turn / std::max(bowl_.elbowUpTurn, 0.01f), 0.0f, 1.0f);
    // Only the bowling hand is driven; the body follows it through the reach
    // stages (reverse IK).
    setActionPoint(point);
    actionGap_ = gap;
    // Chest turn (side-on, away from the batter; the chest bone alone, the
    // body stays): chestBase always, growing once the point is less than
    // coilAhead in front of the shoulder, maxCoil an arm length behind.
    const float ahead = point.z - (reachLeft_ ? restShoulderL_ : restShoulderR_).z;
    coil_ = math::lerp(bowl_.chestBase, bowl_.maxCoil,
                       math::clamp((bowl_.coilAhead - ahead) / std::max(bowl_.coilAhead + armLength_, 0.01f), 0.0f, 1.0f));
    bowlBaseYaw_ = walker_.yaw();
    bowlYaw_ = bowlBaseYaw_;
    strideFoot_ = -1;
    leanAxis_ = math::Vec3(0.0f, reachLeft_ ? 1.0f : -1.0f, 0.0f);
    bowlTwist_ = true;
    LOG_INFO("Bowl action: chest turn {:.0f} deg", math::degrees(coil_));
    throwTime_ = 0.0f;
    released_ = false;
    throwFrom_ = phase_ == Phase::Return ? goal_ : (reachLeft_ ? restWristL_ : restWristR_);
    bowlStage_ = BowlStage::Coil;
    releaseShift_ = math::Vec3(0.0f);
    followThrough_ = false;  // a new delivery never starts mid run-on
    frontGoal_ = restHandNow(!reachLeft_);
    phase_ = Phase::Bowl;
}

bool Human::bodyTurned() const {
    return std::abs(std::remainder(bowlYaw_ - walker_.yaw(), math::kTwoPi)) <= bowl_.turnDone;
}

void Human::downBowl() {
    if (!actionHeld()) return;
    // From the action point round the shoulder down to the bottom point.
    // Whole-player turn once the hand leaves the action point: bodyBase (the
    // release circle is picked only at the bottom).
    backPoint_ = bottomPoint();
    bodyTurn_ = (reachLeft_ ? 1.0f : -1.0f) * bowl_.bodyBase;  // + yaw: toward +X
    bowlStage_ = BowlStage::Back;
    throwTime_ = 0.0f;
    strideWaiting_ = true;  // the front foot steps once the hand is part way down (updateBowl)
}

void Human::releaseBowl(float releaseAngle, float releaseTilt) {
    if (!armDown()) return;
    // From the bottom point on round the release circle the bowling way
    // (larger angles: from the bottom, back, over the top) to the release.
    releaseTilt_ = releaseTilt;
    swingFrom_ = math::kPi;
    swingTo_ = releaseAngle;
    while (swingTo_ <= swingFrom_) swingTo_ += math::kTwoPi;
    while (swingTo_ > swingFrom_ + math::kTwoPi) swingTo_ -= math::kTwoPi;
    bowlStage_ = BowlStage::Swing;
    throwTime_ = 0.0f;
}

void Human::cancelCoil() {
    if (coiled() || armDown()) startReturn();
}

void Human::setPoseHand(bool left, math::Vec3 world) {
    if (phase_ != Phase::Idle && phase_ != Phase::Pose) return;
    phase_ = Phase::Pose;
    poseHand_ = true;
    reachLeft_ = left;
    poseGoal_ = world;
}

void Human::setPoseFoot(motion::Foot foot, math::Vec3 world) {
    if (phase_ != Phase::Idle && phase_ != Phase::Pose) return;
    phase_ = Phase::Pose;
    posedFoot_ = static_cast<int>(foot);
    stepper_.place(foot, math::inverse(root_).transformPoint(world));
}

void Human::clearPose() {
    if (phase_ != Phase::Pose) return;
    phase_ = Phase::Idle;
    posedFoot_ = -1;
    bodyReposition_.reset();
    poseHand_ = false;
    plantFeet();
}

void Human::updateBowl(float dt) {
    // BIOMECHANICS 11.2: angles on the bowling circle (0 top, +90 in front),
    // unwrapped so the hand only moves forward: start (front, low) -> past the
    // bottom -> 3/2 pi (horizontal behind) -> top -> release, speeding up all
    // the way; then it slows evenly to a stop on the follow-through while
    // moving across the body, and the usual return carries it back.
    // Character space (rest shoulder).
    if (phase_ != Phase::Bowl) return;
    throwTime_ += dt * bowl_.slowMotion;
    const math::Vec3 actionPoint = shoulderNow(reachLeft_) + actionOffset_;  // rides the shoulder
    const math::Vec3 bottom = bottomPoint();
    auto smooth = [](float u) { return u * u * (3.0f - 2.0f * u); };
    if (bowlStage_ == BowlStage::Coil) {
        // The hand goes up round the shoulder from rest past the action point
        // by the gap, then is pulled back onto it (the wrist loads) and holds.
        const float reach = std::max(coilReach(), 0.01f);
        const math::Vec3 rest = reachLeft_ ? restWristL_ : restWristR_;
        chestLean_ = coil_ * smooth(std::min(1.0f, throwTime_ / reach));
        math::Vec3 onPath;
        if (throwTime_ < reach) {
            const float u = smooth(throwTime_ / reach);
            onPath = actionPathPoint(actionPoint, actionGap_, u);
            goal_ = onPath + (throwFrom_ - rest) * (1.0f - u);
        } else {
            const float v = smooth(std::min(1.0f, (throwTime_ - reach) / std::max(bowl_.pullTime, 0.01f)));
            onPath = math::lerp(actionPathPoint(actionPoint, actionGap_, 1.0f), actionPoint, v);
            goal_ = onPath;
        }
        // The front hand starts frontDelay later on its own way and joins the
        // bowling hand at the end of its way, then stays with it.
        const float front = (throwTime_ - bowl_.frontDelay) / std::max(reach - bowl_.frontDelay, 0.01f);
        frontGoal_ = frontPathPoint(actionPoint, actionGap_, smooth(math::clamp(front, 0.0f, 1.0f)));
        // Run-up delivery: the gather reached, the hand goes on down without
        // a pause (reaching the release circle as the back foot lands).
        if (autoDeliver_ && actionHeld()) {
            frontFrom_ = frontGoal_;
            downBowl();
        }
        return;
    }
    if (bowlStage_ == BowlStage::Back) {
        // From the action point round the shoulder down to the bottom point,
        // speeding up; then it holds there (Down) for the release click.
        const float back = std::max(landTurn_ ? bowl_.runBackTime : bowl_.backTime, 0.01f);
        const float way = std::min(1.0f, throwTime_ / back);
        // After a run-up the front foot is in the air: it strides at once, straight down from there
        // (front-foot contact ~0.1-0.15 s after back-foot contact).
        if (strideWaiting_ && !autoDeliver_ && way * way >= bowl_.strideAt) {  // share of the way down (it speeds up as u^2)
            // The front foot (opposite the bowling arm) steps straight ahead,
            // to strideLength in front of the back foot (wherever a landing
            // left it; a foot already ahead steps less).
            strideWaiting_ = false;
            const motion::Foot foot = reachLeft_ ? motion::Foot::Right : motion::Foot::Left;
            const motion::Foot back = reachLeft_ ? motion::Foot::Left : motion::Foot::Right;
            const math::Vec3 straight = math::fromAxisAngle(math::kUp, bowlBaseYaw_ - walker_.yaw()) * math::kForward;
            const float along = math::dot(stepper_.foot(back) - stepper_.foot(foot), straight);
            stepper_.stepTo(foot, stepper_.foot(foot) + straight * (along + bowl_.strideLength));
            strideFoot_ = static_cast<int>(foot);
        }
        if (throwTime_ < back) {
            const float u = throwTime_ / back;
            chestLean_ = coil_;  // held
            bowlYaw_ = bowlBaseYaw_ + bodyTurn_ * u * u;  // turns with the hand, full at the bottom point
            goal_ = arcBetween(actionPoint, bottom, u * u);
            frontGoal_ = frontDownPoint(actionPoint, actionGap_, u * u);  // the front hand down its own way, alongside
            if (autoDeliver_) {
                // Run-up delivery: the front arm reaches up and out at the
                // batter (straight ahead before the turn) through the stride.
                const math::Vec3 batter = math::fromAxisAngle(math::kUp, bowlBaseYaw_ - walker_.yaw()) * math::kForward;
                const math::Vec3 high =
                    shoulderNow(!reachLeft_) + math::normalize(batter + math::kUp * 0.5f) * (armLength_ * 0.95f);
                frontGoal_ = math::lerp(frontFrom_, high, smooth(u));
            }
            return;
        }
        bowlStage_ = BowlStage::Down;
        if (autoDeliver_) frontFrom_ = frontGoal_;
    }
    if (bowlStage_ == BowlStage::Down) {
        chestLean_ = coil_;
        bowlYaw_ = bowlBaseYaw_ + bodyTurn_;
        goal_ = bottom;
        if (!autoDeliver_) {
            frontGoal_ = frontDownPoint(actionPoint, actionGap_, 1.0f);
            return;
        }
        // Run-up delivery: over the top from back-foot contact, once the
        // front foot's stride is under way; timed so it is down before the
        // release.
        const int front = static_cast<int>(reachLeft_ ? motion::Foot::Right : motion::Foot::Left);
        if (!landTurn_ || stepper_.swingFoot() != front) return;
        autoSwingTime_ = std::max(bowl_.runSwingTime, stepper_.swingTimeLeft() + 0.02f);
        releaseBowl(autoReleaseAngle_, autoReleaseTilt_);
    }
    if (bowlStage_ == BowlStage::Swing) {
        // The hand goes round speeding up (s = u^2); the reach stages turn,
        // bend and fold the body after it.
        const float swing = std::max(autoDeliver_ ? autoSwingTime_ : bowl_.swingTime, 0.01f),
                    follow = std::max(bowl_.swingFollowTime, 0.01f);
        float t = throwTime_;
        if (t < swing) {
            const float u = t / swing;
            bowlYaw_ = bowlBaseYaw_ + bodyTurn_ * (1.0f - u * u);  // back with the hand, straight at the release
            chestLean_ = coil_ * (1.0f - u * u);  // unwinds with the hand, square at the release
            // From the bottom point onto the release circle by a quarter turn.
            // The circle moves forward toward the batter as the arm comes
            // over; the reach stages lean the body after the hand.
            releaseShift_ = math::fromAxisAngle(math::kUp, bowlBaseYaw_ - walker_.yaw()) * math::kForward *
                            (bowl_.releaseLean * u * u);
            const float angle = swingFrom_ + (swingTo_ - swingFrom_) * u * u;
            const float onCircle = smooth(math::clamp((angle - swingFrom_) / math::kHalfPi, 0.0f, 1.0f));
            const motion::ThrowArc arc = bowlArc(releaseTilt_);
            goal_ = arc.point(angle) + (bottom - arc.point(swingFrom_)) * (1.0f - onCircle);
            // The front arm pulls down and tucks in to the side (eased from
            // its high reach on a run-up delivery).
            frontGoal_ = autoDeliver_ ? math::lerp(frontFrom_, restHandNow(!reachLeft_), smooth(u)) : restHandNow(!reachLeft_);
            return;
        }
        if (holding_) {
            // Release on the circle's tangent. The world position is taken
            // from the solved hand (tick()).
            if (checkStops_ && autoDeliver_) checkHeld_ = true;  // check stop: the release point
            holding_ = false;
            released_ = true;
            // Along the facing before the action (toward the batter), whatever
            // the body's turn still is.
            const math::Quat facing = math::fromAxisAngle(math::kUp, bowlBaseYaw_);
            releaseVelocity_ = (facing * bowlArc(releaseTilt_).tangent(swingTo_)) *
                               math::lerp(bowl_.minSpeed, bowl_.maxSpeed, bowlPace_);
        }
        if ((t -= swing) < follow) {
            // The arm carries on, slowing, and the hand is carried forward,
            // so the body folds after it.
            const float u = t / follow;
            const float s = 1.0f - (1.0f - u) * (1.0f - u);
            goal_ = bowlArc(releaseTilt_).point(swingTo_ + bowl_.followAngle * s) +
                    math::kForward * (bowl_.followReach * smooth(u));
            frontGoal_ = reachLeft_ ? restWristR_ : restWristL_;
            return;
        }
        startReturn();
        pickTime_ = 1.0f;
        return;
    }
    const motion::ThrowArc arc = motion::makeBowlingArc(reachLeft_ ? restShoulderL_ : restShoulderR_, math::identityQuat(),
                                                        reachLeft_, bowlRadius(), throwTilt_);
    const float a0 = bowl_.startAngle;
    const float a1 = math::kTwoPi - math::kHalfPi;  // horizontal behind
    const float a2 = math::kTwoPi + releaseAngle_;  // release
    const float setup = std::max(bowl_.setupTime, 0.01f);
    const float t1 = std::max(bowl_.downBackTime, 0.01f);
    const float t2 = std::max(bowl_.overTime, 0.01f);
    const float t3 = std::max(bowl_.followTime, 0.01f);
    // Down and back from rest (a0 + d u^2) ends at speed v1; over the top
    // accelerates evenly to reach the release point at t2.
    const float v1 = 2.0f * (a1 - a0) / t1;
    const float acc = 2.0f * (a2 - a1 - v1 * t2) / (t2 * t2);
    const float vRelease = v1 + acc * t2;
    if (holding_ && throwTime_ >= setup + t1 + t2) {
        // The ball leaves at the release point, along the circle; its world
        // position is taken from the solved hand (tick()).
        holding_ = false;
        released_ = true;
        releaseVelocity_ = root_.transformDirection(arc.tangent(a2)) * bowl_.speed;
    }
    float t = throwTime_;
    if (t < setup) {
        const float u = t / setup;
        goal_ = math::lerp(throwFrom_, arc.point(a0), u * u * (3.0f - 2.0f * u));
    } else if ((t -= setup) < t1) {
        const float u = t / t1;
        goal_ = arc.point(a0 + (a1 - a0) * u * u);
    } else if ((t -= t1) < t2) {
        goal_ = arc.point(a1 + v1 * t + 0.5f * acc * t * t);
    } else if ((t -= t2) < t3) {
        const math::Vec3 p = arc.point(a2 + vRelease * t - 0.5f * vRelease / t3 * t * t);
        const math::Vec3 across(-p.x, p.y, p.z);
        const float u = t / t3;
        goal_ = math::lerp(p, across, bowl_.followAcross * u * u * (3.0f - 2.0f * u));
    } else {
        startReturn();
        pickTime_ = 1.0f;
    }
}

void Human::updatePick(float dt) {
    if (phase_ == Phase::Idle || phase_ == Phase::Throw || phase_ == Phase::Bowl || phase_ == Phase::Pose) return;
    const math::Transform toModel = math::inverse(root_);
    const math::Vec3 ball = toModel.transformPoint(reachTarget_);

    // Progress eases in and out (smoothstep) over the Fitts' law times, so
    // the hand leads and the body, driven by the same goal through every
    // stage, follows.
    if (phase_ == Phase::Reach && stepper_.stepping() && pickTime_ <= 0.0f) return;
    if (phase_ == Phase::Reach) pickTime_ = std::min(1.0f, pickTime_ + dt / std::max(pickDuration_, 0.05f));
    else pickTime_ = std::max(0.0f, pickTime_ - dt / std::max(returnDuration_, 0.05f));
    if (phase_ == Phase::Return && pickTime_ <= 0.0f) {
        phase_ = Phase::Idle;
        return;
    }
    const float s = pickTime_ * pickTime_ * (3.0f - 2.0f * pickTime_);

    // Wrist goal on the ball: back from it toward the shoulder (palm on the ball).
    const math::Vec3 shoulder = reachLeft_ ? restShoulderL_ : restShoulderR_;
    const math::Vec3 onBall = ball + math::safeNormalize(shoulder - ball, math::kUp) * reach_.wristOffset;
    const math::Vec3 end = phase_ == Phase::Reach ? onBall : carryFrom_;
    goal_ = math::lerp(reachLeft_ ? restWristL_ : restWristR_, end, s);
}

void Human::finishPick(float) {
    // The reach is done (at full stretch if the ball is out of reach): a hand
    // on the ball grips it; either way it goes back to the standing pose.
    if (phase_ != Phase::Reach || pickTime_ < 1.0f) return;
    holding_ = reachError_ <= reach_.gripTolerance;
    startReturn();
}

void Human::lungeToward(math::Vec3 ball) {
    // The foot nearer the ball steps to land beside it (BIOMECHANICS 8: low
    // pickups put the foot by the ball), lungeShort short of it, at most maxLunge.
    const motion::Foot foot = math::distance(stepper_.foot(motion::Foot::Left), ball) <=
                                      math::distance(stepper_.foot(motion::Foot::Right), ball)
                                  ? motion::Foot::Left
                                  : motion::Foot::Right;
    const math::Vec3 from = stepper_.foot(foot);
    const math::Vec3 flat(ball.x - from.x, 0.0f, ball.z - from.z);
    const float length = math::clamp(math::length(flat) - reach_.lungeShort, 0.0f, reach_.maxLunge);
    stepper_.stepTo(foot, from + math::safeNormalize(flat, math::kForward) * length);
}

math::Vec3 Human::ballSpot(bool left) const {
    // Half the palm down the hand (-Y), out along the palm normal.
    return math::Vec3(0.0f, -reach_.palmCentre, 0.0f) +
           math::Vec3(left ? -1.0f : 1.0f, 0.0f, 0.0f) * (reach_.ballRadius + reach_.palmThickness);
}

math::Vec3 Human::ballInHand(const std::vector<math::Mat4>& models, const math::Transform& root) const {
    // In front of the holding palm.
    const ik::TwoBoneChain& arm = reachLeft_ ? joints_.armL : joints_.armR;
    return root.transformPoint(math::Vec3(models[static_cast<size_t>(arm.end)] * math::Vec4(ballSpot(reachLeft_), 1.0f)));
}

void Human::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
    