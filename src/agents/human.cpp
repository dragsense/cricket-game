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

} // namespace

bool Human::init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion) {
    def_ = def;
    jointsConfig_ = &joints;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);
    previousRoot_ = root_;
    loadLocomotion(locomotion);
    ik_.kneePole = joints.getVec3("ik.kneePole", ik_.kneePole);
    return setHeight(height);
}

void Human::loadLocomotion(const core::Config& c) {
    // Load-time only (builds path strings): the decider's balance step
    // (locomotion.json "step": its shape, its passive shares; its timing is
    // the decider's own, per step).
    toeClearance_ = c.getFloat("step.toeClearance", toeClearance_);
    stepSettings_.stepHeight = c.getFloat("step.stepHeight", stepSettings_.stepHeight);
    stepSettings_.peakAt = c.getFloat("step.swingPeakAt", stepSettings_.peakAt);
    toeOffPitch_ = math::radians(c.getFloat("step.toeOffPitchDeg", math::degrees(toeOffPitch_)));
    stepPassive_.head = c.getFloat("step.passive.head", stepPassive_.head);
    stepPassive_.hands = c.getFloat("step.passive.hands", stepPassive_.hands);
    stepPassive_.hips = c.getFloat("step.passive.hips", stepPassive_.hips);
    stepPassive_.chest = c.getFloat("step.passive.chest", stepPassive_.chest);
    motion::StepPlannerSettings& shape = deciderSettings_.stepShape;
    shape.height = stepSettings_.stepHeight;
    shape.peakAt = stepSettings_.peakAt;
    shape.toeOffPitch = toeOffPitch_;
    deciderSettings_.stepPassive = stepPassive_;
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
    models_.assign(static_cast<size_t>(rig_.numJoints()), math::Mat4(1.0f));
    plantFeet();
    return true;
}

void Human::placeAt(math::Vec3 position, float yaw) {
    root_.position = position;
    root_.rotation = math::fromAxisAngle(math::kUp, yaw);
    previousRoot_ = root_;
    if (physics_) physics_->placeCharacter(capsule_, position);
    plantFeet();
}

void Human::followCapsule(float dt) {
    if (!physics_ || capsule_ == physics::kInvalidCharacter || dt <= 0.0f) return;
    math::Vec3 velocity(0.0f);  // let go (no hold): the capsule stays
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
    previous_.locals = current_.locals;  // same size: no allocation
    previousRoot_ = root_;
    current_.locals = rig_.restPose().locals;
    followCapsule(static_cast<float>(dt));
    // The full-physics body: its muscles pull toward the decider's plan for
    // the limb test's targets, the capsule holding it up.
    if (physics_ && humanBody_.ready()) {
        rig_.localToModel(current_, models_);
        const std::vector<math::Mat4>* plan = limbTestPlan(static_cast<float>(dt));
        physicsTick(plan ? *plan : models_, static_cast<float>(dt));
    }
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
            deciderSettings_.stepShape.ballReach = ballReach_;
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
    rig_.localToModel(rig_.restPose(), models_);  // the body starts standing
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
                    // One axis's pair, or a list of them (one per axis).
                    const nlohmann::json one = nlohmann::json::array({*it});
                    for (const nlohmann::json& v : it->is_array() ? *it : one) {
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
                }
        if (auto m = root.find("muscles"); m != root.end())
            if (auto t = m->find("stiffness"); t != m->end() && t->is_object())
                for (auto it = t->begin(); it != t->end(); ++it) {
                    if (!it->is_object() || !it->contains("relaxed")) {
                        LOG_WARN("fielder: muscles.stiffness.{} needs relaxed", it.key());
                        continue;
                    }
                    MuscleSettings::Spring s;
                    s.bone = it.key();
                    const nlohmann::json& r = (*it)["relaxed"];
                    if (r.is_array() && r.size() == 3) s.relaxed = math::Vec3(r[0].get<float>(), r[1].get<float>(), r[2].get<float>());
                    else if (r.is_number()) s.relaxed = math::Vec3(r.get<float>());
                    s.slope = it->value("slope", 0.0f);
                    s.damping = it->value("damping", 0.0f);
                    s.dampingSlope = it->value("dampingSlope", 0.0f);
                    muscles.springs.push_back(s);
                }
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
        // A step's swing is a foot's aimed move too: the same Fitts' law.
        deciderSettings_.fittsA = muscleTiming_.fittsA;
        deciderSettings_.fittsB = muscleTiming_.fittsB;
        deciderSettings_.fittsWidth = muscleTiming_.fittsWidth;
        deciderSettings_.armCancel = jointsConfig_->getFloat("decider.armCancel", deciderSettings_.armCancel);
        deciderSettings_.passiveHead = jointsConfig_->getFloat("decider.passiveInfluence.head", deciderSettings_.passiveHead);
        deciderSettings_.passiveHands = jointsConfig_->getFloat("decider.passiveInfluence.hands", deciderSettings_.passiveHands);
        deciderSettings_.passiveHips = jointsConfig_->getFloat("decider.passiveInfluence.hips", deciderSettings_.passiveHips);
        deciderSettings_.passiveChest = jointsConfig_->getFloat("decider.passiveInfluence.chest", deciderSettings_.passiveChest);
        auto dv = [&](const char* key, float value) { return jointsConfig_->getFloat(key, value); };
        deciderSettings_.pelvisList =
            math::radians(dv("decider.pelvisListDeg", math::degrees(deciderSettings_.pelvisList)));
        deciderSettings_.kneeForwardBias = dv("decider.kneeForwardBias", deciderSettings_.kneeForwardBias);
        deciderSettings_.standingReachShare = dv("decider.standingReachShare", deciderSettings_.standingReachShare);
        deciderSettings_.trunkTurnFrequency = dv("decider.trunkTurn.frequency", deciderSettings_.trunkTurnFrequency);
        deciderSettings_.trunkTurnDamping = dv("decider.trunkTurn.damping", deciderSettings_.trunkTurnDamping);
        deciderSettings_.limbMinBend = math::radians(dv("decider.limbMinBendDeg", math::degrees(deciderSettings_.limbMinBend)));
        deciderSettings_.blendTime = dv("decider.blendTime", deciderSettings_.blendTime);
        deciderSettings_.supportBlendTime = dv("decider.supportBlendTime", deciderSettings_.supportBlendTime);
        footSupport_.heelIn = dv("footSupport.heelIn", footSupport_.heelIn);
        footSupport_.toeIn = dv("footSupport.toeIn", footSupport_.toeIn);
        footSupport_.width = dv("footSupport.width", footSupport_.width);
        deciderSettings_.hipsMaxSpeed = dv("decider.hipsMaxSpeed", deciderSettings_.hipsMaxSpeed);
        deciderSettings_.hipsMaxAccel = dv("decider.hipsMaxAccel", deciderSettings_.hipsMaxAccel);
        deciderSettings_.hipsMaxTurnRate = dv("decider.hipsMaxTurnRate", deciderSettings_.hipsMaxTurnRate);
        deciderSettings_.trunkMaxRate = dv("decider.trunkMaxRate", deciderSettings_.trunkMaxRate);
        deciderSettings_.headMaxRate = dv("decider.headMaxRate", deciderSettings_.headMaxRate);
        deciderSettings_.limbMaxRate = dv("decider.limbMaxRate", deciderSettings_.limbMaxRate);
        deciderSettings_.legMaxRate = dv("decider.legMaxRate", deciderSettings_.legMaxRate);
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
        deciderSettings_.stopBlendSpeed = dv("decider.balanceStep.stopBlendSpeed", deciderSettings_.stopBlendSpeed);
        deciderSettings_.minFootGap = dv("decider.balanceStep.minFootGap", deciderSettings_.minFootGap);
        deciderSettings_.maxStepForward = dv("decider.balanceStep.maxStepForward", deciderSettings_.maxStepForward);
        deciderSettings_.maxStepSide = dv("decider.balanceStep.maxStepSide", deciderSettings_.maxStepSide);
        deciderSettings_.reachDrop = dv("decider.balanceStep.reachDrop", deciderSettings_.reachDrop);
        deciderSettings_.weightShiftMost = dv("decider.balanceStep.weightShiftMost", deciderSettings_.weightShiftMost);
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

const std::vector<math::Mat4>* Human::limbTestPlan(float dt) {
    // The physics body now (character space): only where a hand's or foot's
    // path starts; the decider never reads it (its plan carries on).
    humanBody_.boneModels(*physics_, 1.0f, rig_, current_, root_, physicsModels_);
    // The commanded point on its way to the goal (GoalPath), from the end's
    // real position when a move starts. The path is kept in the world (the
    // root moves under it as the body does); returned in character space.
    auto travel = [&](GoalPath& path, math::Vec3 goalWorld, int end) {
        // A new goal (a press: it jumps) starts a new path from where the
        // point is, in Fitts' time for the new way; a goal moving on smoothly
        // retargets the path under way, or starts one once it has ended.
        constexpr float kJump = 0.02f, kMoved = 1e-3f;  // m in one tick
        const float moved = math::distance(goalWorld, path.to);
        if (!path.on || path.end != end || moved > kJump || (path.time >= path.duration && moved > kMoved)) {
            path.from = path.on && path.end == end ? path.at
                                                   : root_.transformPoint(math::Vec3(physicsModels_[static_cast<size_t>(end)][3]));
            path.end = end;
            path.time = 0.0f;
            path.duration = math::fittsTime(math::distance(path.from, goalWorld), muscleTiming_.fittsWidth,
                                         muscleTiming_.fittsA, muscleTiming_.fittsB);
            path.on = true;
        }
        path.to = goalWorld;
        path.time += dt;
        const float s = std::clamp(path.time / std::max(path.duration, 1e-3f), 0.0f, 1.0f);
        path.at = math::lerp(path.from, path.to, s * s * s * (10.0f + s * (6.0f * s - 15.0f)));  // minimum jerk
        return math::inverse(root_).transformPoint(path.at);
    };
    // A path's end (character space) and its time left, for the sync formula.
    auto pathOf = [&](const GoalPath& path, motion::Target& target) {
        target.pathEnd = math::inverse(root_).transformPoint(path.to);
        target.pathLeft = std::max(0.0f, path.duration - path.time);
    };
    // The planner's targets: every foot pinned where the stepper has it (on
    // the ground), the hips, chest and head free (the decider's own frames),
    // then the limb test's one goal or the step.
    motion::TargetSet& t = bodyTargets_;
    t = motion::TargetSet{};
    t.dt = dt;
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
        hand.position = travel(handPath_, posePosition_[g], left ? joints_.armL.end : joints_.armR.end);
        pathOf(handPath_, hand);
        turn(hand);
    } else {
        handPath_.on = false;
    }
    if (g == static_cast<int>(motion::Goal::Hips) && joints_.pelvis >= 0) {
        // The hips go there (every direction) and turn, within the pelvis's range.
        t.hips.mode = motion::TargetMode::Driven;
        t.hips.position = travel(hipsPath_, posePosition_[g], joints_.pelvis);
        pathOf(hipsPath_, t.hips);
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
    const bool poseFoot = g == static_cast<int>(motion::Goal::FootL) || g == static_cast<int>(motion::Goal::FootR);
    if (poseFoot) {
        const int f = g == static_cast<int>(motion::Goal::FootL) ? 0 : 1;
        const int ankle = f == 0 ? joints_.legL.end : joints_.legR.end;
        math::Vec3 goal = math::inverse(root_).transformPoint(posePosition_[g]);
        if (goal.y <= liftedAnkleMin_) {
            // On or under the ground: the foot is at once on the ground under
            // the target and stands there (pinned: it carries weight, the
            // hips go between the feet); it stays there after.
            // The decider steps there (a pinned foot never slides): in Fitts'
            // time for the way (the sync formula).
            const math::Vec3 spot(goal.x, liftedAnkleMin_ - toeClearance_, goal.z);
            const float way = math::distance(stepper_.foot(static_cast<motion::Foot>(f)), spot);
            t.foot[f].pathEnd = spot;
            t.foot[f].pathLeft =
                math::fittsTime(way, muscleTiming_.fittsWidth, muscleTiming_.fittsA, muscleTiming_.fittsB);
            stepper_.place(static_cast<motion::Foot>(f), spot);
            t.foot[f].mode = motion::TargetMode::Pinned;
            t.foot[f].position = spot;
            footPath_.on = false;
        } else {
            // Above the ground: the foot is held up there (no weight on it).
            t.foot[f].mode = motion::TargetMode::Driven;
            t.foot[f].position = travel(footPath_, root_.transformPoint(goal), ankle);
            pathOf(footPath_, t.foot[f]);
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
    }
    return queue_.plan;
}

void Human::groundLoads(const std::vector<math::Mat4>& target, float dt) {
    const physics::PhysicsWorld& world = *physics_;
    // Each foot as it really is: whether it (or its toes) touches the ground,
    // its ankle and level axes, and its functional base of support (forward
    // and side, m from the ankle) from the foot's and toes' shapes.
    struct Sole {
        int foot = -1, toe = -1;
        bool touching = false, toeTouching = false;
        math::Vec3 ankle{0.0f}, forward = math::kForward, side = math::kRight;
        float lo[2] = {0.0f, 0.0f}, hi[2] = {0.0f, 0.0f};
        float groundY = 0.0f, toeAt = 0.0f;
    } sole[2];
    auto extent = [&](int part, int axis, float& lo, float& hi) {
        const HumanBody::Part& p = humanBody_.parts()[static_cast<size_t>(part)];
        const float centre = axis == 0 ? p.shape.position.z : p.shape.position.x;
        const float half = p.shapeType == physics::ShapeType::Box ? (axis == 0 ? p.halfExtents.z : p.halfExtents.x) : p.radius;
        lo = centre - half;
        hi = centre + half;
    };
    constexpr int kCorners = 8;
    math::Vec3 corner[kCorners];
    for (int f = 0; f < 2; ++f) {
        Sole& s = sole[f];
        s.foot = humanBody_.partOf(f == 0 ? joints_.legL.end : joints_.legR.end);
        s.toe = humanBody_.partOf(f == 0 ? joints_.toeL : joints_.toeR);
        if (s.foot < 0) continue;
        const math::Transform ankle = humanBody_.boneWorld(world, s.foot);
        s.ankle = ankle.position;
        s.groundY = ankle.position.y;
        const math::Vec3 ahead = ankle.rotation * math::kForward;
        s.forward = math::safeNormalize(math::Vec3(ahead.x, 0.0f, ahead.z), math::kForward);
        s.side = math::cross(math::kUp, s.forward);
        if (s.toe >= 0) s.toeAt = math::dot(humanBody_.boneWorld(world, s.toe).position - s.ankle, s.forward);
        float ySum = 0.0f;
        int n = 0;
        for (const int part : {s.foot, s.toe}) {
            if (part < 0) continue;
            const int touches = world.groundContacts(humanBody_.parts()[static_cast<size_t>(part)].body, corner, kCorners);
            if (part == s.toe) s.toeTouching = touches > 0;
            for (int i = 0; i < touches; ++i) ySum += corner[i].y;
            n += touches;
        }
        s.touching = n > 0;
        if (n > 0) s.groundY = ySum / static_cast<float>(n);
        float heel = 0.0f, tip = 0.0f, right = 0.0f, left = 0.0f;
        extent(s.foot, 0, heel, tip);
        extent(s.foot, 1, right, left);
        if (s.toe >= 0) {
            float lo = 0.0f, hi = 0.0f;
            extent(s.toe, 0, lo, hi);
            tip = std::max(tip, s.toeAt + hi);
        }
        const float length = tip - heel, middle = 0.5f * (left + right), half = 0.5f * footSupport_.width * (left - right);
        s.lo[0] = heel + footSupport_.heelIn * length;
        s.hi[0] = std::max(s.lo[0], tip - footSupport_.toeIn * length);
        s.lo[1] = middle - half;
        s.hi[1] = middle + half;
    }
    // The plan's statics (character space, level).
    const math::Vec3 planCom = humanBody_.centreOfMass(target);
    const math::Vec3 planAnkle[2] = {modelPos(target, joints_.legL.end), modelPos(target, joints_.legR.end)};
    float t = 0.5f;
    {
        math::Vec3 line = planAnkle[1] - planAnkle[0], off = planCom - planAnkle[0];
        line.y = off.y = 0.0f;
        const float len2 = math::dot(line, line);
        if (len2 > 1e-6f) t = std::clamp(math::dot(off, line) / len2, 0.0f, 1.0f);
    }
    const float s0 = std::clamp(queue_.supportWeight[0], 0.0f, 1.0f), s1 = std::clamp(queue_.supportWeight[1], 0.0f, 1.0f);
    const float plan[2] = {s0 * (1.0f - s1) + s0 * s1 * (1.0f - t), s1 * (1.0f - s0) + s0 * s1 * t};
    const float on[2] = {sole[0].touching ? plan[0] : 0.0f, sole[1].touching ? plan[1] : 0.0f};
    const float onSum = on[0] + on[1];
    const float step = dt / std::max(deciderSettings_.supportBlendTime, 1e-3f);
    for (int f = 0; f < 2; ++f) {
        const float want = onSum > 1e-4f ? on[f] / onSum : 0.0f;
        groundShare_[f] += std::clamp(want - groundShare_[f], -step, step);
    }
    const float sum = groundShare_[0] + groundShare_[1];
    math::Vec3 off(0.0f);
    if (sum > 1e-3f) {
        off = planCom - (groundShare_[0] * planAnkle[0] + groundShare_[1] * planAnkle[1]) / sum;
        off.y = 0.0f;
        off = root_.rotation * off;
    }
    for (int f = 0; f < 2; ++f) {
        const Sole& s = sole[f];
        loads_[f] = HumanBody::GroundLoad{};
        if (s.foot < 0 || groundShare_[f] <= 0.0f) continue;
        const float u = std::clamp(math::dot(off, s.forward), s.lo[0], s.hi[0]);
        const float v = std::clamp(math::dot(off, s.side), s.lo[1], s.hi[1]);
        math::Vec3 at = s.ankle + s.forward * u + s.side * v;
        at.y = s.groundY;
        loads_[f].part = s.toeTouching && u > s.toeAt ? s.toe : s.foot;
        loads_[f].share = groundShare_[f];
        loads_[f].at = at;
    }
}

void Human::physicsTick(const std::vector<math::Mat4>& target, float dt) {
    physics::PhysicsWorld& world = *physics_;
    const physics::CharacterSettings& c = world.settings().character;
    // The muscles: they carry the weight beyond each joint, and Jolt's pose
    // drive corrects toward this tick's pose.
    groundLoads(target, dt);
    // Free arms (no hand goal) are left to physics: relaxed muscles toward
    // hanging, gravity and the body's motion swing them (the muscle plan B4).
    for (int h = 0; h < 2; ++h) {
        const bool free = poseLast_ != static_cast<int>(h == 0 ? motion::Goal::HandL : motion::Goal::HandR);
        const ik::TwoBoneChain& arm = h == 0 ? joints_.armL : joints_.armR;
        for (int j : {h == 0 ? joints_.clavicleL : joints_.clavicleR, arm.start, arm.mid, arm.end})
            if (j >= 0 && humanBody_.partOf(j) >= 0 && rig_.boneDefIndex(j) >= 0 && humanBody_.parts()[static_cast<size_t>(humanBody_.partOf(j))].joint == j)
                humanBody_.setFree(humanBody_.partOf(j), free);
    }
    humanBody_.holdWeight(world, loads_, 2);
    humanBody_.drivePose(world, target, root_, joints_.head, dt);
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
        planned.position.y -= std::max(0.0f, pelvisY - restPelvisHeight_);
        const math::Transform pelvis = root_ * planned;  // world
        const math::Quat turn = root_.rotation;
        world.moveKinematic(anchor_, math::Transform{pelvis.position, turn}, dt);
        world.setSupportTarget(support_, math::Vec3(0.0f), math::inverse(turn) * pelvis.rotation);
        // The hold's strength: eased back in after a recovery, and the
        // support under the feet.
        if (supportShare_ < 1.0f) supportShare_ = std::min(1.0f, supportShare_ + dt / std::max(c.recoverTime, 1e-3f));
        const float onFeet = std::min(1.0f, queue_.supportWeight[0] + queue_.supportWeight[1]);
        const float k = supportShare_ * onFeet;
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
    supportShare_ = 0.0f;
    world.setSupportLimits(support_, 0.0f, 0.0f);
    world.setJointEnabled(support_, true);
    supportOn_ = true;
    armed_ = false;
    restTime_ = 0.0f;
    LOG_INFO("human: at rest, the capsule takes the body again");
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
    supportOn_ = armed_ = true;
    supportShare_ = 1.0f;
    restTime_ = 0.0f;
    pelvisError_ = PelvisError::Small;
    pelvisErrorNow_ = pelvisErrorTime_ = 0.0f;
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
    
