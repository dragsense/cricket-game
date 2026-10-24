#include "agents/fielder.h"

#include "core/config.h"

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

bool Fielder::init(const rig::SkeletonDef& def, float height, const core::Config& joints) {
    def_ = def;
    jointsConfig_ = &joints;
    root_.position = def.spawnPosition;
    root_.rotation = math::fromAxisAngle(math::kUp, def.spawnYaw);

    ik_.kneePole = joints.getVec3("ik.kneePole", ik_.kneePole);
    ik_.elbowPole = joints.getVec3("ik.elbowPole", ik_.elbowPole);
    ik_.soften = joints.getFloat("ik.soften", ik_.soften);
    ik_.chestAimWeight = joints.getFloat("ik.chestAimWeight", ik_.chestAimWeight);
    ik_.headAimWeight = joints.getFloat("ik.headAimWeight", ik_.headAimWeight);
    demo_.period = joints.getFloat("ikDemo.period", demo_.period);
    demo_.handReach = joints.getFloat("ikDemo.handReach", demo_.handReach);
    demo_.footLift = joints.getFloat("ikDemo.footLift", demo_.footLift);
    demo_.footSwing = joints.getFloat("ikDemo.footSwing", demo_.footSwing);
    demo_.pelvisDrop = joints.getFloat("ikDemo.pelvisDrop", demo_.pelvisDrop);
    demo_.lookRadius = joints.getFloat("ikDemo.lookRadius", demo_.lookRadius);
    return setHeight(height);
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
    return true;
}

void Fielder::tick(double dt) {
    previous_.locals = current_.locals;  // same size: no allocation
    current_.locals = rig_.restPose().locals;
    if (!ikDemo_) return;  // neutral standing pose

    demoTime_ += dt;
    updateDemoTargets();
    solveIk();
}

void Fielder::updateDemoTargets() {
    // Rest positions in model space.
    rig_.localToModel(rig_.restPose(), models_);
    const float ph = static_cast<float>(math::kTwoPi * demoTime_ / demo_.period);
    auto world = [&](math::Vec3 p) { return root_.transformPoint(p); };

    // Pelvis: from slightly above full leg length (legs must not overstretch)
    // down into a deep crouch.
    float crouch = 0.5f - 0.5f * std::cos(ph);
    targets_.pelvisOffset = math::Vec3(0.0f, demo_.pelvisDrop * (0.25f - 1.25f * crouch), 0.0f);

    math::Vec3 footL = modelPos(models_, joints_.legL.end);
    math::Vec3 footR = modelPos(models_, joints_.legR.end);
    targets_.footL = world(footL + math::Vec3(0.0f, demo_.footLift * std::max(0.0f, std::sin(ph)),
                                              demo_.footSwing * std::cos(ph)));
    targets_.footR = world(footR + math::Vec3(0.0f, demo_.footLift * std::max(0.0f, -std::sin(ph)),
                                              -demo_.footSwing * std::cos(ph)));

    // Hands sweep spheres around each shoulder, reaching past arm length.
    auto handPath = [&](int shoulder, float side, float phase) {
        math::Vec3 d(side * 0.8f * std::sin(phase), -std::cos(phase), std::sin(2.0f * phase));
        return world(modelPos(models_, shoulder) + math::safeNormalize(d, -math::kUp) * demo_.handReach);
    };
    targets_.handL = handPath(joints_.armL.start, 1.0f, ph);
    targets_.handR = handPath(joints_.armR.start, -1.0f, ph + math::kHalfPi);

    // Look target circles all the way round (behind too, to test neck limits).
    math::Vec3 head = modelPos(models_, joints_.head);
    targets_.lookAt = world(head + demo_.lookRadius * math::Vec3(std::sin(0.5f * ph), 0.3f * std::sin(ph),
                                                                 std::cos(0.5f * ph)));
}

void Fielder::solveIk() {
    const math::Transform toModel = math::inverse(root_);
    auto model = [&](math::Vec3 w) { return toModel.transformPoint(w); };
    const math::Vec3 up = math::kUp;

    current_.locals[static_cast<size_t>(joints_.pelvis)].position += targets_.pelvisOffset;

    // Chest takes part of the look rotation, then the head finishes it.
    rig_.localToModel(current_, models_);
    ik::solveAim(joints_.chest, models_, model(targets_.lookAt), math::kForward, up, up, ik_.chestAimWeight, current_);
    limits_.apply(current_);
    rig_.localToModel(current_, models_);
    ik::solveAim(joints_.head, models_, model(targets_.lookAt), math::kForward, up, up, ik_.headAimWeight, current_);

    ik::solveTwoBone(joints_.legL, models_, model(targets_.footL), ik_.kneePole, ik_.soften, current_);
    ik::solveTwoBone(joints_.legR, models_, model(targets_.footR), ik_.kneePole, ik_.soften, current_);
    math::Vec3 elbowPoleR(-ik_.elbowPole.x, ik_.elbowPole.y, ik_.elbowPole.z);
    ik::solveTwoBone(joints_.armL, models_, model(targets_.handL), ik_.elbowPole, ik_.soften, current_);
    ik::solveTwoBone(joints_.armR, models_, model(targets_.handR), elbowPoleR, ik_.soften, current_);
    limits_.apply(current_);

    // Feet stay flat (rest orientation in character space) whatever the shin does.
    rig_.localToModel(current_, models_);
    for (const ik::TwoBoneChain* leg : {&joints_.legL, &joints_.legR}) {
        math::Quat shin = modelRot(models_, leg->mid);
        current_.locals[static_cast<size_t>(leg->end)].rotation = math::normalize(math::inverse(shin));
    }
    limits_.apply(current_);
}

void Fielder::interpolatedPose(float alpha, rig::Pose& out) const {
    out.locals.resize(current_.locals.size());
    for (size_t i = 0; i < current_.locals.size(); ++i)
        out.locals[i] = math::lerp(previous_.locals[i], current_.locals[i], alpha);
}

} // namespace agents
