#include "motion/body_reposition.h"

#include "motion/reach.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

math::Vec3 jointPos(const std::vector<math::Mat4>& models, int joint) {
    return math::Vec3(models[static_cast<size_t>(joint)][3]);
}

math::Quat jointRot(const std::vector<math::Mat4>& models, int joint) {
    return math::toTransform(models[static_cast<size_t>(joint)]).rotation;
}

} // namespace

bool BodyReposition::init(const rig::Rig& rig, const ik::JointLimits& limits, const BodyRepositionJoints& joints) {
    rig_ = &rig;
    limits_ = &limits;
    joints_ = joints;
    if (joints.pelvis < 0) return false;
    std::vector<math::Mat4> models(static_cast<size_t>(rig.numJoints()));
    if (!rig.localToModel(rig.restPose(), models)) return false;
    const math::Vec3 pelvis = jointPos(models, joints.pelvis);
    restPelvis_ = pelvis;
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& c = joints.legs[i];
        const math::Vec3 hip = jointPos(models, c.start), knee = jointPos(models, c.mid), ankle = jointPos(models, c.end);
        hipOffset_[i] = hip - pelvis;
        legDir_[i] = math::normalize(ankle - hip);
        legLength_[i] = math::distance(hip, knee) + math::distance(knee, ankle);
        const ik::TwoBoneChain& a = joints.arms[i];
        if (a.start >= 0)
            armLength_[i] = math::distance(jointPos(models, a.start), jointPos(models, a.mid)) +
                            math::distance(jointPos(models, a.mid), jointPos(models, a.end));
    }
    return true;
}

void BodyReposition::rotateInModel(rig::Pose& pose, const std::vector<math::Mat4>& models, int joint,
                                   math::Quat rotation) const {
    // Model rotation M = P * L, so a model-space rotation R becomes the local
    // L' = P^-1 * R * P * L.
    const math::Quat parent = jointRot(models, rig_->parent(joint));
    math::Quat& local = pose.locals[static_cast<size_t>(joint)].rotation;
    local = ik::clampRotation(math::normalize(math::inverse(parent) * rotation * parent * local), limits_->limit(joint));
}

void BodyReposition::solve(rig::Pose& pose, std::vector<math::Mat4>& models, const BodyTargets& t,
                           const BodyRepositionSettings& s) const {
    const int p = t.planted, o = 1 - t.planted;
    const math::Vec3 ankles[2] = {p == 0 ? t.plantedAnkle : t.movingAnkle, p == 1 ? t.plantedAnkle : t.movingAnkle};

    // The body stands at its spot over the root; the legs reach the feet.
    math::Vec3 pelvisPos = restPelvis_;
    math::Quat pelvisRot = math::identityQuat();

    // A leg cannot stretch: its hip stays within its length of its foot.
    // Only a foot on the ground holds the body; one in the air just reaches.
    auto keepLeg = [&](int i) {
        if (!t.down[i]) return;
        const math::Vec3 d = pelvisPos + pelvisRot * hipOffset_[i] - ankles[i];
        const float len = math::length(d), most = s.legReach * legLength_[i];
        if (len > most) pelvisPos -= d * ((len - most) / len);
    };
    keepLeg(p);

    // --- Feet ---
    for (int pass = 0; pass < s.passes; ++pass) {
        // Past the hip's range: the pelvis tilts by the rest, over the standing hip.
        const math::Vec3 standingHip = pelvisPos + pelvisRot * hipOffset_[p];
        const math::Vec3 movingHip = pelvisPos + pelvisRot * hipOffset_[o];
        const math::Vec3 want = math::safeNormalize(t.movingAnkle - movingHip, pelvisRot * legDir_[o]);
        const math::Quat swing = math::fromTo(legDir_[o], math::inverse(pelvisRot) * want);
        const math::Quat held = ik::clampRotation(swing, limits_->limit(joints_.legs[o].start));
        const math::Quat rest = math::fromTo(pelvisRot * (held * legDir_[o]), want);
        pelvisRot = clampRotationVector(math::normalize(rest * pelvisRot), s.pelvisMin, s.pelvisMax);
        pelvisPos = standingHip - pelvisRot * hipOffset_[p];

        // Past the leg's length: the pelvis moves toward the foot (the standing
        // leg keeps its foot, so the body lowers).
        const math::Vec3 to = t.movingAnkle - (pelvisPos + pelvisRot * hipOffset_[o]);
        const float len = math::length(to), most = s.legReach * legLength_[o];
        if (len > most) pelvisPos += to * ((len - most) / len);
        keepLeg(p);
    }
    math::Transform& pelvis = pose.locals[static_cast<size_t>(joints_.pelvis)];
    pelvis.position = pelvisPos;
    pelvis.rotation = pelvisRot;

    const math::Vec3 hipCentre = 0.5f * (hipOffset_[0] + hipOffset_[1]);

    // --- Chest: the spine turns it first, the pelvis takes the rest ---
    if (t.chest && joints_.spine[2] >= 0) {
        const int chest = joints_.spine[2];
        const int spine[3] = {joints_.spine[2], joints_.spine[1], joints_.spine[0]};
        for (int j : spine) {
            if (j < 0) continue;
            rig_->localToModel(pose, models);
            rotateInModel(pose, models, j, math::normalize(t.chestRot * math::inverse(jointRot(models, chest))));
        }
        rig_->localToModel(pose, models);
        const math::Quat rest = math::normalize(t.chestRot * math::inverse(jointRot(models, chest)));
        const math::Vec3 hips = pelvisPos + pelvisRot * hipCentre;
        pelvisRot = clampRotationVector(math::normalize(rest * pelvisRot), s.pelvisMin, s.pelvisMax);
        pelvisPos = hips - pelvisRot * hipCentre;
        pelvis.position = pelvisPos;
        pelvis.rotation = pelvisRot;
    }

    // --- Hands: each part only takes what the parts before it left ---
    for (int pass = 0; pass < s.passes; ++pass) {
        for (int h = 0; h < 2; ++h) {
            if (!t.hand[h] || joints_.arms[h].start < 0) continue;
            const math::Vec3 goal = t.handGoal[h];
            const float reach = s.armReach * armLength_[h];
            const int shoulder = joints_.arms[h].start;

            // Clavicle, then chest, spine_02, spine_01 swing the shoulder toward the hand.
            const int trunk[4] = {joints_.clavicles[h], joints_.spine[2], joints_.spine[1], joints_.spine[0]};
            for (int j : trunk) {
                if (j < 0) continue;
                rig_->localToModel(pose, models);
                const Swing sw = swingToward(jointPos(models, j), jointPos(models, shoulder), goal, reach);
                if (sw.angle > 0.0f) rotateInModel(pose, models, j, math::fromAxisAngle(sw.axis, sw.angle));
            }

            // Then the pelvis tilts at the hips.
            rig_->localToModel(pose, models);
            const math::Vec3 hips = pelvisPos + pelvisRot * hipCentre;
            const Swing sw = swingToward(hips, jointPos(models, shoulder), goal, reach);
            if (sw.angle > 0.0f) {
                pelvisRot = clampRotationVector(math::normalize(math::fromAxisAngle(sw.axis, sw.angle) * pelvisRot),
                                                s.pelvisMin, s.pelvisMax);
                pelvisPos = hips - pelvisRot * hipCentre;
            }

            // Then the body moves toward the hand; the feet stay, so the legs bend.
            pelvis.position = pelvisPos;
            pelvis.rotation = pelvisRot;
            rig_->localToModel(pose, models);
            const math::Vec3 to = goal - jointPos(models, shoulder);
            const float len = math::length(to);
            if (len > reach) pelvisPos += to * ((len - reach) / len);
            keepLeg(0);
            keepLeg(1);
            pelvis.position = pelvisPos;
        }
    }

    // Legs: each ankle to its target, knees forward, then the joint limits.
    rig_->localToModel(pose, models);
    const math::Vec3 knee = pelvisRot * s.kneePole;
    for (int i = 0; i < 2; ++i) ik::solveTwoBone(joints_.legs[i], models, ankles[i], knee, 1.0f, pose);
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& c = joints_.legs[i];
        for (int j : {c.start, c.mid})
            pose.locals[static_cast<size_t>(j)].rotation =
                ik::clampRotation(pose.locals[static_cast<size_t>(j)].rotation, limits_->limit(j));
    }

    // Feet flat (as far as the ankle allows), toes straight.
    rig_->localToModel(pose, models);
    for (int i = 0; i < 2; ++i) {
        const ik::TwoBoneChain& c = joints_.legs[i];
        pose.locals[static_cast<size_t>(c.end)].rotation =
            ik::clampRotation(math::normalize(math::inverse(jointRot(models, c.mid))), limits_->limit(c.end));
        if (joints_.toes[i] >= 0) pose.locals[static_cast<size_t>(joints_.toes[i])].rotation = math::identityQuat();
    }

    // Arms: each wrist to its target, elbow back and out (made square to the
    // shoulder-to-hand line), then the joint limits.
    for (int h = 0; h < 2; ++h) {
        if (!t.hand[h] || joints_.arms[h].start < 0) continue;
        const ik::TwoBoneChain& arm = joints_.arms[h];
        rig_->localToModel(pose, models);
        const math::Vec3 shoulder = jointPos(models, arm.start);
        const math::Vec3 dir = math::safeNormalize(t.handGoal[h] - shoulder, -math::kUp);
        const math::Vec3 side(h == 0 ? s.elbowPole.x : -s.elbowPole.x, s.elbowPole.y, s.elbowPole.z);
        const math::Vec3 pole = math::safeNormalize(side - dir * math::dot(side, dir), side);
        ik::solveTwoBone(arm, models, clampToReach(shoulder, t.handGoal[h], s.armReach * armLength_[h]), pole, 1.0f, pose);
        for (int j : {arm.start, arm.mid})
            pose.locals[static_cast<size_t>(j)].rotation =
                ik::clampRotation(pose.locals[static_cast<size_t>(j)].rotation, limits_->limit(j));
    }
}

} // namespace motion
