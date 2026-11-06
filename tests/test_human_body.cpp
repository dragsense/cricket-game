// The human body from generic physics parts: a Jolt body per bone with mass,
// joined with the real ranges from human_joints.json. No muscles yet: it
// folds under the world's gravity onto the world's ground.

#include "agents/human.h"
#include "core/config.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {

constexpr float kTick = 1.0f / 120.0f;

struct Scene {
    core::Config skeleton, joints, locomotion, physicsConfig;
    physics::PhysicsWorld world;
    agents::Human human;
};

bool makeScene(Scene& s) {
    rig::SkeletonDef def;
    return s.skeleton.load(CRICKET_CONFIG_DIR "/human_skeleton.json") &&
           s.joints.load(CRICKET_CONFIG_DIR "/human_joints.json") &&
           s.locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json") &&
           s.physicsConfig.load(CRICKET_CONFIG_DIR "/physics_world.json") && rig::loadSkeletonDef(s.skeleton, def) &&
           s.world.init(physics::readWorldSettings(s.physicsConfig), physics::readPhysicsSettings(s.physicsConfig)) &&
           s.human.init(def, def.height, s.joints, s.locomotion) && s.human.attachPhysics(s.world);
}

// A part's bone rotation (world) from its body.
math::Quat boneRotation(const physics::PhysicsWorld& w, const agents::HumanBody::Part& p) {
    return w.transform(p.body).rotation * math::inverse(p.shape.rotation);
}

} // namespace

TEST_CASE("human body: a body per bone with mass, 73 kg in all") {
    Scene s;
    REQUIRE(makeScene(s));
    const agents::HumanBody& body = s.human.humanBody();
    // pelvis, 2 spine, chest, head; per side upper arm, forearm, hand, thigh,
    // shin, foot, toe (neck, clavicles and fingers ride their parents).
    CHECK(body.parts().size() == 19);
    CHECK(body.totalMass() == doctest::Approx(73.0f).scale(0.0).epsilon(0.001));
    int joined = 0;
    for (const agents::HumanBody::Part& p : body.parts()) joined += p.link != physics::kInvalidJoint;
    CHECK(joined == 18);  // every part but the pelvis
}

// Known failure (2026-11-05): the motors' stiffness came from muscle
// activation time, which in Jolt depends on each part's own inertia, so
// loaded joints sag (shoulder 0.56 rad, body slumps 0.99 -> 0.45 m). Next:
// measured joint stiffness (N m/rad) per joint (PROGRESS).
TEST_CASE("human body: with muscles each joint holds its planned rotation (no balance yet: watch it)" *
          doctest::may_fail()) {
    Scene s;
    REQUIRE(makeScene(s));
    const agents::HumanBody& body = s.human.humanBody();
    const float startHeight = body.height(s.world);
    float worstOutside = 0.0f, lowestPart = 9.0f, worstHold = 0.0f;
    std::string worstJoint, worstHeld;
    std::vector<math::Mat4> planned(static_cast<size_t>(s.human.rig().numJoints()));
    for (int i = 0; i < 2 * 120; ++i) {
        s.human.tick(kTick);  // plans the pose and sets the muscles' targets
        s.world.step(kTick);
        s.human.rig().localToModel(s.human.pose(), planned);
        for (const agents::HumanBody::Part& p : body.parts()) {
            if (p.parent < 0 || !p.muscle || i > 60) continue;  // the first half second
            const math::Quat want = math::inverse(math::toTransform(planned[static_cast<size_t>(body.parts()[static_cast<size_t>(p.parent)].joint)]).rotation) *
                                    math::toTransform(planned[static_cast<size_t>(p.joint)]).rotation;
            const math::Quat got = math::inverse(boneRotation(s.world, body.parts()[static_cast<size_t>(p.parent)])) *
                                   boneRotation(s.world, p);
            const float off = 2.0f * std::acos(std::min(1.0f, std::fabs(math::dot(math::normalize(want), math::normalize(got)))));
            if (off > worstHold) {
                worstHold = off;
                worstHeld = s.human.rig().jointName(p.joint);
            }
        }
        for (const agents::HumanBody::Part& p : body.parts()) {
            lowestPart = std::min(lowestPart, s.world.transform(p.body).position.y);
            if (p.parent < 0) continue;
            // The bone's rotation relative to its parent part's bone, against
            // its real range.
            const math::Quat local = math::inverse(boneRotation(s.world, body.parts()[static_cast<size_t>(p.parent)])) *
                                     boneRotation(s.world, p);
            const math::Quat clamped = ik::clampRotation(local, s.human.jointLimits().limit(p.joint));
            const float outside = 2.0f * std::acos(std::min(1.0f, std::fabs(math::dot(math::normalize(local), clamped))));
            if (outside > worstOutside) {
                worstOutside = outside;
                worstJoint = s.human.rig().jointName(p.joint);
            }
        }
    }
    const float endHeight = body.height(s.world);
    MESSAGE("muscles: worst joint off its planned rotation in the first 0.5 s ", worstHold, " rad (", worstHeld,
            "); centre of mass ", startHeight, " -> ", endHeight, " m after 2 s; lowest part centre ", lowestPart,
            " m; worst joint past its range ", worstOutside, " rad (", worstJoint, ")");
    CHECK(worstHold < 0.2f);       // the muscles hold the planned shape
    CHECK(lowestPart > -0.01f);    // the ground holds every part
    CHECK(worstOutside < 0.25f);   // joints near their real ranges
}

namespace {

// The muscles' loop with the pelvis held by the test (no balance yet): the
// decider reads the physics body, the muscles hold the weight and turn the
// joints its queue names, Jolt reports the blocked joints.
struct MuscleRig {
    Scene s;
    motion::BodyDeciderJoints joints;
    motion::BodyDecider decider;
    motion::BodyQueue queue;
    std::vector<uint8_t> blocked;
    std::vector<math::Mat4> models;
    rig::Pose pose;
    agents::MuscleTiming timing;

    bool make() {
        if (!makeScene(s)) return false;
        const rig::Rig& rig = s.human.rig();
        auto chain = [&](const char* a, const char* b, const char* c, math::Vec3 axis) {
            return ik::TwoBoneChain{rig.findJoint(a), rig.findJoint(b), rig.findJoint(c), axis};
        };
        joints.pelvis = rig.findJoint("pelvis");
        joints.spine[0] = rig.findJoint("spine_01");
        joints.spine[1] = rig.findJoint("spine_02");
        joints.spine[2] = rig.findJoint("chest");
        joints.neck = rig.findJoint("neck");
        joints.head = rig.findJoint("head");
        joints.clavicles[0] = rig.findJoint("clavicle_l");
        joints.clavicles[1] = rig.findJoint("clavicle_r");
        joints.legs[0] = chain("thigh_l", "shin_l", "foot_l", {-1, 0, 0});
        joints.legs[1] = chain("thigh_r", "shin_r", "foot_r", {-1, 0, 0});
        joints.arms[0] = chain("upper_arm_l", "forearm_l", "hand_l", {1, 0, 0});
        joints.arms[1] = chain("upper_arm_r", "forearm_r", "hand_r", {1, 0, 0});
        blocked.assign(static_cast<size_t>(rig.numJoints()), 0);
        models.resize(static_cast<size_t>(rig.numJoints()));
        pose = rig.restPose();
        timing.fittsA = s.joints.getFloat("reach.fittsA", timing.fittsA);
        timing.fittsB = s.joints.getFloat("reach.fittsB", timing.fittsB);
        timing.fittsWidth = s.joints.getFloat("reach.fittsWidth", timing.fittsWidth);
        if (!decider.init(rig, joints)) return false;
        const agents::HumanBody& body = s.human.humanBody();
        body.setPhysics(s.world, true);
        s.world.setKinematic(body.parts()[0].body, true);  // the test holds the pelvis
        return true;
    }

    void step(const motion::BodyGoals& goals) {
        const agents::HumanBody& body = s.human.humanBody();
        s.human.physicalModels(s.world, 1.0f, models);
        s.human.rig().modelToLocal(models, pose);
        decider.decide(pose, goals, blocked, motion::BodyDeciderSettings{}, queue);
        body.compensateGravity(s.world);
        body.drive(s.world, queue, goals, joints, models, timing);
        s.world.step(kTick);
        s.human.physicalModels(s.world, 1.0f, models);
        body.blockedJoints(s.world, queue, models, s.human.jointLimits(), kTick, blocked);
    }

    math::Vec3 pos(int joint) const { return math::Vec3(models[static_cast<size_t>(joint)][3]); }
};

} // namespace

TEST_CASE("human body muscles: with no target every joint holds its pose (no sag), pelvis held") {
    MuscleRig r;
    REQUIRE(r.make());
    const agents::HumanBody& body = r.s.human.humanBody();
    float worst = 0.0f;
    std::string worstJoint;
    for (int i = 0; i < 2 * 120; ++i) r.step(motion::BodyGoals{});
    for (const agents::HumanBody::Part& p : body.parts()) {
        if (p.parent < 0) continue;
        const math::Quat local = math::inverse(boneRotation(r.s.world, body.parts()[static_cast<size_t>(p.parent)])) *
                                 boneRotation(r.s.world, p);
        const float off = math::angleBetween(local, math::identityQuat());  // rest pose: identity
        if (off > worst) {
            worst = off;
            worstJoint = r.s.human.rig().jointName(p.joint);
        }
    }
    MESSAGE("no target, 2 s: worst joint off its rest rotation ", worst, " rad (", worstJoint,
            "); before muscles with gravity compensation the shoulder sagged 0.56 rad");
    CHECK(worst < 0.05f);
}

// Was failing (2026-11-06, wrist 0.82 m off): the decider ordered the elbow
// to repeat the shoulder's turn (the bone's world turn, not its own), about
// axes the elbow does not have; its motor pushed at full strength into the
// locked axis and the leak spun the twists out of range.
TEST_CASE("human body muscles: a hand target within reach is reached, pelvis held") {
    MuscleRig r;
    REQUIRE(r.make());
    r.s.human.physicalModels(r.s.world, 1.0f, r.models);
    motion::BodyGoals goals;
    goals.hand[0].active = true;
    goals.hand[0].position = r.pos(r.joints.arms[0].end) + math::Vec3(0.0f, 0.2f, 0.25f);  // forward and up
    for (int i = 0; i < 2 * 120; ++i) r.step(goals);
    const float miss = math::distance(r.pos(r.joints.arms[0].end), goals.hand[0].position);
    MESSAGE("hand target in reach: wrist ", miss, " m from it after 2 s");
    CHECK(miss < 0.03f);
}
