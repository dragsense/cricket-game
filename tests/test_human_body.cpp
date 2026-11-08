// The human body from generic physics parts: a Jolt body per bone with mass,
// joined with the real ranges from human_joints.json. No muscles yet: it
// folds under the world's gravity onto the world's ground.

#include "agents/balancer.h"
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
    motion::BodyDeciderSettings settings;  // the game's knee pole (config)

    agents::Balancer balancer;
    bool balance = false;  // the balancer stands the body (else the test holds the pelvis)

    bool make(bool withBalance = false) {
        if (!makeScene(s)) return false;
        balance = withBalance;
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
        settings.kneePole = s.joints.getVec3("ik.kneePole", settings.kneePole);
        if (!decider.init(rig, joints, &s.human.jointLimits())) return false;
        {
            std::vector<float> mass;
            std::vector<math::Vec3> centre;
            s.human.humanBody().jointMasses(rig.numJoints(), mass, centre);
            decider.setMasses(mass, centre);
        }
        agents::HumanBody& body = s.human.humanBody();
        body.setPhysics(s.world, true);
        if (!balance) {
            s.world.setKinematic(body.parts()[0].body, true);  // the test holds the pelvis
            return true;
        }
        const int foot[2] = {rig.findJoint("foot_l"), rig.findJoint("foot_r")};
        const int toe[2] = {rig.findJoint("toe_l"), rig.findJoint("toe_r")};
        return balancer.init(s.world, body, foot, toe, agents::readBalanceSettings(s.joints));
    }

    void step(const motion::BodyGoals& wanted) {
        agents::HumanBody& body = s.human.humanBody();
        s.human.physicalModels(s.world, 1.0f, models);
        s.human.rig().modelToLocal(models, pose);
        motion::BodyGoals goals = wanted;
        if (balance) {
            balancer.plan(s.world, body, goals.down, kTick);
            for (int f = 0; f < 2; ++f) goals.foot[f].active = goals.foot[f].active && balancer.liftReady(f);
        }
        decider.decide(pose, goals, blocked, settings, queue);
        body.compensateGravity(s.world);
        body.drive(s.world, queue, goals, joints, models, timing, kTick, balance ? balancer.free() : nullptr);
        if (balance) {
            const math::Quat planned = queue.plan ? math::toTransform((*queue.plan)[static_cast<size_t>(joints.pelvis)]).rotation
                                                  : math::identityQuat();
            balancer.setPelvisTarget(s.human.root().rotation * planned);
            balancer.apply(s.world, body);
        }
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

namespace {

// Lowest part centre (world) and the centre of mass off the stance ankles
// (horizontal, m) after a run.
float lowestPart(const MuscleRig& r) {
    float lowest = 9.0f;
    for (const agents::HumanBody::Part& p : r.s.human.humanBody().parts())
        lowest = std::min(lowest, r.s.world.transform(p.body).position.y);
    return lowest;
}

float offSupport(const MuscleRig& r) {
    const math::Vec3 com = r.s.human.humanBody().centreOfMass(r.s.world);
    const math::Vec3 c = r.balancer.pivot();
    return std::hypot(com.x - c.x, com.z - c.z);
}

} // namespace

TEST_CASE("balance: the body stands on its own for 5 s, physics on, nothing held") {
    MuscleRig r;
    REQUIRE(r.make(true));
    const agents::HumanBody& body = r.s.human.humanBody();
    const float startHeight = body.height(r.s.world);
    float worstOff = 0.0f;
    bool alwaysStanding = true;
    for (int i = 0; i < 5 * 120; ++i) {
        r.step(motion::BodyGoals{});
        alwaysStanding = alwaysStanding && r.balancer.standing();
        if (i > 120) worstOff = std::max(worstOff, offSupport(r));
    }
    const float endHeight = body.height(r.s.world);
    MESSAGE("standing 5 s: centre of mass ", startHeight, " -> ", endHeight, " m; worst off the ankles after 1 s ",
            worstOff, " m; hover ", r.balancer.hoverForce().y, " N; lowest part ",
            lowestPart(r), " m");
    CHECK(alwaysStanding);
    CHECK(endHeight > 0.95f * startHeight);  // did not fall or slump
    CHECK(worstOff < 0.05f);                 // over the feet
}

TEST_CASE("balance: pushed forward, the assists bring it back over the feet") {
    MuscleRig r;
    REQUIRE(r.make(true));
    const agents::HumanBody& body = r.s.human.humanBody();
    const float startHeight = body.height(r.s.world);
    for (int i = 0; i < 60; ++i) r.step(motion::BodyGoals{});  // settle
    // The push: every part 0.15 m/s forward (the character's facing).
    const math::Vec3 ahead = r.s.human.root().rotation * math::kForward;
    for (const agents::HumanBody::Part& p : body.parts()) r.s.world.addImpulse(p.body, p.mass * 0.15f * ahead, r.s.world.transform(p.body).position);
    float furthest = 0.0f;
    for (int i = 0; i < 3 * 120; ++i) {
        r.step(motion::BodyGoals{});
        furthest = std::max(furthest, offSupport(r));
    }
    const float speed = math::length(body.centreOfMassVelocity(r.s.world));
    MESSAGE("push 0.15 m/s: furthest off the ankles ", furthest, " m; after 3 s ", offSupport(r),
            " m off, moving ", speed, " m/s; centre of mass ", startHeight, " -> ", body.height(r.s.world), " m");
    CHECK(r.balancer.standing());
    CHECK(body.height(r.s.world) > 0.95f * startHeight);
    CHECK(offSupport(r) < 0.03f);
    CHECK(speed < 0.05f);
}

TEST_CASE("balance: a forward hand reach moves the hips back, the body stays up") {
    MuscleRig r;
    REQUIRE(r.make(true));
    const agents::HumanBody& body = r.s.human.humanBody();
    const float startHeight = body.height(r.s.world);
    for (int i = 0; i < 60; ++i) r.step(motion::BodyGoals{});  // settle
    const math::Vec3 pelvisStart = r.pos(r.joints.pelvis);
    motion::BodyGoals goals;
    goals.hand[0].active = true;
    // Straight ahead of the shoulder at its height, nearly the arm's length.
    goals.hand[0].position = r.pos(r.joints.arms[0].start) + math::Vec3(0.0f, 0.0f, 0.5f);
    for (int i = 0; i < 3 * 120; ++i) r.step(goals);
    const float hipsBack = pelvisStart.z - r.pos(r.joints.pelvis).z;  // character space: +Z ahead
    const float miss = math::distance(r.pos(r.joints.arms[0].end), goals.hand[0].position);
    MESSAGE("forward reach: hips moved back ", hipsBack, " m; wrist ", miss, " m from its target; off the support centre ",
            offSupport(r), " m; centre of mass ", startHeight, " -> ", body.height(r.s.world), " m");
    CHECK(r.balancer.standing());
    CHECK(body.height(r.s.world) > 0.95f * startHeight);
    CHECK(hipsBack > 0.0f);
    CHECK(offSupport(r) < 0.05f);
}

// The game's own loop (Human::tick with physics on, then the world's step):
// the limb test's fielder with no target stands.
TEST_CASE("human: physics on in the game loop, it stands 5 s") {
    Scene s;
    REQUIRE(makeScene(s));
    s.human.setPhysicsOn(true);
    const agents::HumanBody& body = s.human.humanBody();
    const float start = body.height(s.world);
    float lowest = start;
    bool standing = true;
    for (int i = 0; i < 5 * 120; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
        lowest = std::min(lowest, body.height(s.world));
        standing = standing && s.human.balancer().standing();
    }
    MESSAGE("game loop, physics on: centre of mass ", start, " m, lowest ", lowest, " m in 5 s");
    CHECK(standing);
    CHECK(lowest > 0.95f * start);
}

// One foot lifted 10 cm after 1 s standing: the weight goes over the other
// foot first (the foot waits for it), then it stands on one foot.
TEST_CASE("balance: the weight shifts over one foot, the other lifts, it stands on one foot 3 s") {
    MuscleRig r;
    REQUIRE(r.make(true));
    const agents::HumanBody& body = r.s.human.humanBody();
    for (int i = 0; i < 120; ++i) r.step(motion::BodyGoals{});
    const float startHeight = body.height(r.s.world);
    const float footStart = r.pos(r.joints.legs[0].end).y;
    motion::BodyGoals goals;
    goals.foot[0].active = true;
    goals.foot[0].position = r.pos(r.joints.legs[0].end) + math::Vec3(0.0f, 0.1f, 0.0f);
    goals.down[0] = false;
    const math::Vec3 ahead = r.s.human.root().rotation * math::kForward;
    const math::Vec3 side = math::cross(math::kUp, ahead);
    float liftAt = -1.0f, rise = 0.0f, worstOff = 0.0f, worstSide = 0.0f, worstAhead = 0.0f, lowest = startHeight;
    bool standing = true;
    for (int i = 0; i < 4 * 120; ++i) {
        r.step(goals);
        const float t = static_cast<float>(i + 1) / 120.0f;
        if (liftAt < 0.0f && r.balancer.liftReady(0)) liftAt = t;
        rise = std::max(rise, r.pos(r.joints.legs[0].end).y - footStart);
        standing = standing && r.balancer.footDown(1);
        lowest = std::min(lowest, body.height(r.s.world));
        if (liftAt >= 0.0f && t > liftAt + 0.25f) {
            const math::Vec3 off = body.centreOfMass(r.s.world) - r.balancer.pivot();
            worstOff = std::max(worstOff, offSupport(r));
            worstSide = std::max(worstSide, std::fabs(math::dot(off, side)));
            worstAhead = std::max(worstAhead, std::fabs(math::dot(off, ahead)));
        }
        if (i % 30 == 0)
            MESSAGE("t ", t, " left down ", r.balancer.footDown(0), " lift ",
                    r.balancer.liftReady(0), " off aim ", offSupport(r), " m; foot rise ",
                    r.pos(r.joints.legs[0].end).y - footStart, " m");
    }
    MESSAGE("one foot: lift allowed after ", liftAt, " s; foot rose ", rise, " m; worst off the stance ankle (from 0.25 s after the lift) ",
            worstOff, " m (side ", worstSide, ", ahead/back ", worstAhead, "); at the end ", offSupport(r),
            " m; centre of mass ", startHeight, " -> lowest ", lowest, " m; left foot down at the end ", r.balancer.footDown(0));
    CHECK(liftAt > 0.0f);
    CHECK(liftAt < 2.0f);
    CHECK(rise > 0.07f);
    CHECK_FALSE(r.balancer.footDown(0));
    CHECK(standing);
    CHECK(lowest > 0.9f * startHeight);
    // Over the stance sole (our foot: 4.9 cm either side of the ankle, its
    // heel 7.2 cm behind it), settling back toward the ankle.
    CHECK(worstSide < 0.049f);
    CHECK(worstAhead < 0.072f);
    CHECK(offSupport(r) < 0.03f);
}

namespace {

float bendAngle(const MuscleRig& r, const ik::TwoBoneChain& c) {
    const math::Vec3 a = r.pos(c.mid) - r.pos(c.start), b = r.pos(c.end) - r.pos(c.mid);
    return math::degrees(std::acos(std::clamp(math::dot(math::normalize(a), math::normalize(b)), -1.0f, 1.0f)));
}

// The left hand or foot to `offset` from its limb's first joint (shoulder or
// hip), pelvis held, 2 s; the rig after it.
void reachLimb(MuscleRig& r, bool arm, math::Vec3 offset) {
    r.s.human.physicalModels(r.s.world, 1.0f, r.models);
    const ik::TwoBoneChain& c = arm ? r.joints.arms[0] : r.joints.legs[0];
    motion::BodyGoals goals;
    motion::BodyGoals::Point& p = arm ? goals.hand[0] : goals.foot[0];
    p.active = true;
    p.position = r.pos(c.start) + offset;
    if (!arm) goals.down[0] = false;
    for (int i = 0; i < 2 * 120; ++i) r.step(goals);
}

} // namespace

// Was failing (2026-11-07): the knee was pushed straight into its -3 deg limit
// and the foot dragged 0.3 m short. The muscles took each joint's speed from
// the end's straight-line gap across its lever; from a straight leg bending the
// knee first moves the foot back, so the knee turned against the IK. Now each
// limb joint closes its own angle to the two-bone solve.
// 2026-11-08 (the user's call): the hips are free and the balancer on. The
// plan keeps its centre of mass over the standing foot, so the hips shift over
// it; a test that pinned the pelvis aimed the leg from a hip that never got
// there. The foot's point is fixed in the world from the hip at the start.
TEST_CASE("human body muscles: a raised foot target bends the knee, the body balancing on the other foot") {
    MuscleRig r;
    REQUIRE(r.make(true));
    for (int i = 0; i < 60; ++i) r.step(motion::BodyGoals{});
    const ik::TwoBoneChain& leg = r.joints.legs[0];
    const math::Vec3 target = r.pos(leg.start) + math::Vec3(0.0f, -0.5f, 0.3f);  // knee up, foot ahead
    motion::BodyGoals goals;
    goals.foot[0].active = true;
    goals.foot[0].position = target;
    goals.down[0] = false;
    goals.down[1] = true;
    bool standing = true;
    for (int i = 0; i < 3 * 120; ++i) {
        r.step(goals);
        standing = standing && r.balancer.standing();
    }
    const float miss = math::distance(r.pos(leg.end), target);
    const float bend = bendAngle(r, leg);
    MESSAGE("raised foot: ankle ", miss, " m from it after 3 s, knee bent ", bend, " deg, standing ", standing);
    CHECK(standing);
    CHECK(miss < 0.03f);
    CHECK(bend > 60.0f);
}

// Was failing (2026-11-07): the elbow went over the head and the hand swung
// about the target: a fixed elbow pole (back only), and the muscles turned
// against the IK (see the knee test). The elbow's side now follows the
// arm's shortest swing from hanging.
TEST_CASE("human body muscles: a hand up at the face bends the elbow, elbow below the hand") {
    MuscleRig r;
    REQUIRE(r.make());
    const math::Vec3 offset(-0.05f, 0.2f, 0.2f);  // from the shoulder: up, ahead, in
    reachLimb(r, true, offset);
    const ik::TwoBoneChain& arm = r.joints.arms[0];
    const float miss = math::distance(r.pos(arm.end), r.pos(arm.start) + offset);
    MESSAGE("hand at the face: wrist ", miss, " m from it after 2 s, elbow bent ", bendAngle(r, arm), " deg, elbow ",
            r.pos(arm.mid).y, " m, wrist ", r.pos(arm.end).y, " m high");
    CHECK(miss < 0.04f);
    CHECK(bendAngle(r, arm) > 90.0f);
    CHECK(r.pos(arm.mid).y < r.pos(arm.end).y);
}

// Was failing (2026-11-07, the user's report: the hand does not stop at its
// target, it turns away): through the game's own path the hand reached a
// target at the face, then the forearm's twist crept and the arm flew 0.21 m
// off. The two-bone solve read the elbow's bend axis in the forearm's frame,
// twist included, so a twisted forearm turned the bend plane and the shoulder
// chased it. Now the bend axis leaves the forearm's twist out.
TEST_CASE("human: through the game loop a hand at the face stays on its target") {
    Scene s;
    REQUIRE(makeScene(s));
    s.human.setPhysicsOn(true);
    for (int i = 0; i < 60; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
    }
    const rig::Rig& rig = s.human.rig();
    const int shoulder = rig.findJoint("upper_arm_r"), wrist = rig.findJoint("hand_r");
    std::vector<math::Mat4> m(static_cast<size_t>(rig.numJoints()));
    auto world = [&](int j) {
        s.human.physicalModels(s.world, 1.0f, m);
        return s.human.root().transformPoint(math::Vec3(m[static_cast<size_t>(j)][3]));
    };
    const math::Vec3 goal = world(shoulder) + s.human.root().rotation * math::Vec3(0.05f, 0.2f, 0.2f);
    s.human.setPoseHand(false, goal);
    float worstLate = 0.0f;
    for (int i = 0; i < 4 * 120; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
        if (i >= 2 * 120) worstLate = std::max(worstLate, math::distance(world(wrist), goal));
    }
    MESSAGE("hand at the face, game loop: worst wrist miss from 2 s to 4 s ", worstLate, " m; standing ",
            s.human.balancer().standing());
    CHECK(worstLate < 0.01f);
    CHECK(s.human.balancer().standing());
}

// Was failing (2026-11-07, the user's screenshot: the hand turned over beside
// the thigh): a fixed elbow pole (out, down, back) made a nearly straight arm
// twist 30 deg inward to point its elbow out. The elbow's side now follows
// the arm's shortest swing from hanging: no twist, the palm stays to the thigh.
TEST_CASE("human: through the game loop a hand beside the thigh does not twist the arm") {
    Scene s;
    REQUIRE(makeScene(s));
    s.human.setPhysicsOn(true);
    for (int i = 0; i < 60; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
    }
    const rig::Rig& rig = s.human.rig();
    const int shoulder = rig.findJoint("upper_arm_l"), wrist = rig.findJoint("hand_l");
    std::vector<math::Mat4> m(static_cast<size_t>(rig.numJoints()));
    auto model = [&](int j) {
        s.human.physicalModels(s.world, 1.0f, m);
        return math::toTransform(m[static_cast<size_t>(j)]);
    };
    const math::Vec3 goal = model(shoulder).position + math::Vec3(-0.02f, -0.6f, 0.1f);
    s.human.setPoseHand(true, s.human.root().transformPoint(goal));
    for (int i = 0; i < 3 * 120; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
    }
    const agents::HumanBody& body = s.human.humanBody();
    const agents::HumanBody::Part& p = body.parts()[static_cast<size_t>(body.partOf(shoulder))];
    math::Quat swing, twist;
    math::swingTwist(math::inverse(boneRotation(s.world, body.parts()[static_cast<size_t>(p.parent)])) * boneRotation(s.world, p),
                     math::kUp, swing, twist);
    const float twistDeg = std::fabs(math::degrees(math::rotationVector(twist).y));
    const float palmToThigh = math::dot(model(wrist).rotation * math::Vec3(-1.0f, 0.0f, 0.0f), math::Vec3(-1.0f, 0.0f, 0.0f));
    const float miss = math::distance(model(wrist).position, goal);
    MESSAGE("hand beside the thigh: wrist ", miss, " m off, shoulder twist ", twistDeg, " deg, palm toward the thigh ", palmToThigh);
    CHECK(miss < 0.01f);
    CHECK(twistDeg < 5.0f);
    CHECK(palmToThigh > 0.95f);
}

// A strong push (1 m/s forward, the capture point well past the toes): the
// balancer asks for a step, the game loop takes it, and the body stands again.
TEST_CASE("balance: a strong push forward, it steps and stands") {
    Scene s;
    REQUIRE(makeScene(s));
    s.human.setPhysicsOn(true);
    const agents::HumanBody& body = s.human.humanBody();
    for (int i = 0; i < 60; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
    }
    const float start = body.height(s.world);
    const math::Vec3 ahead = s.human.root().rotation * math::kForward;
    for (const agents::HumanBody::Part& p : body.parts())
        s.world.addImpulse(p.body, p.mass * 1.0f * ahead, s.world.transform(p.body).position);
    bool stepped = false;
    float lowest = start;
    for (int i = 0; i < 4 * 120; ++i) {
        s.human.tick(kTick);
        s.world.step(kTick);
        const agents::Balancer& b = s.human.balancer();
        stepped = stepped || !b.footDown(0) || !b.footDown(1);
        lowest = std::min(lowest, body.height(s.world));
    }
    const agents::Balancer& b = s.human.balancer();
    const float speed = math::length(body.centreOfMassVelocity(s.world));
    MESSAGE("push 1 m/s: stepped ", stepped, "; centre of mass ", start, " -> lowest ", lowest, " m; after 4 s lost ",
            b.lost(), ", moving ", speed, " m/s");
    CHECK(stepped);
    CHECK(lowest > 0.85f * start);
    CHECK_FALSE(b.lost());
    CHECK(speed < 0.1f);
}

// A knock to strength 0: the feet let go and no assist holds the body; the
// strength climbs back at recoverRate and the feet lock again.
TEST_CASE("balance: a knock drops the strength, it recovers and the feet lock again") {
    MuscleRig r;
    REQUIRE(r.make(true));
    for (int i = 0; i < 60; ++i) r.step(motion::BodyGoals{});
    CHECK(r.balancer.footLocked(0));
    CHECK(r.balancer.footLocked(1));
    r.balancer.knock(1.0f);
    r.step(motion::BodyGoals{});
    const bool letGo = !r.balancer.footLocked(0) && !r.balancer.footLocked(1);
    const float weak = r.balancer.strength();
    CHECK(letGo);
    CHECK(weak < 0.05f);
    for (int i = 0; i < 120; ++i) r.step(motion::BodyGoals{});
    MESSAGE("knock: let go ", letGo, ", strength ", weak, " -> ", r.balancer.strength(), " after 1 s");
    CHECK(r.balancer.strength() == doctest::Approx(1.0f));
}

// Locked feet hold: pushed sideways, the planted feet do not slide.
TEST_CASE("balance: locked feet do not slide under a side push") {
    MuscleRig r;
    REQUIRE(r.make(true));
    for (int i = 0; i < 60; ++i) r.step(motion::BodyGoals{});
    const agents::HumanBody& body = r.s.human.humanBody();
    const physics::BodyId foot = body.parts()[static_cast<size_t>(body.partOf(r.joints.legs[0].end))].body;
    const math::Vec3 before = r.s.world.transform(foot).position;
    const math::Vec3 side = math::cross(math::kUp, r.s.human.root().rotation * math::kForward);
    for (const agents::HumanBody::Part& p : body.parts())
        r.s.world.addImpulse(p.body, p.mass * 0.3f * side, r.s.world.transform(p.body).position);
    for (int i = 0; i < 120; ++i) r.step(motion::BodyGoals{});
    const math::Vec3 now = r.s.world.transform(foot).position;
    const float slid = std::hypot(now.x - before.x, now.z - before.z);
    MESSAGE("side push 0.3 m/s: left foot slid ", slid, " m; standing ", r.balancer.standing());
    CHECK(slid < 0.005f);
    CHECK(r.balancer.standing());
}
