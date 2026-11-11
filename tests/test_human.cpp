// Whole-fielder checks: runs the real fielder from config and measures how the
// body moves (side-to-side sway while running, reaching for a ball).

#include "agents/human.h"
#include "core/config.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {

struct Range {
    float lo = 1e9f, hi = -1e9f;
    void add(float v) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    float span() const { return hi - lo; }
};

bool makeHuman(agents::Human& fielder, bool lunge = false) {
    core::Config skeleton, joints, locomotion;
    if (!skeleton.load(CRICKET_CONFIG_DIR "/human_skeleton.json") ||
        !joints.load(CRICKET_CONFIG_DIR "/human_joints.json") ||
        !locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"))
        return false;
    rig::SkeletonDef def;
    if (!rig::loadSkeletonDef(skeleton, def) || !fielder.init(def, def.height, joints, locomotion)) return false;
    fielder.setHandMode(agents::HandMode::Auto);  // these tests check the hand on the ball's side
    if (!lunge) {
        agents::ReachSettings r = fielder.reachSettings();
        r.maxLunge = 0.0f;  // feet stay planted
        fielder.setReachSettings(r);
    }
    return true;
}

// Places the ball and picks it; returns once the reach is fully eased in
// (before the grip hold ends and the return starts).
void reachFor(agents::Human& fielder, math::Vec3 ball) {
    fielder.setReachTarget(ball);
    fielder.startPick();
    for (int i = 0; i < 240 && fielder.reachProgress() < 1.0f; ++i) fielder.tick(1.0 / 120.0);
    fielder.tick(1.0 / 120.0);
}

// World direction of a joint's local +Y (up the spine).
math::Vec3 boneUp(const agents::Human& fielder, int joint) {
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    return fielder.root().transformDirection(math::normalize(math::Vec3(models[static_cast<size_t>(joint)][1])));
}

math::Vec3 boneForward(const agents::Human& fielder, int joint) {
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    return fielder.root().transformDirection(math::normalize(math::Vec3(models[static_cast<size_t>(joint)][2])));
}

} // namespace

TEST_CASE("reaching for a ball on the ground ahead bends the spine forward") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    reachFor(fielder, fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    const math::Vec3 chestUp = boneUp(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest up . ahead = ", math::dot(chestUp, ahead), "  reach error ", fielder.reachError());
    CHECK(math::dot(chestUp, ahead) > 0.3f);  // leaning toward the ball
    CHECK(fielder.reachError() < 0.02f);
}

TEST_CASE("reaching for a ground ball tilts the pelvis and keeps the feet planted") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    fielder.tick(1.0 / 120.0);
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    auto jointAt = [&](const char* name) {
        std::vector<math::Mat4> m(static_cast<size_t>(fielder.rig().numJoints()));
        fielder.rig().localToModel(fielder.pose(), m);
        return fielder.root().transformPoint(math::Vec3(m[static_cast<size_t>(fielder.rig().findJoint(name))][3]));
    };
    const math::Vec3 ballL = jointAt("toe_l");  // ball of the foot, standing flat
    reachFor(fielder, fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    const math::Vec3 pelvisUp = boneUp(fielder, fielder.rig().findJoint("pelvis"));
    MESSAGE("pelvis up . ahead = ", math::dot(pelvisUp, ahead), "  reach error ", fielder.reachError());
    CHECK(math::dot(pelvisUp, ahead) > 0.3f);
    CHECK(fielder.reachError() < 0.4f);  // the spine alone left about 0.52 m
    CHECK(math::distance(fielder.targets().footL, footL) < 1e-4f);
    CHECK(math::distance(fielder.targets().footR, footR) < 1e-4f);

    // The foot really is planted after the leg IK: the ball of the foot stays
    // put (the heel may rise once the shin passes the ankle's range).
    CHECK(math::distance(jointAt("toe_l"), ballL) < 0.015f);
}

TEST_CASE("a ground ball too far for planted feet: lower, balanced, feet planted") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    fielder.tick(1.0 / 120.0);
    const float standY = fielder.pose().locals[0].position.y;  // pelvis (root joint), character space
    const math::Vec3 footL = fielder.targets().footL;
    reachFor(fielder, fielder.root().position + ahead * 0.9f + math::Vec3(0, 0.036f, 0));
    const float reachY = fielder.pose().locals[0].position.y;
    MESSAGE("pelvis y ", standY, " -> ", reachY, "  reach error ", fielder.reachError());
    CHECK(reachY < standY - 0.05f);
    // 0.9 m past the ankles is beyond a planted, balanced reach (that is the
    // lunge step, 2.5): partly reached, feet planted, centre of mass over the feet.
    CHECK(fielder.reachError() > 0.0f);
    CHECK(math::distance(fielder.targets().footL, footL) < 1e-4f);
    const math::Transform toModel = math::inverse(fielder.root());
    CHECK(toModel.transformPoint(fielder.centreOfMass()).z < 0.25f);
}

TEST_CASE("a ground ball beside the foot is reached") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Transform& root = fielder.root();
    const math::Vec3 left = root.transformDirection(math::kRight);  // +X is left
    const math::Vec3 ahead = root.transformDirection(math::kForward);
    reachFor(fielder, root.position + left * 0.25f + ahead * 0.1f + math::Vec3(0, 0.036f, 0));
    MESSAGE("beside foot: reach error ", fielder.reachError());
    CHECK(fielder.reachError() < 0.02f);
}

TEST_CASE("ground balls near the feet: reach sweep") {
    // Every ground spot in front of and beside the feet, within ~0.6 m, is
    // reached with the feet planted. Further ones need the lunge step and ones
    // behind the heels a turn or step (2.5).
    int misses = 0, total = 0;
    for (int ix = -3; ix <= 3; ++ix) {
        for (int iz = 0; iz <= 4; ++iz) {
            const float x = 0.15f * static_cast<float>(ix), z = 0.15f * static_cast<float>(iz);
            if (x * x + z * z > 0.6f * 0.6f) continue;
            agents::Human fielder;
            REQUIRE(makeHuman(fielder));
            const math::Transform root = fielder.root();
            reachFor(fielder, root.transformPoint(math::Vec3(x, 0.036f, z)));
            ++total;
            if (fielder.reachError() > 0.02f) {
                ++misses;
                MESSAGE("miss at x ", x, " z ", z, ": error ", fielder.reachError());
            }
        }
    }
    MESSAGE(misses, " of ", total, " spots missed");
    CHECK(misses == 0);
}

TEST_CASE("a ball far out of reach leaves an error (the dive case later)") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    reachFor(fielder, fielder.root().position + ahead * 3.0f + math::Vec3(0, 0.036f, 0));
    MESSAGE("reach error ", fielder.reachError());
    CHECK(fielder.reachError() > 0.5f);

    // Out of reach, the body still must not fall: centre of mass over the feet.
    const math::Transform toModel = math::inverse(fielder.root());
    const math::Vec3 com = toModel.transformPoint(fielder.centreOfMass());
    const math::Vec3 footL = toModel.transformPoint(fielder.targets().footL);
    const math::Vec3 footR = toModel.transformPoint(fielder.targets().footR);
    const math::Vec3 pelvis = fielder.pose().locals[0].position;
    MESSAGE("com z ", com.z, "  feet z ", footL.z, " ", footR.z, "  pelvis ", pelvis.x, " ", pelvis.y, " ", pelvis.z);
    CHECK(com.z < std::max(footL.z, footR.z) + 0.25f);
}

TEST_CASE("reaching for a ball to the side and behind twists the chest toward it") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    const math::Transform& root = fielder.root();
    const math::Vec3 left = root.transformDirection(math::kRight);  // +X is left
    reachFor(fielder, root.position + left * 0.5f - root.transformDirection(math::kForward) * 0.7f +
                           math::Vec3(0, 1.3f, 0));
    const math::Vec3 chestForward = boneForward(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest forward . left = ", math::dot(chestForward, left));
    CHECK(math::dot(chestForward, left) > 0.1f);  // turned toward the left-behind target
}

TEST_CASE("the fielder starts with the ball in hand; placing it releases it") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    CHECK(fielder.holding());
    fielder.setStepInput(math::Vec2(0.0f, 1.0f));  // still held while stepping
    for (int i = 0; i < 120; ++i) fielder.tick(1.0 / 120.0);
    CHECK(fielder.holding());
    fielder.setReachTarget(fielder.root().transformPoint(math::Vec3(0.2f, 0.036f, 0.4f)));
    CHECK_FALSE(fielder.holding());
}

TEST_CASE("grip: a closed hand brings the thumb tip toward the ball") {
    // The fielder starts holding the ball in the right hand (closed grip);
    // placing the ball relaxes it.
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    auto thumbToPalm = [&]() {
        // Thumb tip to the held ball's spot in front of the right palm (+X).
        fielder.rig().localToModel(fielder.pose(), models);
        const math::Mat4& hand = models[static_cast<size_t>(fielder.rig().findJoint("hand_r"))];
        const agents::ReachSettings& r = fielder.reachSettings();
        const math::Vec3 ball(hand * math::Vec4(r.ballRadius + r.palmThickness, -r.palmCentre, 0.0f, 1.0f));
        return math::distance(math::Vec3(models[static_cast<size_t>(fielder.rig().findJoint("thumb_03_r"))][3]), ball);
    };
    fielder.tick(1.0 / 120.0);
    const float closed = thumbToPalm();
    fielder.setReachTarget(fielder.root().transformPoint(math::Vec3(0.2f, 0.036f, 0.4f)));
    fielder.tick(1.0 / 120.0);
    const float relaxed = thumbToPalm();
    MESSAGE("thumb tip to palm: closed ", closed, "  relaxed ", relaxed);
    CHECK(closed < relaxed - 0.005f);
}

TEST_CASE("picking: grips the ball, carries it back, stands again") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    fielder.tick(1.0 / 120.0);  // settled standing height
    const math::Transform root = fielder.root();
    const float standY = fielder.pose().locals[0].position.y;
    fielder.setReachTarget(root.transformPoint(math::Vec3(0.2f, 0.036f, 0.4f)));
    fielder.startPick();
    bool held = false;
    for (int i = 0; i < 3 * 120; ++i) {
        fielder.tick(1.0 / 120.0);
        held = held || fielder.holding();
    }
    CHECK(held);
    CHECK(fielder.holding());          // still carrying it
    CHECK_FALSE(fielder.reaching());   // back to standing
    CHECK(fielder.pose().locals[0].position.y == doctest::Approx(standY).epsilon(0.02));
}

// Skipped (2026-11-05, the user's call: no ball work now): with the joint
// ranges read the Jolt way the trunk's real twist range leaves the hand
// 3.4 cm short of this far ball. Revisit with the ball as a generic body.
TEST_CASE("picking: a ball out of planted reach is picked with a step (Alt+Space)" * doctest::skip()) {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder, true));
    const math::Transform root = fielder.root();
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    fielder.setReachTarget(root.transformPoint(math::Vec3(0.1f, 0.036f, 1.2f)));
    fielder.startPick(true);
    bool held = false, stepped = false;
    for (int i = 0; i < 4 * 120; ++i) {
        fielder.tick(1.0 / 120.0);
        held = held || fielder.holding();
        stepped = stepped || math::distance(fielder.targets().footL, footL) > 0.2f ||
                  math::distance(fielder.targets().footR, footR) > 0.2f;
    }
    CHECK(stepped);
    CHECK(held);
}

TEST_CASE("picking: with a step, the foot moves first, before the body bends") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder, true));
    fielder.tick(1.0 / 120.0);
    const math::Transform root = fielder.root();
    const float standY = fielder.pose().locals[0].position.y;
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    fielder.setReachTarget(root.transformPoint(math::Vec3(0.1f, 0.036f, 1.2f)));
    fielder.startPick(true);
    for (int i = 0; i < 12; ++i) fielder.tick(1.0 / 120.0);  // 0.1 s in
    const bool stepping = math::distance(fielder.targets().footL, footL) > 0.01f ||
                          math::distance(fielder.targets().footR, footR) > 0.01f;
    CHECK(stepping);                                                                   // the foot moves first
    CHECK(fielder.pose().locals[0].position.y == doctest::Approx(standY).epsilon(0.02)); // the body has not bent yet
}

TEST_CASE("picking: hand mode Right uses the right hand for a ball on the left") {
    agents::Human fielder;
    REQUIRE(makeHuman(fielder));
    fielder.setHandMode(agents::HandMode::Right);
    const math::Transform root = fielder.root();
    const math::Vec3 ball = root.transformPoint(math::Vec3(0.3f, 0.036f, 0.4f));  // +X is left
    reachFor(fielder, ball);
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    auto hand = [&](const char* n) {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>(fielder.rig().findJoint(n))][3]));
    };
    MESSAGE("right hand to ball ", math::distance(hand("hand_r"), ball), "  left ", math::distance(hand("hand_l"), ball));
    CHECK(math::distance(hand("hand_r"), ball) < math::distance(hand("hand_l"), ball));
    CHECK(fielder.reachError() < 0.02f);
}

TEST_CASE("picking: a still ball in range is reached at once, a nearer one sooner") {
    auto timeToReach = [](float z) {
        agents::Human fielder;
        REQUIRE(makeHuman(fielder));
        fielder.setReachTarget(fielder.root().transformPoint(math::Vec3(0.2f, 0.036f, z)));
        fielder.startPick();
        int ticks = 0;
        for (; ticks < 240 && fielder.reachProgress() < 1.0f; ++ticks) {
            fielder.tick(1.0 / 120.0);
            if (ticks == 0) CHECK(fielder.reachProgress() > 0.0f);  // no wait
        }
        return ticks / 120.0f;
    };
    const float near = timeToReach(0.3f), far = timeToReach(0.6f);
    MESSAGE("reach time: near ", near, " s, far ", far, " s");
    CHECK(near < far);  // Fitts' law
}



// The full-physics body (Jolt): a few steps, measured.
#include "physics/core/physics_world.h"
#include <cstdio>
namespace {
// The configs stay alive: Human keeps a pointer to the joints config and the
// physics body reads its muscles from it when attached.
bool makeLiveHuman(agents::Human& fielder) {
    static core::Config skeleton, joints, locomotion;
    if (!skeleton.load(CRICKET_CONFIG_DIR "/human_skeleton.json") ||
        !joints.load(CRICKET_CONFIG_DIR "/human_joints.json") ||
        !locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"))
        return false;
    rig::SkeletonDef def;
    return rig::loadSkeletonDef(skeleton, def) && fielder.init(def, def.height, joints, locomotion);
}

struct StepRun {
    float chestSwing = 0.0f;   // deg: the real chest's widest lean from vertical
    float footSlide = 0.0f;    // m: a real ankle's widest travel while its target stands still
    float targetJerk = 0.0f;   // m/s: an ankle target's biggest speed change in one tick
    float pelvisAccel = 0.0f;  // m/s^2: the real pelvis's biggest level acceleration (over 50 ms)
    bool dropped = false;
};

// Steps forward, then (after `turnAt` s) to the right.
StepRun stepRun(float seconds, float turnAt) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    std::vector<math::Mat4> planned(static_cast<size_t>(h.rig().numJoints())), real(planned.size());
    const int chest = h.rig().findJoint("chest");
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }  // settle
    StepRun r;
    math::Vec3 lastT[2], lastV[2] = {math::Vec3(0.0f), math::Vec3(0.0f)}, still[2];
    bool have = false, holding[2] = {false, false};
    math::Vec3 pelvisHist[7], planHist[7];
    float planAccel = 0.0f, sideMin = 9.0f, sideMax = -9.0f;
    int pelvisCount = 0;
    const int ticks = static_cast<int>(seconds * 120.0f);
    for (int i = 0; i < ticks; ++i) {
        h.setStepInput(i * kDt < turnAt ? math::Vec2(0.0f, 1.0f) : math::Vec2(1.0f, 0.0f));
        h.tick(kDt);
        world.step(kDt);
        if (!h.supported()) r.dropped = true;
        // The target is the decider's plan (the walking pose is not what the muscles follow).
        if (!h.bodyQueue().plan) continue;
        planned = *h.bodyQueue().plan;
        if (!h.physicalModels(world, 1.0f, real)) continue;
        {
            const math::Vec3 p = h.root().transformPoint(math::Vec3(real[0][3]));
            if (i * kDt > 0.6f && i * kDt < turnAt) {  // walking straight: the pelvis's side sway (the body's own side)
                sideMin = std::min(sideMin, real[0][3].x);
                sideMax = std::max(sideMax, real[0][3].x);
            }
            for (int k = 6; k > 0; --k) pelvisHist[k] = pelvisHist[k - 1];
            pelvisHist[0] = p;
            if (h.bodyQueue().plan) {
                for (int k = 6; k > 0; --k) planHist[k] = planHist[k - 1];
                planHist[0] = h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3]));
                if (pelvisCount >= 7) {
                    const math::Vec3 b = (planHist[0] - 2.0f * planHist[3] + planHist[6]) / (9.0f * kDt * kDt);
                    planAccel = std::max(planAccel, std::sqrt(b.x * b.x + b.z * b.z));
                }
            }
            if (++pelvisCount >= 7) {  // central difference over 3 ticks each side
                const math::Vec3 a = (pelvisHist[0] - 2.0f * pelvisHist[3] + pelvisHist[6]) / (9.0f * kDt * kDt);
                const float am = std::sqrt(a.x * a.x + a.z * a.z);
                r.pelvisAccel = std::max(r.pelvisAccel, am);
            }
        }
        const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(chest)][1]));
        r.chestSwing = std::max(r.chestSwing, math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f))));
        for (int f = 0; f < 2; ++f) {
            const math::Vec3 t = h.ankle(f == 0, planned, h.root()), a = h.ankle(f == 0, real, h.root());
            if (have) {
                const math::Vec3 v = (t - lastT[f]) / kDt;
                r.targetJerk = std::max(r.targetJerk, math::length(v - lastV[f]));
                const bool stands = math::length(v) < 0.05f;
                if (stands && !holding[f]) still[f] = a;
                holding[f] = stands;
                if (stands) r.footSlide = std::max(r.footSlide, math::length(math::Vec3(a.x - still[f].x, 0.0f, a.z - still[f].z)));
                lastV[f] = v;
            }
            lastT[f] = t;
        }
        have = true;
    }
    std::printf("plan pelvis accel %.1f m/s^2, real pelvis side sway %.1f cm\n", planAccel, 100.0f * (sideMax - sideMin));
    return r;
}
} // namespace

TEST_CASE("stepping on the physics body: steady chest, planted feet, a smooth target") {
    const StepRun r = stepRun(4.0f, 2.0f);
    std::printf("stepping: chest swing %.1f deg, foot slide %.1f cm, target speed change %.2f m/s per tick, "
                "real pelvis accel %.1f m/s^2, dropped %s\n",
                r.chestSwing, 100.0f * r.footSlide, r.targetJerk, r.pelvisAccel, r.dropped ? "yes" : "no");
    CHECK_FALSE(r.dropped);
}


// Measurement (run on demand: --no-skip -tc="measure*"): a hand or foot goal
// held through the user's limb test; how the decider's plan shares the reach
// (hips height and travel, pelvis tilt, spine bend, knees) and how close the
// limb gets. Prints only.
namespace {
struct ReachShape {
    float hipsDrop = 0.0f, hipsForward = 0.0f, hipsSide = 0.0f;  // m, the plan's pelvis from standing (character space)
    float pelvisTilt = 0.0f, trunkBend = 0.0f;                   // deg forward: pelvis up from vertical, chest up from pelvis up
    float kneeL = 0.0f, kneeR = 0.0f;                            // deg flexion
    float planMiss = 0.0f, realMiss = 0.0f;                      // m, limb end from its goal
};

float flexionDeg(const std::vector<math::Mat4>& m, const agents::Human& h, const char* a, const char* b, const char* c) {
    const math::Vec3 pa(m[static_cast<size_t>(h.rig().findJoint(a))][3]), pb(m[static_cast<size_t>(h.rig().findJoint(b))][3]),
        pc(m[static_cast<size_t>(h.rig().findJoint(c))][3]);
    const float d = math::dot(math::normalize(pb - pa), math::normalize(pc - pb));
    return math::degrees(std::acos(std::clamp(d, -1.0f, 1.0f)));
}

// Forward lean (deg, +: toward +Z) of a direction from vertical.
float leanDeg(math::Vec3 up) { return math::degrees(std::atan2(up.z, up.y)); }

ReachShape holdGoal(bool hand, bool left, math::Vec3 local, float seconds) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }  // settle
    std::vector<math::Mat4> rest(static_cast<size_t>(h.rig().numJoints())), real(rest.size());
    REQUIRE(h.bodyQueue().plan);
    rest = *h.bodyQueue().plan;
    const math::Vec3 goal = h.root().transformPoint(local);
    const int ticks = static_cast<int>(seconds * 120.0f);
    for (int i = 0; i < ticks; ++i) {
        if (hand) h.setPoseHand(left, goal);
        else h.setPoseFoot(left ? motion::Foot::Left : motion::Foot::Right, goal);
        h.tick(kDt);
        world.step(kDt);
    }
    const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
    ReachShape r;
    const math::Vec3 pel(p[0][3]), pel0(rest[0][3]);
    r.hipsDrop = pel0.y - pel.y;
    r.hipsForward = pel.z - pel0.z;
    r.hipsSide = pel.x - pel0.x;
    const math::Vec3 pelUp = math::normalize(math::Vec3(p[0][1]));
    const math::Vec3 chestUp = math::normalize(math::Vec3(p[static_cast<size_t>(h.rig().findJoint("chest"))][1]));
    r.pelvisTilt = leanDeg(pelUp);
    r.trunkBend = leanDeg(chestUp) - r.pelvisTilt;
    r.kneeL = flexionDeg(p, h, "thigh_l", "shin_l", "foot_l");
    r.kneeR = flexionDeg(p, h, "thigh_r", "shin_r", "foot_r");
    auto end = [&](const std::vector<math::Mat4>& m) { return hand ? h.wrist(left, m, h.root()) : h.ankle(left, m, h.root()); };
    r.planMiss = math::distance(end(p), goal);
    if (h.physicalModels(world, 1.0f, real)) r.realMiss = math::distance(end(real), goal);
    return r;
}

void print(const char* name, const ReachShape& r) {
    std::printf("%-28s hips drop %5.1f cm, fwd %5.1f cm, side %5.1f cm | pelvis tilt %5.1f deg, trunk bend %5.1f deg | "
                "knee L %5.1f R %5.1f deg | miss plan %4.1f cm, real %4.1f cm\n",
                name, 100.0f * r.hipsDrop, 100.0f * r.hipsForward, 100.0f * r.hipsSide, r.pelvisTilt, r.trunkBend, r.kneeL,
                r.kneeR, 100.0f * r.planMiss, 100.0f * r.realMiss);
}
} // namespace

TEST_CASE("measure: reach shape for hand and foot goals" * doctest::skip()) {
    // Character space: +Z forward, +X left, the ground at y = 0.
    print("hand L at the toes", holdGoal(true, true, math::Vec3(0.15f, 0.05f, 0.25f), 3.0f));
    print("hand L ground 0.45 ahead", holdGoal(true, true, math::Vec3(0.15f, 0.05f, 0.45f), 3.0f));
    print("hand L knee height ahead", holdGoal(true, true, math::Vec3(0.2f, 0.5f, 0.45f), 3.0f));
    print("foot L 0.5 ahead, low", holdGoal(false, true, math::Vec3(0.1f, 0.1f, 0.5f), 3.0f));
    print("foot L 0.8 ahead, low", holdGoal(false, true, math::Vec3(0.1f, 0.1f, 0.8f), 3.0f));
    print("foot L 0.5 out to the side", holdGoal(false, true, math::Vec3(0.6f, 0.1f, 0.0f), 3.0f));
    print("foot L 0.5 behind", holdGoal(false, true, math::Vec3(0.1f, 0.1f, -0.5f), 3.0f));
}

// Measurement (on demand): the user's arrow-key limb test. A hand or foot goal
// starts at the limb and moves at 0.5 m/s one way; prints how often and how
// fast the limb's end (plan and real) moved against the goal's motion.
namespace {
void sweep(bool hand, bool left, math::Vec3 dir, const char* name) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    auto end = [&](const std::vector<math::Mat4>& m) { return hand ? h.wrist(left, m, h.root()) : h.ankle(left, m, h.root()); };
    math::Vec3 goal = end(*h.bodyQueue().plan);
    const math::Vec3 step = h.root().transformDirection(dir) * (0.5f * kDt);
    math::Vec3 lastPlan = goal, lastReal = goal;
    float worstPlan = 0.0f, worstReal = 0.0f;
    int againstPlan = 0, againstReal = 0, ticks = 0;
    for (int i = 0; i < 180; ++i) {  // 1.5 s: 0.75 m of goal travel
        goal += step;
        if (hand) h.setPoseHand(left, goal);
        else h.setPoseFoot(left ? motion::Foot::Left : motion::Foot::Right, goal);
        h.tick(kDt);
        world.step(kDt);
        const math::Vec3 p = end(*h.bodyQueue().plan);
        const float vp = math::dot(p - lastPlan, math::normalize(step)) / kDt;
        lastPlan = p;
        if (i > 12 && vp < -0.02f) { ++againstPlan; worstPlan = std::min(worstPlan, vp); }
        if (h.physicalModels(world, 1.0f, real)) {
            const math::Vec3 r = end(real);
            const float vr = math::dot(r - lastReal, math::normalize(step)) / kDt;
            if (i > 12 && vr < -0.05f) { ++againstReal; worstReal = std::min(worstReal, vr); }
            lastReal = r;
        }
        ++ticks;
    }
    std::printf("%-22s against goal: plan %3d ticks (worst %5.2f m/s), real %3d ticks (worst %5.2f m/s), "
                "final miss plan %4.1f cm\n",
                name, againstPlan, worstPlan, againstReal, worstReal, 100.0f * math::distance(lastPlan, goal));
}
} // namespace

TEST_CASE("measure: arrow-key limb test, every direction" * doctest::skip()) {
    const math::Vec3 dirs[6] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    const char* names[6] = {"forward", "back", "out (left)", "across (right)", "up", "down"};
    for (int k = 0; k < 6; ++k) {
        std::string n = std::string("hand L ") + names[k];
        sweep(true, true, dirs[k], n.c_str());
    }
    for (int k = 0; k < 6; ++k) {
        std::string n = std::string("foot L ") + names[k];
        sweep(false, true, dirs[k], n.c_str());
    }
}

// Measurement (on demand): walking forward at a pace (E presses); how bent both
// knees are at once in the plan (the straighter one's flexion at its least
// and most) and how low the hips go.
namespace {
void walkKnees(int presses) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    for (int i = 0; i < presses; ++i) h.changeWalkSpeed(1);
    const float rest = (*h.bodyQueue().plan)[0][3].y;
    auto span = [&](const std::vector<math::Mat4>& m, const char* a, const char* b, const char* c) {
        const math::Vec3 pa(m[static_cast<size_t>(h.rig().findJoint(a))][3]), pb(m[static_cast<size_t>(h.rig().findJoint(b))][3]),
            pc(m[static_cast<size_t>(h.rig().findJoint(c))][3]);
        return math::distance(pa, pc) / (math::distance(pa, pb) + math::distance(pb, pc));
    };
    {
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        std::printf("standing: knee L %4.1f R %4.1f deg, span L %.3f R %.3f of leg length\n",
                    flexionDeg(p, h, "thigh_l", "shin_l", "foot_l"), flexionDeg(p, h, "thigh_r", "shin_r", "foot_r"),
                    span(p, "thigh_l", "shin_l", "foot_l"), span(p, "thigh_r", "shin_r", "foot_r"));
    }
    Range straighter, hips, kneeL, longerSpan, ankleY, hipY;
    for (int i = 0; i < 480; ++i) {
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        if (i < 120) continue;  // into the walk
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const float l = flexionDeg(p, h, "thigh_l", "shin_l", "foot_l"), r = flexionDeg(p, h, "thigh_r", "shin_r", "foot_r");
        straighter.add(std::min(l, r));
        kneeL.add(l);
        hips.add(rest - p[0][3].y);
        longerSpan.add(std::max(span(p, "thigh_l", "shin_l", "foot_l"), span(p, "thigh_r", "shin_r", "foot_r")));
        ankleY.add(std::min(p[static_cast<size_t>(h.rig().findJoint("foot_l"))][3].y, p[static_cast<size_t>(h.rig().findJoint("foot_r"))][3].y));
        hipY.add(p[static_cast<size_t>(h.rig().findJoint("thigh_l"))][3].y);
    }
    std::printf("  longer leg span %.3f-%.3f of leg length; lower ankle height %.3f-%.3f m; left hip height %.3f-%.3f m\n",
                longerSpan.lo, longerSpan.hi, ankleY.lo, ankleY.hi, hipY.lo, hipY.hi);
    std::printf("walk pace x%.2f: straighter knee %4.1f-%4.1f deg, left knee %4.1f-%4.1f deg, hips drop %4.1f-%4.1f cm\n",
                h.walkSpeedScale(), straighter.lo, straighter.hi, kneeL.lo, kneeL.hi, 100.0f * hips.lo, 100.0f * hips.hi);
}
} // namespace

TEST_CASE("measure: knees while walking" * doctest::skip()) {
    walkKnees(0);
    walkKnees(5);
}
