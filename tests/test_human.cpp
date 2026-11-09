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
    fielder.setMoveInput(math::Vec2(0.0f, 1.0f), true, 0.0f);  // still held while walking
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


