// Whole-fielder checks: runs the real fielder from config and measures how the
// body moves (side-to-side sway while running, reaching for a ball).

#include "agents/fielder.h"
#include "core/config.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
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

bool makeFielder(agents::Fielder& fielder, bool lunge = false) {
    core::Config skeleton, joints, locomotion;
    if (!skeleton.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json") ||
        !joints.load(CRICKET_CONFIG_DIR "/fielder_joints.json") ||
        !locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"))
        return false;
    rig::SkeletonDef def;
    if (!rig::loadSkeletonDef(skeleton, def) || !fielder.init(def, def.height, joints, locomotion)) return false;
    if (!lunge) {
        agents::ReachSettings r = fielder.reachSettings();
        r.maxLunge = 0.0f;  // feet stay planted
        fielder.setReachSettings(r);
    }
    return true;
}

// Places the ball and picks it; returns once the reach is fully eased in
// (before the grip hold ends and the return starts).
void reachFor(agents::Fielder& fielder, math::Vec3 ball) {
    fielder.setReachTarget(ball);
    fielder.startPick();
    for (int i = 0; i < 56; ++i) fielder.tick(1.0 / 120.0);
}

// World direction of a joint's local +Y (up the spine).
math::Vec3 boneUp(const agents::Fielder& fielder, int joint) {
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    return fielder.root().transformDirection(math::normalize(math::Vec3(models[static_cast<size_t>(joint)][1])));
}

math::Vec3 boneForward(const agents::Fielder& fielder, int joint) {
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    return fielder.root().transformDirection(math::normalize(math::Vec3(models[static_cast<size_t>(joint)][2])));
}

} // namespace

TEST_CASE("reaching for a ball on the ground ahead bends the spine forward") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    reachFor(fielder, fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    const math::Vec3 chestUp = boneUp(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest up . ahead = ", math::dot(chestUp, ahead), "  reach error ", fielder.reachError());
    CHECK(math::dot(chestUp, ahead) > 0.3f);  // leaning toward the ball
    CHECK(fielder.reachError() < 0.02f);
}

TEST_CASE("reaching for a ground ball tilts the pelvis and keeps the feet planted") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    fielder.tick(1.0 / 120.0);
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    reachFor(fielder, fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    const math::Vec3 pelvisUp = boneUp(fielder, fielder.rig().findJoint("pelvis"));
    MESSAGE("pelvis up . ahead = ", math::dot(pelvisUp, ahead), "  reach error ", fielder.reachError());
    CHECK(math::dot(pelvisUp, ahead) > 0.3f);
    CHECK(fielder.reachError() < 0.4f);  // the spine alone left about 0.52 m
    CHECK(math::distance(fielder.targets().footL, footL) < 1e-4f);
    CHECK(math::distance(fielder.targets().footR, footR) < 1e-4f);

    // The ankles really are at the foot targets after the leg IK.
    std::vector<math::Mat4> models(static_cast<size_t>(fielder.rig().numJoints()));
    fielder.rig().localToModel(fielder.pose(), models);
    const math::Vec3 ankle = fielder.root().transformPoint(math::Vec3(models[static_cast<size_t>(fielder.rig().findJoint("foot_l"))][3]));
    CHECK(math::distance(ankle, footL) < 0.015f);
}

TEST_CASE("a ground ball too far for planted feet: lower, balanced, feet planted") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
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
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
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
            agents::Fielder fielder;
            REQUIRE(makeFielder(fielder));
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
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
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
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    const math::Transform& root = fielder.root();
    const math::Vec3 left = root.transformDirection(math::kRight);  // +X is left
    reachFor(fielder, root.position + left * 0.5f - root.transformDirection(math::kForward) * 0.7f +
                           math::Vec3(0, 1.3f, 0));
    const math::Vec3 chestForward = boneForward(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest forward . left = ", math::dot(chestForward, left));
    CHECK(math::dot(chestForward, left) > 0.1f);  // turned toward the left-behind target
}

TEST_CASE("the fielder starts with the ball in hand; placing it releases it") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    CHECK(fielder.holding());
    fielder.setMoveInput(math::Vec2(0.0f, 1.0f), true, 0.0f);  // still held while walking
    for (int i = 0; i < 120; ++i) fielder.tick(1.0 / 120.0);
    CHECK(fielder.holding());
    fielder.setReachTarget(fielder.root().transformPoint(math::Vec3(0.2f, 0.036f, 0.4f)));
    CHECK_FALSE(fielder.holding());
}

TEST_CASE("picking: grips the ball, carries it back, stands again") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
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

TEST_CASE("picking: a ball out of planted reach is picked with a lunge step") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder, true));
    const math::Transform root = fielder.root();
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    fielder.setReachTarget(root.transformPoint(math::Vec3(0.1f, 0.036f, 1.2f)));
    fielder.startPick();
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

TEST_CASE("picking: hand mode Right uses the right hand for a ball on the left") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
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

TEST_CASE("running fielder keeps the body and feet in their lanes") {
    core::Config skeleton, joints, locomotion;
    REQUIRE(skeleton.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json"));
    REQUIRE(joints.load(CRICKET_CONFIG_DIR "/fielder_joints.json"));
    REQUIRE(locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"));
    rig::SkeletonDef def;
    REQUIRE(rig::loadSkeletonDef(skeleton, def));
    agents::Fielder fielder;
    REQUIRE(fielder.init(def, def.height, joints, locomotion));

    fielder.setGait(motion::Gait::Run);
    fielder.setMoveInput(math::Vec2(0.0f, 1.0f), true, 0.0f);
    const double dt = 1.0 / 120.0;
    Range pelvis, footL, footR;
    for (int i = 0; i < 4 * 120; ++i) {
        fielder.tick(dt);
        if (i < 2 * 120) continue;  // up to speed first
        const math::Transform toModel = math::inverse(fielder.root());
        pelvis.add(fielder.pose().locals[0].position.x);  // the pelvis is the root joint (character space)
        footL.add(toModel.transformPoint(fielder.targets().footL).x);
        footR.add(toModel.transformPoint(fielder.targets().footR).x);
    }
    MESSAGE("pelvis x [", pelvis.lo, ", ", pelvis.hi, "]  footL x [", footL.lo, ", ", footL.hi, "]  footR x [", footR.lo,
            ", ", footR.hi, "]");
    CHECK(pelvis.span() < 0.04f);  // running CoM moves about 1-2 cm sideways
    CHECK(footL.lo > -0.02f);      // feet never cross the midline (+X is left)
    CHECK(footR.hi < 0.02f);
}
