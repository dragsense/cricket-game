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

bool makeFielder(agents::Fielder& fielder) {
    core::Config skeleton, joints, locomotion;
    if (!skeleton.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json") ||
        !joints.load(CRICKET_CONFIG_DIR "/fielder_joints.json") ||
        !locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"))
        return false;
    rig::SkeletonDef def;
    return rig::loadSkeletonDef(skeleton, def) && fielder.init(def, def.height, joints, locomotion);
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
    fielder.setReachTarget(fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    for (int i = 0; i < 10; ++i) fielder.tick(1.0 / 120.0);
    const math::Vec3 chestUp = boneUp(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest up . ahead = ", math::dot(chestUp, ahead), "  reach error ", fielder.reachError());
    CHECK(math::dot(chestUp, ahead) > 0.3f);  // leaning toward the ball
}

TEST_CASE("reaching for a ground ball tilts the pelvis and keeps the feet planted") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    const math::Vec3 ahead = fielder.root().transformDirection(math::kForward);
    fielder.tick(1.0 / 120.0);
    const math::Vec3 footL = fielder.targets().footL, footR = fielder.targets().footR;
    fielder.setReachTarget(fielder.root().position + ahead * 0.5f + math::Vec3(0, 0.036f, 0));
    for (int i = 0; i < 10; ++i) fielder.tick(1.0 / 120.0);
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
    CHECK(math::distance(ankle, footL) < 0.02f);
}

TEST_CASE("reaching for a ball to the side and behind twists the chest toward it") {
    agents::Fielder fielder;
    REQUIRE(makeFielder(fielder));
    const math::Transform& root = fielder.root();
    const math::Vec3 left = root.transformDirection(math::kRight);  // +X is left
    fielder.setReachTarget(root.position + left * 0.5f - root.transformDirection(math::kForward) * 0.7f +
                           math::Vec3(0, 1.3f, 0));
    for (int i = 0; i < 10; ++i) fielder.tick(1.0 / 120.0);
    const math::Vec3 chestForward = boneForward(fielder, fielder.rig().findJoint("chest"));
    MESSAGE("chest forward . left = ", math::dot(chestForward, left));
    CHECK(math::dot(chestForward, left) > 0.1f);  // turned toward the left-behind target
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
