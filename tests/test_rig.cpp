// Tests for the fielder skeleton: config parsing, mirroring, ozz build and
// rest-pose landmark heights against the anthropometric ratios
// (docs/BIOMECHANICS.md section 1).

#include "core/config.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

using namespace rig;

namespace {

SkeletonDef loadDef() {
    core::Config c;
    REQUIRE(c.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json"));
    SkeletonDef def;
    REQUIRE(loadSkeletonDef(c, def));
    return def;
}

math::Vec3 modelPos(const Rig& rig, const std::vector<math::Mat4>& models, const char* name) {
    int j = rig.findJoint(name);
    REQUIRE(j >= 0);
    return math::Vec3(models[static_cast<size_t>(j)][3]);
}

} // namespace

TEST_CASE("skeleton definition") {
    SkeletonDef def = loadDef();
    CHECK(def.bones.size() == 22);
    CHECK(def.find("pelvis") == 0);
    CHECK(def.bones[0].parent == kNoParent);
    for (const char* n : {"spine_01", "spine_02", "chest", "neck", "head", "clavicle_l", "upper_arm_r", "forearm_l",
                          "hand_r", "thigh_l", "shin_r", "foot_l", "toe_r"})
        CHECK_MESSAGE(def.find(n) >= 0, n);

    SUBCASE("parents come before children") {
        for (size_t i = 1; i < def.bones.size(); ++i) CHECK(def.bones[i].parent < static_cast<int>(i));
    }
    SUBCASE("right side mirrors left in X") {
        const BoneDef& l = def.bones[static_cast<size_t>(def.find("thigh_l"))];
        const BoneDef& r = def.bones[static_cast<size_t>(def.find("thigh_r"))];
        CHECK(l.offset.x > 0.0f);  // +X is the character's left
        CHECK(math::nearlyEqual(r.offset, math::Vec3(-l.offset.x, l.offset.y, l.offset.z)));
        CHECK(r.parent == def.find("pelvis"));
        CHECK(def.bones[static_cast<size_t>(def.find("shin_r"))].parent == def.find("thigh_r"));
    }
    SUBCASE("leg length is thigh + shank ratios") { CHECK(def.legLengthRatio() == doctest::Approx(0.491f)); }
    SUBCASE("masses sum to 100 percent") {
        float total = 0.0f;
        for (const BoneDef& b : def.bones) total += b.massPercent;
        CHECK(total == doctest::Approx(100.0f).epsilon(0.005));
    }
}

TEST_CASE("rig built with ozz matches anthropometric landmarks") {
    SkeletonDef def = loadDef();
    const float h = 1.8f;
    Rig rig;
    REQUIRE(rig.build(def, h));
    REQUIRE(rig.numJoints() == 22);
    CHECK(rig.legLength() == doctest::Approx(0.491f * h));

    std::vector<math::Mat4> models(static_cast<size_t>(rig.numJoints()));
    REQUIRE(rig.localToModel(rig.restPose(), models));

    const float tol = 0.005f * h;
    CHECK(modelPos(rig, models, "thigh_l").y == doctest::Approx(0.530f * h).epsilon(tol));
    CHECK(modelPos(rig, models, "shin_l").y == doctest::Approx(0.285f * h).epsilon(tol));
    CHECK(modelPos(rig, models, "foot_r").y == doctest::Approx(0.039f * h).epsilon(tol));
    CHECK(modelPos(rig, models, "upper_arm_l").y == doctest::Approx(0.818f * h).epsilon(0.01));
    CHECK(modelPos(rig, models, "forearm_l").y == doctest::Approx(0.630f * h).epsilon(0.01));
    CHECK(modelPos(rig, models, "hand_l").y == doctest::Approx(0.485f * h).epsilon(0.01));

    SUBCASE("parents precede children in joint order") {
        for (int j = 0; j < rig.numJoints(); ++j) CHECK(rig.parent(j) < j);
    }
    SUBCASE("rebuilding at another height scales every joint") {
        Rig tall;
        REQUIRE(tall.build(def, 2.0f));
        std::vector<math::Mat4> tallModels(models.size());
        REQUIRE(tall.localToModel(tall.restPose(), tallModels));
        CHECK(math::nearlyEqual(modelPos(tall, tallModels, "head"), modelPos(rig, models, "head") * (2.0f / h), 1e-4f));
    }
    SUBCASE("local rotations propagate to children") {
        Pose p = rig.restPose();
        int knee = rig.findJoint("shin_l");
        p.locals[static_cast<size_t>(knee)].rotation = math::fromAxisAngle(math::kRight, math::kHalfPi);
        REQUIRE(rig.localToModel(p, models));
        math::Vec3 kneePos = modelPos(rig, models, "shin_l");
        math::Vec3 anklePos = modelPos(rig, models, "foot_l");
        // +90 deg about +X turns the shin's -Y direction into -Z: the ankle
        // swings back to knee height (the knee bends the right way).
        CHECK(math::distance(kneePos, anklePos) == doctest::Approx(0.246f * h));
        CHECK(anklePos.y == doctest::Approx(kneePos.y).epsilon(1e-4));
        CHECK(anklePos.z < kneePos.z);
    }
}
