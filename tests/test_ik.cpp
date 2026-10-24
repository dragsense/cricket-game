// Tests for joint limits (hinge, swing-twist, mirroring) and the ozz-based
// two-bone IK on the fielder rig: targets are reached and joints never bend
// the wrong way.

#include "core/config.h"
#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "rig/rig.h"

#include <doctest/doctest.h>

#include <random>

using namespace ik;
using math::Quat;
using math::Vec3;

namespace {

float deg(float d) { return math::radians(d); }

Quat rotX(float d) { return math::fromAxisAngle(math::kRight, deg(d)); }
Quat rotY(float d) { return math::fromAxisAngle(math::kUp, deg(d)); }
Quat rotZ(float d) { return math::fromAxisAngle(math::kForward, deg(d)); }

JointLimit knee() {
    JointLimit l;
    l.type = LimitType::Hinge;
    l.axis = 0;
    l.twistMin = deg(-3);
    l.twistMax = deg(140);
    return l;
}

// Hip-like limit: twist about Y, swing1 = X, swing2 = Z.
JointLimit hip() {
    JointLimit l;
    l.type = LimitType::SwingTwist;
    l.axis = 1;
    l.twistMin = deg(-40);
    l.twistMax = deg(50);
    l.swing1Min = deg(-120);
    l.swing1Max = deg(30);
    l.swing2Min = deg(-30);
    l.swing2Max = deg(45);
    return l;
}

struct Fixture {
    rig::SkeletonDef def;
    rig::Rig rig;
    JointLimits limits;
    std::vector<math::Mat4> models;

    Fixture() {
        core::Config skel, joints;
        REQUIRE(skel.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json"));
        REQUIRE(joints.load(CRICKET_CONFIG_DIR "/fielder_joints.json"));
        REQUIRE(rig::loadSkeletonDef(skel, def));
        REQUIRE(rig.build(def, 1.8f));
        REQUIRE(limits.load(joints, rig));
        models.resize(static_cast<size_t>(rig.numJoints()));
    }
    Vec3 pos(const rig::Pose& p, const char* name) {
        REQUIRE(rig.localToModel(p, models));
        return Vec3(models[static_cast<size_t>(rig.findJoint(name))][3]);
    }
};

} // namespace

TEST_CASE("hinge limit") {
    JointLimit l = knee();
    CHECK(math::nearlyEqual(clampRotation(rotX(90), l), rotX(90)));
    CHECK(math::nearlyEqual(clampRotation(rotX(170), l), rotX(140)));
    CHECK(math::nearlyEqual(clampRotation(rotX(-30), l), rotX(-3)));
    SUBCASE("off-axis rotation is removed") {
        Quat q = clampRotation(rotZ(30) * rotX(60), l);
        CHECK(math::nearlyEqual(q * math::kRight, math::kRight, 1e-4f));  // pure rotation about X
    }
}

TEST_CASE("swing-twist limit") {
    JointLimit l = hip();
    SUBCASE("inside the limits is unchanged") {
        Quat q = rotX(-30) * rotZ(10) * rotY(10);
        CHECK(math::nearlyEqual(clampRotation(q, l), math::normalize(q), 1e-4f));
    }
    SUBCASE("swing past the cone is clamped to it") {
        CHECK(math::nearlyEqual(clampRotation(rotX(-150), l), rotX(-120), 1e-4f));
        CHECK(math::nearlyEqual(clampRotation(rotX(60), l), rotX(30), 1e-4f));
        CHECK(math::nearlyEqual(clampRotation(rotZ(80), l), rotZ(45), 1e-4f));
    }
    SUBCASE("twist past its range is clamped") {
        CHECK(math::nearlyEqual(clampRotation(rotY(90), l), rotY(50), 1e-4f));
        CHECK(math::nearlyEqual(clampRotation(rotY(-90), l), rotY(-40), 1e-4f));
    }
    SUBCASE("diagonal swing is held inside the ellipse") {
        Quat q = clampRotation(math::fromAxisAngle(math::normalize(Vec3(-1, 0, 1)), deg(100)), l);
        Quat swing, twist;
        math::swingTwist(q, math::kUp, swing, twist);
        float angle = 2.0f * std::acos(std::min(1.0f, std::fabs(swing.w)));
        Vec3 axis = math::normalize(Vec3(swing.x, swing.y, swing.z)) * (swing.w < 0 ? -1.0f : 1.0f);
        float a = angle * axis.x / deg(120), b = angle * axis.z / deg(45);
        CHECK(a * a + b * b <= doctest::Approx(1.0f).epsilon(1e-3));
    }
    SUBCASE("clamping is idempotent for random rotations") {
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f);
        for (int i = 0; i < 500; ++i) {
            Quat q = math::normalize(Quat(u(rng), u(rng), u(rng), u(rng)));
            Quat once = clampRotation(q, l);
            CHECK(math::nearlyEqual(clampRotation(once, l), once, 1e-3f));
        }
    }
}

TEST_CASE("mirrored limits for the right side") {
    JointLimit r = mirrored(hip());
    // Left abducts with +Z; the right leg abducts with -Z (outward is -X).
    CHECK(math::nearlyEqual(clampRotation(rotZ(-40), r), rotZ(-40), 1e-4f));
    CHECK(math::nearlyEqual(clampRotation(rotZ(40), r), rotZ(30), 1e-4f));
    // Flexion about X is the same on both sides.
    CHECK(math::nearlyEqual(clampRotation(rotX(-150), r), rotX(-120), 1e-4f));
    CHECK(math::nearlyEqual(clampRotation(rotY(-90), r), rotY(-50), 1e-4f));
}

TEST_CASE("joint limits load for every limited fielder joint") {
    Fixture f;
    CHECK(f.limits.limit(f.rig.findJoint("shin_l")).type == LimitType::Hinge);
    CHECK(f.limits.limit(f.rig.findJoint("thigh_r")).type == LimitType::SwingTwist);
    CHECK(f.limits.limit(f.rig.findJoint("pelvis")).type == LimitType::Free);
    // The right knee hinge is about X, so it keeps the same range.
    CHECK(f.limits.limit(f.rig.findJoint("shin_r")).twistMax == doctest::Approx(deg(140)));
}

TEST_CASE("two-bone IK on the fielder") {
    Fixture f;
    const ik::TwoBoneChain legL{f.rig.findJoint("thigh_l"), f.rig.findJoint("shin_l"), f.rig.findJoint("foot_l"),
                                {-1, 0, 0}};
    const ik::TwoBoneChain armL{f.rig.findJoint("upper_arm_l"), f.rig.findJoint("forearm_l"),
                                f.rig.findJoint("hand_l"), {1, 0, 0}};

    SUBCASE("leg reaches a raised target and the knee bends forward") {
        rig::Pose p = f.rig.restPose();
        Vec3 hip = f.pos(p, "thigh_l");
        Vec3 target = f.pos(p, "foot_l") + Vec3(0.0f, 0.35f, 0.15f);
        REQUIRE(ik::solveTwoBone(legL, f.models, target, Vec3(0, 0, 1), 1.0f, p));
        f.limits.apply(p);
        CHECK(math::distance(f.pos(p, "foot_l"), target) < 0.01f);
        Vec3 kneePos = f.pos(p, "shin_l");
        // The knee sits in front of the hip-ankle line.
        Vec3 mid = 0.5f * (hip + f.pos(p, "foot_l"));
        CHECK(kneePos.z > mid.z + 0.05f);
    }
    SUBCASE("unreachable target: leg straightens without hyperextending") {
        rig::Pose p = f.rig.restPose();
        Vec3 target = f.pos(p, "foot_l") + Vec3(0.0f, -0.5f, 0.0f);
        ik::solveTwoBone(legL, f.models, target, Vec3(0, 0, 1), 1.0f, p);
        f.limits.apply(p);
        Quat kneeRot = p.locals[static_cast<size_t>(legL.mid)].rotation;
        Vec3 shinDir = kneeRot * Vec3(0, -1, 0);
        CHECK(shinDir.z <= 0.06f);  // never bends forward past -3 deg
    }
    SUBCASE("arm reaches forward and the elbow bends backward") {
        rig::Pose p = f.rig.restPose();
        Vec3 shoulder = f.pos(p, "upper_arm_l");
        Vec3 target = shoulder + Vec3(0.05f, -0.15f, 0.4f);
        REQUIRE(ik::solveTwoBone(armL, f.models, target, Vec3(0.3f, 0, -1), 1.0f, p));
        f.limits.apply(p);
        CHECK(math::distance(f.pos(p, "hand_l"), target) < 0.01f);
        Vec3 elbow = f.pos(p, "forearm_l");
        Vec3 mid = 0.5f * (shoulder + f.pos(p, "hand_l"));
        CHECK(elbow.z < mid.z);
        // Elbow flexion is a negative rotation about X (forearm swings forward).
        Quat elbowRot = p.locals[static_cast<size_t>(armL.mid)].rotation;
        CHECK((elbowRot * Vec3(0, -1, 0)).z > 0.1f);
    }
}
