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

TEST_CASE("human body: with no muscles it folds under gravity, inside its real joint ranges, onto the ground") {
    Scene s;
    REQUIRE(makeScene(s));
    const agents::HumanBody& body = s.human.humanBody();
    const float startHeight = body.height(s.world);
    float worstOutside = 0.0f, lowestPart = 9.0f;
    std::string worstJoint;
    for (int i = 0; i < 3 * 120; ++i) {
        s.world.step(kTick);
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
    MESSAGE("fold: centre of mass ", startHeight, " -> ", endHeight, " m; lowest part centre ", lowestPart,
            " m; worst joint past its range ", worstOutside, " rad (", worstJoint, ")");
    CHECK(endHeight < startHeight - 0.4f);  // it came down
    CHECK(lowestPart > -0.01f);             // the ground held every part
    // Joints stay near their real ranges: with no muscles the whole body
    // crashes onto them; Jolt's solver (jolt.jointVelocitySteps 20 / 5) lets
    // them overshoot about 0.21 rad at the hardest moment.
    CHECK(worstOutside < 0.25f);
}
