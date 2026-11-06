// Tests for the body's decider: a hand target out of the arm's reach moves
// every part of its queue at once, led by where the target is (ahead: the
// clavicle; at the feet: the hips), the shares falling along the queue and a
// blocked part's share going to the rest.
#include "core/config.h"
#include "motion/body_decider.h"
#include "rig/rig.h"

#include <doctest/doctest.h>

using math::Vec3;

namespace {

struct Fixture {
    rig::SkeletonDef def;
    rig::Rig rig;
    motion::BodyDeciderJoints joints;
    motion::BodyDecider decider;
    std::vector<uint8_t> blocked;
    std::vector<math::Mat4> models;

    Fixture() {
        core::Config skel;
        REQUIRE(skel.load(CRICKET_CONFIG_DIR "/human_skeleton.json"));
        REQUIRE(rig::loadSkeletonDef(skel, def));
        REQUIRE(rig.build(def, 1.8f));
        auto chain = [&](const char* a, const char* b, const char* c, Vec3 axis) {
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
        REQUIRE(decider.init(rig, joints));
        blocked.assign(static_cast<size_t>(rig.numJoints()), 0);
        models.resize(static_cast<size_t>(rig.numJoints()));
        REQUIRE(rig.localToModel(rig.restPose(), models));
    }
    Vec3 pos(int joint) const { return Vec3(models[static_cast<size_t>(joint)][3]); }
};

// Index of the first order for `joint` of `kind`, -1 if none.
int find(const motion::BodyQueue& q, int joint, motion::PartMove kind = motion::PartMove::Turn) {
    for (int i = 0; i < q.count; ++i)
        if (q.orders[static_cast<size_t>(i)].joint == joint && q.orders[static_cast<size_t>(i)].kind == kind) return i;
    return -1;
}

bool active(const motion::BodyQueue& q, int index) { return q.orders[static_cast<size_t>(index)].active; }

} // namespace

// Sum of the shares of the orders (other than the arm's own two joints).
float trunkShares(const Fixture& f, const motion::BodyQueue& q) {
    float sum = 0.0f;
    for (int i = 0; i < q.count; ++i) {
        const motion::PartOrder& o = q.orders[static_cast<size_t>(i)];
        if (o.joint != f.joints.arms[0].start && o.joint != f.joints.arms[0].mid) sum += o.share;
    }
    return sum;
}

float shareOf(const motion::BodyQueue& q, int index) { return q.orders[static_cast<size_t>(index)].share; }

TEST_CASE("body decider: a far hand target ahead: every part moves at once, shares falling along the queue") {
    Fixture f;
    motion::BodyGoals goals;
    goals.hand[0].active = true;
    goals.hand[0].position = f.pos(f.joints.arms[0].start) + Vec3(0.0f, 0.0f, 1.5f);  // far ahead at shoulder height
    const motion::BodyDeciderSettings s;
    motion::BodyQueue q;
    f.decider.decide(f.rig.restPose(), goals, f.blocked, s, q);

    const int shoulder = find(q, f.joints.arms[0].start), clavicle = find(q, f.joints.clavicles[0]);
    const int chest = find(q, f.joints.spine[2]), spine1 = find(q, f.joints.spine[0]);
    const int tilt = find(q, f.joints.pelvis), move = find(q, f.joints.pelvis, motion::PartMove::Move);
    REQUIRE(shoulder >= 0);
    REQUIRE(clavicle >= 0);
    REQUIRE(chest >= 0);
    REQUIRE(spine1 >= 0);
    REQUIRE(tilt >= 0);
    REQUIRE(move >= 0);
    for (int i = 0; i < q.count; ++i) {
        CHECK(math::length(q.orders[static_cast<size_t>(i)].direction) == doctest::Approx(1.0f));
        CHECK(active(q, i));  // all together from the start
    }
    CHECK(shareOf(q, clavicle) > shareOf(q, chest));
    CHECK(shareOf(q, chest) > shareOf(q, spine1));
    CHECK(shareOf(q, spine1) > shareOf(q, tilt));
    CHECK(shareOf(q, tilt) > shareOf(q, move));
    CHECK(trunkShares(f, q) == doctest::Approx(1.0f));
    CHECK(q.orders[static_cast<size_t>(move)].direction.z > 0.9f);  // the hips go toward the hand

    // The clavicle blocked: its share goes to the rest; the chest now leads.
    f.blocked[static_cast<size_t>(f.joints.clavicles[0])] = 1;
    f.decider.decide(f.rig.restPose(), goals, f.blocked, s, q);
    CHECK(shareOf(q, find(q, f.joints.clavicles[0])) == 0.0f);
    CHECK_FALSE(active(q, find(q, f.joints.clavicles[0])));
    CHECK(shareOf(q, find(q, f.joints.spine[2])) > shareOf(q, find(q, f.joints.spine[0])));
    CHECK(trunkShares(f, q) == doctest::Approx(1.0f));

    // A target within reach moves the arm only.
    f.blocked.assign(f.blocked.size(), 0);
    goals.hand[0].position = f.pos(f.joints.arms[0].start) + Vec3(0.0f, -0.2f, 0.3f);
    f.decider.decide(f.rig.restPose(), goals, f.blocked, s, q);
    CHECK(find(q, f.joints.arms[0].start) >= 0);
    CHECK(find(q, f.joints.clavicles[0]) < 0);
    CHECK(find(q, f.joints.pelvis) < 0);
    CHECK(find(q, f.joints.pelvis, motion::PartMove::Move) < 0);
}

TEST_CASE("body decider: a ball at the feet: the hips carry the most, the clavicle the least") {
    Fixture f;
    motion::BodyGoals goals;
    goals.hand[0].active = true;
    goals.hand[0].position = Vec3(f.pos(f.joints.legs[0].end).x, 0.05f, 0.35f);  // on the ground ahead of the left foot
    const motion::BodyDeciderSettings s;
    motion::BodyQueue q;
    f.decider.decide(f.rig.restPose(), goals, f.blocked, s, q);

    const int tilt = find(q, f.joints.pelvis), lower = find(q, f.joints.pelvis, motion::PartMove::Move);
    const int spine1 = find(q, f.joints.spine[0]), clavicle = find(q, f.joints.clavicles[0]);
    REQUIRE(tilt >= 0);
    REQUIRE(lower >= 0);
    REQUIRE(spine1 >= 0);
    CHECK(shareOf(q, tilt) > shareOf(q, lower));
    CHECK(shareOf(q, lower) > shareOf(q, spine1));
    if (clavicle >= 0) CHECK(shareOf(q, clavicle) < shareOf(q, spine1));
    CHECK(trunkShares(f, q) == doctest::Approx(1.0f));
    CHECK(q.orders[static_cast<size_t>(lower)].direction.y < -0.5f);  // the hips go down

    // Both knees blocked: the legs cannot lower the hips; the rest share it.
    f.blocked[static_cast<size_t>(f.joints.legs[0].mid)] = 1;
    f.blocked[static_cast<size_t>(f.joints.legs[1].mid)] = 1;
    f.decider.decide(f.rig.restPose(), goals, f.blocked, s, q);
    CHECK_FALSE(active(q, find(q, f.joints.pelvis, motion::PartMove::Move)));
    CHECK(trunkShares(f, q) == doctest::Approx(1.0f));
}
