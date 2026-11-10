// Tests for the body's decider on its seven targets (DECISIONS 2026-11-10):
// the reach queues (a far hand target: every part at once, led by where the
// target is, the lumbopelvic split), and the plan's smoothness: support
// weights and activations blend, never switch; a pinned foot never moves;
// the hips keep to their speed and acceleration bounds whatever the targets
// do; the knee keeps its bend; a far target is reported, not snapped; a
// target let go blends back; a step through the step planner runs smooth.
#include "core/config.h"
#include "motion/body_decider.h"
#include "motion/step_planner.h"
#include "rig/rig.h"

#include <doctest/doctest.h>

#include <cmath>

using math::Vec3;

namespace {

constexpr float kDt = 1.0f / 120.0f;

struct Fixture {
    rig::SkeletonDef def;
    rig::Rig rig;
    motion::BodyDeciderJoints joints;
    motion::BodyDecider decider;
    motion::BodyDeciderSettings settings;
    motion::BodyQueue queue;
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
        models.resize(static_cast<size_t>(rig.numJoints()));
        REQUIRE(rig.localToModel(rig.restPose(), models));
    }
    Vec3 pos(int joint) const { return Vec3(models[static_cast<size_t>(joint)][3]); }
    // Standing targets: both feet pinned where they stand, the rest free.
    motion::TargetSet standing() const {
        motion::TargetSet t;
        t.dt = kDt;
        for (int i = 0; i < 2; ++i) {
            t.foot[i].mode = motion::TargetMode::Pinned;
            t.foot[i].position = pos(joints.legs[i].end);
        }
        return t;
    }
    Vec3 planPos(int joint) const { return Vec3((*queue.plan)[static_cast<size_t>(joint)][3]); }
    float kneeAngle(int leg) const {
        const math::Quat q = decider.outputPose().locals[static_cast<size_t>(joints.legs[leg].mid)].rotation;
        return math::length(math::rotationVector(q));
    }
};

// Index of the first order for `joint` of `kind`, -1 if none.
int find(const motion::BodyQueue& q, int joint, motion::PartMove kind = motion::PartMove::Turn) {
    for (int i = 0; i < q.count; ++i)
        if (q.orders[static_cast<size_t>(i)].joint == joint && q.orders[static_cast<size_t>(i)].kind == kind) return i;
    return -1;
}

bool active(const motion::BodyQueue& q, int index) { return q.orders[static_cast<size_t>(index)].active; }

// Sum of the shares of the orders of one goal.
float goalShares(const motion::BodyQueue& q, motion::Goal goal) {
    float sum = 0.0f;
    for (int i = 0; i < q.count; ++i)
        if (q.orders[static_cast<size_t>(i)].goal == goal) sum += q.orders[static_cast<size_t>(i)].share;
    return sum;
}

float shareOf(const motion::BodyQueue& q, int index) { return q.orders[static_cast<size_t>(index)].share; }

float level(Vec3 v) { return std::sqrt(v.x * v.x + v.z * v.z); }

} // namespace

TEST_CASE("body decider: a far hand target ahead: every part moves at once, shares falling along the queue") {
    Fixture f;
    motion::TargetSet t = f.standing();
    t.hand[0].mode = motion::TargetMode::Driven;
    t.hand[0].position = f.pos(f.joints.arms[0].start) + Vec3(0.0f, 0.0f, 1.5f);  // far ahead at shoulder height
    for (int i = 0; i < 24; ++i) f.decider.decide(t, f.settings, f.queue);  // the target blends in over blendTime

    const int clavicle = find(f.queue, f.joints.clavicles[0]), chest = find(f.queue, f.joints.spine[2]);
    const int spine1 = find(f.queue, f.joints.spine[0]), tilt = find(f.queue, f.joints.pelvis);
    const int move = find(f.queue, f.joints.pelvis, motion::PartMove::Move);
    REQUIRE(clavicle >= 0);
    REQUIRE(chest >= 0);
    REQUIRE(spine1 >= 0);
    REQUIRE(tilt >= 0);
    REQUIRE(move >= 0);
    CHECK(clavicle < chest);  // ahead: the shoulder girdle leads
    CHECK(chest < spine1);
    for (int i : {clavicle, chest, spine1, tilt, move}) CHECK(active(f.queue, i));
    CHECK(shareOf(f.queue, clavicle) > shareOf(f.queue, chest));
    CHECK(shareOf(f.queue, chest) > shareOf(f.queue, spine1));
    CHECK(shareOf(f.queue, move) < shareOf(f.queue, tilt));
    CHECK(goalShares(f.queue, motion::Goal::HandL) == doctest::Approx(1.0f));
    CHECK(f.queue.reach[0].driven);
}

TEST_CASE("body decider: a ball at the feet: the back and hips split by the measured lumbopelvic rhythm") {
    Fixture f;
    motion::TargetSet t = f.standing();
    t.hand[0].mode = motion::TargetMode::Driven;
    t.hand[0].position = Vec3(f.pos(f.joints.legs[0].end).x, 0.05f, 0.35f);  // on the ground ahead of the left foot
    for (int i = 0; i < 24; ++i) f.decider.decide(t, f.settings, f.queue);

    const int tilt = find(f.queue, f.joints.pelvis), lower = find(f.queue, f.joints.pelvis, motion::PartMove::Move);
    const int spine1 = find(f.queue, f.joints.spine[0]), clavicle = find(f.queue, f.joints.clavicles[0]);
    REQUIRE(tilt >= 0);
    REQUIRE(lower >= 0);
    REQUIRE(spine1 >= 0);
    // Standing upright (flexion < 30 deg): the back moves 1.9 times the hips (Esola 1996).
    CHECK(shareOf(f.queue, spine1) / shareOf(f.queue, tilt) == doctest::Approx(1.9f));
    if (clavicle >= 0) CHECK(shareOf(f.queue, clavicle) < shareOf(f.queue, tilt));  // last in the queue: the least
    CHECK(goalShares(f.queue, motion::Goal::HandL) == doctest::Approx(1.0f));
    CHECK(f.queue.orders[static_cast<size_t>(lower)].direction.y < -0.5f);  // the hips go down
}

TEST_CASE("body decider: activation and support weights blend, never switch") {
    Fixture f;
    motion::TargetSet t = f.standing();
    f.decider.decide(t, f.settings, f.queue);
    CHECK(f.queue.supportWeight[0] == doctest::Approx(1.0f));
    // The left foot goes on its way (a step's start): its weight leaves it
    // at 1 / supportBlendTime, the hips' reference moves over the right foot.
    t.foot[0].mode = motion::TargetMode::Driven;
    float last = 1.0f;
    const float step = kDt / f.settings.supportBlendTime;
    int ticks = 0;
    for (; ticks < 60 && f.queue.supportWeight[0] > 0.0f; ++ticks) {
        f.decider.decide(t, f.settings, f.queue);
        CHECK(last - f.queue.supportWeight[0] <= step + 1e-5f);
        CHECK(f.queue.supportWeight[0] >= 0.0f);
        last = f.queue.supportWeight[0];
    }
    CHECK(ticks == doctest::Approx(f.settings.supportBlendTime / kDt).epsilon(0.1));
    CHECK(f.queue.supportWeight[1] == doctest::Approx(1.0f));
    const Vec3 rightFoot = f.pos(f.joints.legs[1].end);
    const float k = f.settings.passiveHips;
    CHECK(f.queue.hipsReference.x == doctest::Approx((1.0f - k) * rightFoot.x).epsilon(0.05));
}

TEST_CASE("body decider: the hips never exceed their speed and acceleration bounds, whatever the targets do") {
    Fixture f;
    f.settings.passiveHips = 0.0f;  // the hips go fully over the support
    motion::TargetSet t = f.standing();
    // Two ticks of history (the hips already move as they settle to the
    // legs' reach), so the first measured change is a real one.
    f.decider.decide(t, f.settings, f.queue);
    Vec3 p0 = f.planPos(f.joints.pelvis);
    f.decider.decide(t, f.settings, f.queue);
    Vec3 p1 = f.planPos(f.joints.pelvis);
    float worstSpeed = 0.0f, worstAccel = 0.0f;
    auto measure = [&] {
        const Vec3 p2 = f.planPos(f.joints.pelvis);
        const Vec3 v = (p2 - p1) / kDt, a = (p2 - 2.0f * p1 + p0) / (kDt * kDt);
        worstSpeed = std::max(worstSpeed, math::length(v));
        worstAccel = std::max(worstAccel, math::length(a));
        p0 = p1;
        p1 = p2;
    };
    // A sudden one-foot stance (the left foot driven at once), then at once
    // a step's landing 0.5 m ahead, then both feet pinned again.
    t.foot[0].mode = motion::TargetMode::Driven;
    t.foot[0].position = f.pos(f.joints.legs[0].end) + Vec3(0.0f, 0.1f, 0.0f);
    for (int i = 0; i < 60; ++i) {
        f.decider.decide(t, f.settings, f.queue);
        measure();
    }
    t.foot[0].mode = motion::TargetMode::Pinned;
    t.foot[0].position = f.pos(f.joints.legs[0].end) + Vec3(0.0f, 0.0f, 0.5f);
    for (int i = 0; i < 120; ++i) {
        f.decider.decide(t, f.settings, f.queue);
        measure();
    }
    MESSAGE("hips: worst speed ", worstSpeed, " m/s, worst acceleration ", worstAccel, " m/s^2");
    CHECK(worstSpeed <= f.settings.hipsMaxSpeed + 1e-3f);
    CHECK(worstAccel <= f.settings.hipsMaxAccel * 1.05f + 0.5f);  // the measured second difference
    // Settled between the feet, within reach of both.
    const Vec3 hips = f.planPos(f.joints.pelvis);
    const float mid = 0.5f * (f.pos(f.joints.legs[1].end).z + f.pos(f.joints.legs[0].end).z + 0.5f);
    CHECK(hips.z == doctest::Approx(mid).epsilon(0.1));
    CHECK(f.queue.reach[2].reached);
    CHECK(f.queue.reach[3].reached);
}

TEST_CASE("body decider: a pinned foot never moves in the plan while the other steps") {
    Fixture f;
    motion::TargetSet t = f.standing();
    const Vec3 rightAnkle = f.pos(f.joints.legs[1].end);
    const Vec3 from = f.pos(f.joints.legs[0].end);
    motion::StepPlanner planner;
    motion::StepPlannerSettings s;
    s.duration = 0.6f;
    s.liftAt = 0.25f;
    s.height = 0.2f;
    planner.start(0, from, from + Vec3(0.0f, 0.0f, 0.4f));
    float worstSlide = 0.0f, worstKnee = 0.0f, lowestKnee = 10.0f;
    Vec3 lastTarget = from;
    float worstTargetJump = 0.0f;
    bool landed = false;
    for (int i = 0; i < 120 && !landed; ++i) {
        t.foot[0].mode = motion::TargetMode::Pinned;
        t.foot[0].position = from;
        planner.tick(kDt, s, t);
        landed = planner.landed();
        worstTargetJump = std::max(worstTargetJump, math::distance(t.foot[0].position, lastTarget));
        lastTarget = t.foot[0].position;
        f.decider.decide(t, f.settings, f.queue);
        worstSlide = std::max(worstSlide, math::distance(f.planPos(f.joints.legs[1].end), rightAnkle));
        lowestKnee = std::min(lowestKnee, f.kneeAngle(1));
        worstKnee = std::max(worstKnee, f.kneeAngle(1));
    }
    REQUIRE(landed);
    // The hips then settle over the weighted feet (bounded speed): a few
    // moments on, the leg reaches the landed foot's spot.
    for (int i = 0; i < 120; ++i) {
        t.foot[0].mode = motion::TargetMode::Pinned;
        t.foot[0].position = from + Vec3(0.0f, 0.0f, 0.4f);
        f.decider.decide(t, f.settings, f.queue);
    }
    MESSAGE("stance ankle moved ", 1000.0f * worstSlide, " mm; stance knee ", math::degrees(lowestKnee), "-",
            math::degrees(worstKnee), " deg; foot target's biggest jump ", 1000.0f * worstTargetJump, " mm");
    CHECK(worstSlide < 0.002f);        // the pinned foot stays (2 mm: the two-bone solve's precision)
    CHECK(lowestKnee > math::radians(3.0f));  // never straight
    CHECK(worstTargetJump < 0.02f);    // the planner's path has no jump
    // Landed: pinned on the spot, the leg reaches it.
    CHECK(math::distance(f.planPos(f.joints.legs[0].end), from + Vec3(0.0f, 0.0f, 0.4f)) < 0.01f);
}

TEST_CASE("body decider: a foot target out of reach is reported short, the leg keeps its bend") {
    Fixture f;
    motion::TargetSet t = f.standing();
    t.foot[0].mode = motion::TargetMode::Driven;
    t.foot[0].position = f.pos(f.joints.legs[0].end) + Vec3(0.0f, 0.3f, 1.5f);  // far ahead, lifted
    for (int i = 0; i < 240; ++i) f.decider.decide(t, f.settings, f.queue);
    CHECK(f.queue.reach[2].driven);
    CHECK_FALSE(f.queue.reach[2].reached);
    CHECK(f.queue.reach[2].shortfall > 0.1f);
    CHECK(f.queue.reach[2].direction.z > 0.5f);
    CHECK(f.kneeAngle(0) > math::radians(3.0f));
    // The stance foot stayed, the plan's hips stayed within the root's bound.
    CHECK(math::distance(f.planPos(f.joints.legs[1].end), f.pos(f.joints.legs[1].end)) < 0.002f);
    CHECK(level(f.queue.pelvisOffset) <= f.settings.hipsOffsetMax + 1e-3f);
}

TEST_CASE("body decider: a hand let go blends back to rest without a jump") {
    Fixture f;
    motion::TargetSet t = f.standing();
    t.hand[1].mode = motion::TargetMode::Driven;
    t.hand[1].position = f.pos(f.joints.arms[1].start) + Vec3(0.0f, 0.1f, 0.5f);
    for (int i = 0; i < 120; ++i) f.decider.decide(t, f.settings, f.queue);
    const Vec3 reached = f.planPos(f.joints.arms[1].end);
    CHECK(math::distance(reached, t.hand[1].position) < 0.02f);
    t.hand[1].mode = motion::TargetMode::Free;
    Vec3 last = reached;
    float worstStep = 0.0f;
    for (int i = 0; i < 120; ++i) {
        f.decider.decide(t, f.settings, f.queue);
        const Vec3 now = f.planPos(f.joints.arms[1].end);
        worstStep = std::max(worstStep, math::distance(now, last));
        last = now;
    }
    MESSAGE("hand let go: biggest move per tick ", 1000.0f * worstStep, " mm");
    // A limb's joint rate bounds it: the shoulder and the elbow together
    // move the wrist by at most rate x dt x (the arm's length twice over).
    const float arm = math::distance(f.pos(f.joints.arms[1].start), f.pos(f.joints.arms[1].end));
    CHECK(worstStep <= f.settings.limbMaxRate * kDt * 2.0f * arm);
    CHECK(math::distance(last, f.pos(f.joints.arms[1].end)) < 0.05f);  // back at rest
}
