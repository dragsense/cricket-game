// Whole-fielder checks: runs the real fielder from config and measures how the
// physics body moves (steps, the limb test).

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

} // namespace

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

} // namespace

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
    float worstPlan = 0.0f, worstReal = 0.0f, worstGap = 0.0f, endGap = 0.0f;
    int againstPlan = 0, againstReal = 0, ticks = 0, stepTicks = 0;
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
            endGap = math::distance(r, p);  // the real end off the plan's
            worstGap = std::max(worstGap, endGap);
        }
        ++ticks;
    }
    std::printf("%-22s against goal: plan %3d ticks (worst %5.2f m/s), real %3d ticks (worst %5.2f m/s), "
                "final miss plan %4.1f cm, stepping %d ticks | real off plan: worst %4.1f cm, end %4.1f cm\n",
                name, againstPlan, worstPlan, againstReal, worstReal, 100.0f * math::distance(lastPlan, goal), stepTicks,
                100.0f * worstGap, 100.0f * endGap);
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


// Measurement (on demand): the hand's chain in time. A hand goal set at once
// (behind, across, far ahead); per joint of the chain (pelvis .. upper arm)
// the plan's turn of its own (local) from standing: the tick it starts (past
// 1 deg) and reaches 80% of where it ends, and that end.
namespace {
void chainTiming(math::Vec3 local, const char* name, math::Vec3 second = {}, int pressAt = -1) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const char* names[6] = {"pelvis", "spine_01", "spine_02", "chest", "clavicle_l", "upper_arm_l"};
    int joint[6];
    for (int k = 0; k < 6; ++k) joint[k] = h.rig().findJoint(names[k]);
    auto localRot = [&](const std::vector<math::Mat4>& m, int j) {
        const int parent = h.rig().parent(j);
        const math::Quat r = math::toTransform(m[static_cast<size_t>(j)]).rotation;
        return parent >= 0 ? math::inverse(math::toTransform(m[static_cast<size_t>(parent)]).rotation) * r : r;
    };
    math::Quat start[6];
    for (int k = 0; k < 6; ++k) start[k] = localRot(*h.bodyQueue().plan, joint[k]);
    // Which way the pelvis and each foot turned about the vertical (deg, +
    // to the body's left), in the plan, from before the goal.
    const int yawJoint[3] = {joint[0], h.rig().findJoint("foot_l"), h.rig().findJoint("foot_r")};
    auto modelRot = [&](int j) { return math::toTransform((*h.bodyQueue().plan)[static_cast<size_t>(j)]).rotation; };
    math::Quat yawStart[3];
    for (int k = 0; k < 3; ++k) yawStart[k] = modelRot(yawJoint[k]);
    auto yawNow = [&](int k) {
        const math::Vec3 f = (modelRot(yawJoint[k]) * math::inverse(yawStart[k])) * math::kForward;
        return math::degrees(std::atan2(f.x, f.z));
    };
    constexpr int kTicks = 120;
    std::vector<float> turn(6 * kTicks);
    math::Vec3 goal = h.root().transformPoint(local);
    for (int i = 0; i < kTicks; ++i) {
        if (i == pressAt) {
            // A second press: its turns counted from here.
            goal = h.root().transformPoint(second);
            for (int k = 0; k < 6; ++k) start[k] = localRot(*h.bodyQueue().plan, joint[k]);
        }
        h.setPoseHand(true, goal);
        h.tick(kDt);
        world.step(kDt);
        for (int k = 0; k < 6; ++k)
            turn[static_cast<size_t>(k * kTicks + i)] =
                math::degrees(math::angleBetween(start[k], localRot(*h.bodyQueue().plan, joint[k])));
    }
    std::printf("%s (miss %4.1f cm):\n", name, 100.0f * math::distance(h.wrist(true, *h.bodyQueue().plan, h.root()), goal));
    for (int k = 0; k < 6; ++k) {
        const float* t = &turn[static_cast<size_t>(k * kTicks)];
        const float end = t[kTicks - 1];
        int begin = -1, most = -1;
        for (int i = std::max(pressAt, 0); i < kTicks; ++i) {
            if (begin < 0 && t[i] > 1.0f) begin = i;
            if (most < 0 && end > 1.0f && t[i] >= 0.8f * end) most = i;
        }
        std::printf("  %-11s starts tick %4d, 80%% at tick %4d, ends %6.1f deg\n", names[k], begin, most, end);
    }
    std::printf("  yaw (+ left): pelvis %6.1f, foot L %6.1f, foot R %6.1f deg\n", yawNow(0), yawNow(1), yawNow(2));
}
} // namespace

TEST_CASE("measure: the hand's chain in time" * doctest::skip()) {
    chainTiming(math::Vec3(0.25f, 1.2f, -0.5f), "hand L behind");
    chainTiming(math::Vec3(-0.5f, 1.2f, 0.3f), "hand L across");
    chainTiming(math::Vec3(0.2f, 1.0f, 0.9f), "hand L far ahead");
    chainTiming(math::Vec3(0.45f, 1.0f, 0.2f), "hand L near side");
    chainTiming(math::Vec3(0.2f, 0.5f, 0.9f), "hand L far ahead low");
    chainTiming(math::Vec3(-0.6f, 0.6f, 0.4f), "hand L across low");
    chainTiming(math::Vec3(0.3f, 0.8f, -0.7f), "hand L far behind");
    chainTiming(math::Vec3(-0.5f, 1.2f, 0.3f), "hand L across, again at tick 17", math::Vec3(-0.6f, 1.2f, 0.2f), 17);
}

// Measurement (on demand): a foot goal behind; the hip's extension the plan
// asks for and the real body's, in the pelvis's frame (+: thigh behind).
TEST_CASE("measure: hip extension for a foot behind" * doctest::skip()) {
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
    const int pelvis = h.rig().findJoint("pelvis"), thigh = h.rig().findJoint("thigh_l"), shin = h.rig().findJoint("shin_l");
    auto extension = [&](const std::vector<math::Mat4>& m) {
        const math::Quat inv = math::inverse(math::toTransform(m[static_cast<size_t>(pelvis)]).rotation);
        const math::Vec3 d = inv * (math::Vec3(m[static_cast<size_t>(shin)][3]) - math::Vec3(m[static_cast<size_t>(thigh)][3]));
        return math::degrees(std::atan2(-d.z, -d.y));
    };
    for (float back : {0.2f, 0.3f, 0.4f, 0.5f}) {
        h.resetPhysicsBody();
        for (int i = 0; i < 30; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 goal = h.root().transformPoint(math::Vec3(0.1f, 0.1f, -back));
        for (int i = 0; i < 360; ++i) {
            h.setPoseFoot(motion::Foot::Left, goal);
            h.tick(kDt);
            world.step(kDt);
        }
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const bool haveReal = h.physicalModels(world, 1.0f, real);
        std::printf("foot %.1f m behind: hip extension plan %5.1f deg, real %5.1f deg; knee plan %5.1f, real %5.1f deg; "
                    "ankle miss plan %4.1f cm, real %4.1f cm\n",
                    back, extension(p), haveReal ? extension(real) : 0.0f, flexionDeg(p, h, "thigh_l", "shin_l", "foot_l"),
                    haveReal ? flexionDeg(real, h, "thigh_l", "shin_l", "foot_l") : 0.0f,
                    100.0f * math::distance(h.ankle(true, p, h.root()), goal),
                    haveReal ? 100.0f * math::distance(h.ankle(true, real, h.root()), goal) : 0.0f);
    }
}

// Measurement (on demand): a foot goal behind, over time: does the real foot
// catch the ground (toe and ankle heights), and where are the hips (real vs plan)?
TEST_CASE("measure: foot behind over time" * doctest::skip()) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    const int pelvis = h.rig().findJoint("pelvis"), thigh = h.rig().findJoint("thigh_l"), shin = h.rig().findJoint("shin_l");
    const int foot = h.rig().findJoint("foot_l"), toe = h.rig().findJoint("toe_l");
    auto extension = [&](const std::vector<math::Mat4>& m) {
        const math::Quat inv = math::inverse(math::toTransform(m[static_cast<size_t>(pelvis)]).rotation);
        const math::Vec3 d = inv * (math::Vec3(m[static_cast<size_t>(shin)][3]) - math::Vec3(m[static_cast<size_t>(thigh)][3]));
        return math::degrees(std::atan2(-d.z, -d.y));
    };
    auto worldY = [&](const std::vector<math::Mat4>& m, int j) { return h.root().transformPoint(math::Vec3(m[static_cast<size_t>(j)][3])).y; };
    for (float back : {0.4f, 0.5f}) {
        h.resetPhysicsBody();
        for (int i = 0; i < 30; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 goal = h.root().transformPoint(math::Vec3(0.1f, 0.1f, -back));
        std::printf("foot %.1f m behind (goal ankle height 0.10 m):\n", back);
        for (int i = 1; i <= 360; ++i) {
            h.setPoseFoot(motion::Foot::Left, goal);
            h.tick(kDt);
            world.step(kDt);
            if (i % 30 != 0 || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            std::printf("  t %.2f s: hip ext plan %5.1f real %5.1f deg | pelvis y plan %.3f real %.3f m | "
                        "ankle y plan %.3f real %.3f | toe y plan %.3f real %.3f m | ankle miss %4.1f cm\n",
                        i * kDt, extension(p), extension(real), worldY(p, pelvis), worldY(real, pelvis), worldY(p, foot),
                        worldY(real, foot), worldY(p, toe), worldY(real, toe),
                        100.0f * math::distance(h.ankle(true, real, h.root()), goal));
        }
    }
}
// Measurement (on demand): the limb test's hand pushed far ahead (0.1 m per
// press every 0.3 s, as the arrow keys do), then held: every balance step
// (time, foot, where it landed along the way) and whether the steps stop.

// Measurement (on demand): the limb test's hips moved once (one press) by a
// set way; the sync formula: the step's foot lands when the hips arrive, the
// hips end between the feet. Prints when the plan's hips reach the goal and
// when each step starts and lands, and how far the foot and the hips went.

// Measurement (on demand): the limb test's left foot moved once (one press)
// in the air; by the foot's progress along its path: the plan's hips toward
// the standing foot, the pelvis's drop on the lifted side and its turn, and
// the plan's centre of mass from the standing ankle (part 2 of the sync
// formula: an active foot moves the hips).
TEST_CASE("measure: foot pressed once, the hips with it" * doctest::skip()) {
    struct Case {
        const char* name;
        math::Vec3 way;  // character space from the ankle: +Z forward, +X left, +Y up
    };
    for (const Case& c : {Case{"up 0.15", {0.0f, 0.15f, 0.0f}}, Case{"fwd 0.30 up 0.15", {0.0f, 0.15f, 0.30f}},
                          Case{"back 0.30 up 0.15", {0.0f, 0.15f, -0.30f}}, Case{"out 0.10 up 0.15", {0.10f, 0.15f, 0.0f}}, Case{"out 0.20 up 0.15", {0.20f, 0.15f, 0.0f}},
                          Case{"out 0.35 up 0.15", {0.35f, 0.15f, 0.0f}}}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
        const math::Vec3 goal = a0 + h.root().transformDirection(c.way);
        const math::Vec3 stand = h.ankle(false, *h.bodyQueue().plan, h.root());  // the right foot stands
        const math::Vec3 hips0 = h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3]));
        const math::Vec3 left = h.root().transformDirection(math::Vec3(1, 0, 0)), fwd = h.root().transformDirection(math::kForward);
        std::printf("--- foot L %s (hips start %.1f cm left of the standing ankle)\n", c.name, 100.0f * math::dot(hips0 - stand, left));
        float lastShare = -1.0f;
        for (int i = 0; i < 120; ++i) {
            h.setPoseFoot(motion::Foot::Left, goal);
            h.tick(kDt);
            world.step(kDt);
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            const math::Vec3 a = h.ankle(true, p, h.root());
            const float share = math::length(a - a0) / std::max(math::length(goal - a0), 1e-3f);
            if (share - lastShare < 0.1f && i != 119 && !(i % 12 == 0)) continue;
            lastShare = share;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
            const math::Vec3 side = math::normalize(math::Vec3(p[0][0]));  // the pelvis's left axis (character space)
            const math::Vec3 sideW = h.root().transformDirection(side);
            const float drop = math::degrees(std::asin(std::clamp(-sideW.y, -1.0f, 1.0f)));  // + left side lower
            const float yaw = math::degrees(std::atan2(math::dot(sideW, fwd), math::dot(sideW, left)));  // + left hip forward
            const math::Vec3 com = h.root().transformPoint(h.decider().planCentreOfMass());
            std::printf("  %.2f s foot %3.0f %% | hips toward the standing foot %5.1f cm, fwd %5.1f cm, down %4.1f cm | "
                        "pelvis left side down %4.1f deg, left hip fwd %5.1f deg | CoM from the standing ankle: left %5.1f, fwd %5.1f cm\n",
                        i * kDt, 100.0f * share, 100.0f * math::dot(hips0 - hips, left), 100.0f * math::dot(hips - hips0, fwd),
                        100.0f * (hips0.y - hips.y), drop, yaw, 100.0f * math::dot(com - stand, left),
                        100.0f * math::dot(com - stand, fwd));
        }
        std::printf("  dropped %s\n", h.supported() ? "no" : "yes");
    }
}

// Measurement (on demand): the limb test's hips moved press by press, as in
// the game (engine.json debug.limbTestStep 0.1 m, a press each 0.5 s).
TEST_CASE("measure: hips pressed 0.1 m at a time" * doctest::skip()) {
    struct Case {
        const char* name;
        math::Vec3 press;
        int presses;
    };
    for (const Case& c : {Case{"fwd", {0.0f, 0.0f, 0.1f}, 3}, Case{"left", {0.1f, 0.0f, 0.0f}, 3},
                          Case{"back", {0.0f, 0.0f, -0.1f}, 3}}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 hips0 = h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3]));
        const math::Vec3 dir = math::normalize(h.root().transformDirection(c.press));
        math::Vec3 goal = hips0;
        std::printf("--- hips %s, %d presses of 0.1 m\n", c.name, c.presses);
        for (int i = 0; i < 60 * (c.presses + 2); ++i) {
            const float time = i * kDt;
            if (i % 60 == 0 && i / 60 < c.presses) {
                goal += h.root().transformDirection(c.press);
                std::printf("  %.2f s press %d\n", time, i / 60 + 1);
            }
            h.setPoseGoal(motion::Goal::Hips, goal, math::identityQuat(), false);
            h.tick(kDt);
            world.step(kDt);
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
            for (int f = 0; f < 2; ++f)
                if (h.bodyQueue().landed[f])
                    std::printf("  %.2f s step %s lands | hips %.1f cm along\n", time, f == 0 ? "L" : "R",
                                100.0f * math::dot(hips - hips0, dir));
            if (i % 60 == 59) {
                const math::Vec3 l = h.ankle(true, p, h.root()), r = h.ankle(false, p, h.root());
                std::printf("  %.2f s hips %.1f cm along, %.1f cm from the feet's middle; capture point outside %.1f cm\n",
                            time, 100.0f * math::dot(hips - hips0, dir), 100.0f * math::dot(hips - 0.5f * (l + r), dir),
                            100.0f * h.bodyQueue().step.outside);
            }
        }
    }
}

// Measurement (on demand): the limb test's left hand moved press by press, as
// in the game (0.1 m a press, each 0.5 s): the steps it causes and why (the
// balance step's advice when one starts).

// Measurement (on demand): the user's report (2026-11-15): the left hand
// pressed down, then forward (0.1 m a press, as in the game): steps go back
// and forth. Prints each step's cause and the plan's hips, centre of mass
// and hand through it.

// Measurement (on demand): the user's report (2026-11-15): the left foot sent
// out to the side, the real leg goes forward. The plan's and the real ankle,
// and the thigh's angles in the pelvis's frame (flexion forward +, abduction
// out +), plan against real.
TEST_CASE("measure: foot out to the side, plan against real" * doctest::skip()) {
    for (float out : {0.2f, 0.35f, 0.6f}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
        const math::Vec3 goal = a0 + h.root().transformDirection(math::Vec3(out, 0.15f, 0.0f));
        const int thigh = h.rig().findJoint("thigh_l"), shin = h.rig().findJoint("shin_l");
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        std::printf("--- foot L out %.2f, up 0.15 (character space: x left, z forward, from the ankle's start)\n", out);
        for (int i = 0; i < 240; ++i) {
            h.setPoseFoot(motion::Foot::Left, goal);
            h.tick(kDt);
            world.step(kDt);
            if (i % 30 != 29 || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            auto angles = [&](const std::vector<math::Mat4>& m, float& flex, float& abd) {
                const math::Vec3 hip(m[static_cast<size_t>(thigh)][3]), knee(m[static_cast<size_t>(shin)][3]);
                const math::Mat4& pel = m[0];
                const math::Vec3 x = math::normalize(math::Vec3(pel[0])), y = math::normalize(math::Vec3(pel[1])),
                                 z = math::normalize(math::Vec3(pel[2]));
                const math::Vec3 d = math::normalize(knee - hip);
                flex = math::degrees(std::atan2(math::dot(d, z), -math::dot(d, y)));
                abd = math::degrees(std::atan2(math::dot(d, x), -math::dot(d, y)));
            };
            float pf, pa, rf, ra;
            angles(p, pf, pa);
            angles(real, rf, ra);
            const math::Vec3 pl = math::inverse(h.root()).transformPoint(h.ankle(true, p, h.root())) -
                                  math::inverse(h.root()).transformPoint(a0);
            const math::Vec3 rl = math::inverse(h.root()).transformPoint(h.ankle(true, real, h.root())) -
                                  math::inverse(h.root()).transformPoint(a0);
            std::printf("  %.2f s ankle plan x %5.1f z %5.1f y %5.1f | real x %5.1f z %5.1f y %5.1f cm | thigh plan flex %5.1f abd "
                        "%5.1f | real flex %5.1f abd %5.1f deg\n",
                        (i + 1) * kDt, 100 * pl.x, 100 * pl.z, 100 * pl.y, 100 * rl.x, 100 * rl.z, 100 * rl.y, pf, pa, rf, ra);
        }
    }
}

// Measurement (on demand): the user's report (2026-11-15): the limb test's
// left foot moved forward on the ground (0.1 m a press): it should step there
// (a pinned foot never slides), the hips going between the feet with it.

// Measurement (on demand, 2026-11-21): W held (the hips pushed forward), then
// let go. Per step: when it lands, its length (plan), the real ankle off the
// plan's pinned spot at landing and the most it is off (level) through that
// foot's stance: the slide the user sees. Prints only.

// Measurement (on demand, 2026-11-16, muscles A1): each muscle's stiffness as
// built (K = inertia beyond the joint x (1 / activationTime)^2, damping 2 sqrt(K I))
// against the research values (BIOMECHANICS 14: K = relaxed + slope x |torque
// held|), at the torque the gravity hold gives standing, with the left hand held
// 0.5 m ahead at shoulder height, and with the left foot held 15 cm up. Also
// the natural frequency sqrt(K / I) each gives. Prints only.
namespace {
struct ResearchK {
    const char* bone;
    float relaxed, slope;  // N m/rad, N m/rad per N m
};
// Gomi & Osu 1998 (shoulder, elbow), Huang 2020 (hip), Granata 2002 (knee,
// extension slope), Cholewicki 2010 (trunk, whole about L4-L5, no slope),
// Loram & Lakie 2002 / Casadio 2005 (ankle 0.64-0.91 mgh: 0.8 here).
constexpr ResearchK kResearch[] = {{"upper_arm", 10.80f, 3.18f}, {"forearm", 8.67f, 6.18f}, {"thigh", 169.4f, 5.95f},
                                   {"shin", 75.0f, 6.6f},        {"spine_01", 590.0f, 0.0f}, {"foot", -0.8f, 0.0f}};

void muscleTable(agents::Human& h, physics::PhysicsWorld& world, const char* name) {
    const agents::HumanBody& body = h.humanBody();
    const auto& parts = body.parts();
    core::Config joints;
    REQUIRE(joints.load(CRICKET_CONFIG_DIR "/human_joints.json"));
    const float t = joints.getFloat("muscles.activationTime", 0.03f);
    const float w2 = 1.0f / (t * t);
    // Mass and centre beyond each part (as holdWeight), and inertia about its pivot.
    std::vector<float> mass(parts.size());
    std::vector<math::Vec3> moment(parts.size());
    for (size_t i = 0; i < parts.size(); ++i) {
        mass[i] = parts[i].mass;
        moment[i] = world.transform(parts[i].body).position * parts[i].mass;
    }
    for (size_t i = parts.size(); i-- > 0;)
        if (parts[i].parent >= 0) {
            mass[static_cast<size_t>(parts[i].parent)] += mass[i];
            moment[static_cast<size_t>(parts[i].parent)] += moment[i];
        }
    float bodyMass = 0.0f, comY = 0.0f, groundY = h.root().position.y;
    for (const auto& p : parts) {
        bodyMass += p.mass;
        comY += p.mass * world.transform(p.body).position.y;
    }
    comY = comY / bodyMass - groundY;
    const float mgh = bodyMass * 9.81f * comY;
    std::printf("--- %s (mgh %.0f N m/rad, CoM %.2f m up)\n", name, mgh, comY);
    std::printf("  %-12s %6s %7s | %8s %8s %6s | %7s | %8s %6s %6s\n", "joint", "kg", "I", "K now", "c now", "w now",
                "|hold|", "K resch", "w resch", "ratio");
    for (size_t i = 0; i < parts.size(); ++i) {
        const auto& p = parts[i];
        if (p.parent < 0 || !p.muscle) continue;
        const math::Vec3 pivot = body.boneWorld(world, static_cast<int>(i)).position;
        float inertia = 0.0f;
        for (size_t k = 0; k < parts.size(); ++k)
            for (int a = static_cast<int>(k); a >= 0; a = parts[static_cast<size_t>(a)].parent)
                if (a == static_cast<int>(i)) {
                    inertia += parts[k].mass * math::length2(world.transform(parts[k].body).position - pivot);
                    break;
                }
        const math::Vec3 centre = moment[i] / mass[i];
        const float hold = math::length(math::cross(centre - pivot, math::Vec3(0.0f, -9.81f, 0.0f) * mass[i]));
        const float k = inertia * w2, c = 2.0f * std::sqrt(k * inertia);
        std::string bone = h.rig().jointName(p.joint);
        if (bone.size() > 2 && bone[bone.size() - 2] == '_') bone = bone.substr(0, bone.size() - 2);
        float kr = -1.0f;
        for (const ResearchK& r : kResearch)
            if (bone == r.bone) kr = r.relaxed < 0.0f ? -r.relaxed * mgh : r.relaxed + r.slope * hold;
        if (kr > 0.0f)
            std::printf("  %-12s %6.2f %7.3f | %8.0f %8.1f %6.1f | %7.1f | %8.0f %6.1f %6.2f\n",
                        h.rig().jointName(p.joint).c_str(), mass[i], inertia, k, c, std::sqrt(k / inertia), hold, kr,
                        std::sqrt(kr / inertia), kr / k);
        else
            std::printf("  %-12s %6.2f %7.3f | %8.0f %8.1f %6.1f | %7.1f |      gap\n", h.rig().jointName(p.joint).c_str(),
                        mass[i], inertia, k, c, std::sqrt(k / inertia), hold);
    }
}
} // namespace

TEST_CASE("measure: muscle stiffness, built against research" * doctest::skip()) {
    for (int c = 0; c < 3; ++c) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
        const math::Vec3 shoulder = h.root().transformPoint(
            math::Vec3((*h.bodyQueue().plan)[static_cast<size_t>(h.rig().findJoint("upper_arm_l"))][3]));
        for (int i = 0; i < 240 && c > 0; ++i) {
            if (c == 1) h.setPoseHand(true, shoulder + h.root().transformDirection(math::Vec3(0.0f, 0.0f, 0.5f)));
            else h.setPoseFoot(motion::Foot::Left, a0 + math::Vec3(0.0f, 0.15f, 0.0f));
            h.tick(kDt);
            world.step(kDt);
        }
        muscleTable(h, world, c == 0 ? "standing" : c == 1 ? "hand L 0.5 m ahead at shoulder height" : "foot L 15 cm up");
    }
}

// Measurement (on demand): the limb test's left foot held up, then put down
// on the ground elsewhere: it lands there by a step (never jumps there).

// Measurement (on demand): who carries the body's weight, the stance legs or
// the capsule's support spring (Plan A step 1). Standing, one foot held up,
// a hand at the toes (squat); over the last second: the spring's force on the
// pelvis (character axes: +x left, +y up, +z forward) and torque, the real
// pelvis against the plan's, the knees real against plan, and the stance
// legs' holds.
namespace {
void weightCase(int which, const char* name) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
    const math::Vec3 toes = h.root().transformPoint(math::Vec3(0.15f, 0.05f, 0.25f));
    const int pelvis = h.rig().findJoint("pelvis");
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    constexpr int kTicks = 360, kLast = 120;  // 3 s, the last 1 s averaged
    math::Vec3 force(0.0f);
    float torque = 0.0f, dropY = 0.0f, offLevel = 0.0f;
    math::Vec3 rootStart(0.0f);
    const int legJoint[3] = {h.rig().findJoint("thigh_l"), h.rig().findJoint("shin_l"), h.rig().findJoint("foot_l")};
    math::Quat lastLocal[3] = {math::identityQuat(), math::identityQuat(), math::identityQuat()};
    float jitterMax[3] = {0.0f, 0.0f, 0.0f}, jitterSum[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < kTicks; ++i) {
        if (which == 1) h.setPoseFoot(motion::Foot::Left, a0 + math::Vec3(0.0f, 0.15f, 0.0f));
        if (which == 2) h.setPoseHand(true, toes);
        h.tick(kDt);
        // CRICKET_NOHOLD (one-foot case): from 0.5 s the support spring's
        // motors get no strength for this step (its free-down band stays),
        // so the body can lean: does the plan follow it?
        if (which == 1 && std::getenv("CRICKET_NOHOLD") && i * kDt >= 0.5f) world.setSupportLimits(h.supportJoint(), 0.0f, 0.0f);
        world.step(kDt);
        {
            // The plan's own motion per tick (what the drive's lead follows):
            // each left-leg joint's local turn since the last tick (deg).
            const std::vector<math::Mat4>& pm = *h.bodyQueue().plan;
            for (int b = 0; b < 3; ++b) {
                const int j = legJoint[b], pj = h.rig().parent(j);
                const math::Quat q = math::normalize(math::inverse(math::toTransform(pm[static_cast<size_t>(pj)]).rotation) *
                                                     math::toTransform(pm[static_cast<size_t>(j)]).rotation);
                if (i >= kTicks - kLast) {
                    math::Quat d = math::normalize(q * math::inverse(lastLocal[b]));
                    if (d.w < 0.0f) d = -d;
                    const float a = math::degrees(math::length(math::rotationVector(d)));
                    jitterMax[b] = std::max(jitterMax[b], a);
                    jitterSum[b] += a / static_cast<float>(kLast);
                }
                lastLocal[b] = q;
            }
        }
        if (which == 1 && i % 30 == 0) {
            // How the one-foot stance goes (every 0.25 s): the spring, the
            // standing foot off its target, the centre of mass off the right
            // ankle (character axes, cm), the support weights.
            math::Vec3 sf(0.0f);
            float st = 0.0f;
            world.supportLoad(h.supportJoint(), kDt, sf, st);
            physics::PhysicsWorld::RagdollJointState fs;
            const agents::HumanBody& hb = h.humanBody();
            world.ragdollJointState(hb.ragdoll(), hb.partOf(h.rig().findJoint("foot_r")), kDt, fs);
            math::Vec3 c(0.0f);
            for (const auto& part : hb.parts()) c += world.transform(part.body).position * part.mass;
            c /= hb.totalMass();
            const math::Vec3 off = math::inverse(h.root().rotation) * (c - hb.boneWorld(world, hb.partOf(h.rig().findJoint("foot_r"))).position);
            {
                // The plan's centre of mass (world, through the root) against
                // the real one, and the root's drift since the start (cm, x).
                math::Vec3 pc(0.0f);
                for (size_t pi = 0; pi < hb.parts().size(); ++pi)
                    pc += hb.parts()[pi].mass * h.root().transformPoint(hb.partPose(static_cast<int>(pi), *h.bodyQueue().plan).position);
                pc /= hb.totalMass();
                if (i == 0) rootStart = h.root().position;
                std::printf("      real CoM x %.1f, plan CoM x %.1f, real - plan %.1f cm | root moved x %.1f cm\n",
                            100.0f * c.x, 100.0f * pc.x, 100.0f * (c.x - pc.x), 100.0f * (h.root().position.x - rootStart.x));
            }
            std::printf("    %.2f s spring %.0f N %.0f N m | foot_r off target %.1f deg | CoM off R ankle x %.1f z %.1f cm | weights %.2f %.2f\n",
                        i * kDt, math::length(sf), st, math::degrees(fs.tracking), 100.0f * off.x, 100.0f * off.z,
                        h.bodyQueue().supportWeight[0], h.bodyQueue().supportWeight[1]);
        }
        if (which == 0 && i >= kTicks - 10) {
            // Where the standing jitter starts: the plan's pelvis and left
            // ankle (character space, mm), the hip-to-ankle span, the knee,
            // the decider's hips reference and its height clamp.
            const std::vector<math::Mat4>& pm = *h.bodyQueue().plan;
            const math::Vec3 pel(pm[static_cast<size_t>(pelvis)][3]);
            const math::Vec3 hip(pm[static_cast<size_t>(legJoint[0])][3]), ank(pm[static_cast<size_t>(legJoint[2])][3]);
            const math::Vec3 ref = h.bodyQueue().hipsReference;
            std::printf("    tick %d plan pelvis y %.2f z %.2f | ankle L y %.2f z %.2f | span %.2f mm | knee %.2f deg | ref y %.2f z %.2f | clamped %d\n",
                        i, 1000.0f * pel.y, 1000.0f * pel.z, 1000.0f * ank.y, 1000.0f * ank.z, 1000.0f * math::length(hip - ank),
                        flexionDeg(pm, h, "thigh_l", "shin_l", "foot_l"), 1000.0f * ref.y, 1000.0f * ref.z,
                        h.bodyQueue().heightClamped ? 1 : 0);
        }
        if (i < kTicks - kLast) continue;
        math::Vec3 f(0.0f);
        float t = 0.0f;
        REQUIRE(world.supportLoad(h.supportJoint(), kDt, f, t));
        force += f / static_cast<float>(kLast);
        torque += t / static_cast<float>(kLast);
        REQUIRE(h.physicalModels(world, 1.0f, real));
        const math::Vec3 rp(real[static_cast<size_t>(pelvis)][3]), pp((*h.bodyQueue().plan)[static_cast<size_t>(pelvis)][3]);
        dropY += (pp.y - rp.y) / static_cast<float>(kLast);
        offLevel += std::sqrt((rp.x - pp.x) * (rp.x - pp.x) + (rp.z - pp.z) * (rp.z - pp.z)) / static_cast<float>(kLast);
    }
    const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
    std::printf("--- %s: spring force x %6.1f y %6.1f z %6.1f N, torque %5.1f N m | real pelvis %4.1f cm below plan, "
                "%4.1f cm off level | dropped %s\n",
                name, force.x, force.y, force.z, torque, 100.0f * dropY, 100.0f * offLevel, h.supported() ? "no" : "yes");
    std::printf("    knee L real %5.1f plan %5.1f, R real %5.1f plan %5.1f deg\n", flexionDeg(real, h, "thigh_l", "shin_l", "foot_l"),
                flexionDeg(plan, h, "thigh_l", "shin_l", "foot_l"), flexionDeg(real, h, "thigh_r", "shin_r", "foot_r"),
                flexionDeg(plan, h, "thigh_r", "shin_r", "foot_r"));
    const agents::HumanBody& body = h.humanBody();
    std::printf("    hold |N m|:");
    for (const char* bone : {"thigh_l", "shin_l", "foot_l", "thigh_r", "shin_r", "foot_r"})
        std::printf(" %s %5.1f", bone, math::length(body.holdTorque(body.partOf(h.rig().findJoint(bone)))));
    std::printf("\n    motor |N m|:");
    for (const char* bone : {"thigh_l", "shin_l", "foot_l", "thigh_r", "shin_r", "foot_r"}) {
        physics::PhysicsWorld::RagdollJointState st;
        REQUIRE(world.ragdollJointState(body.ragdoll(), body.partOf(h.rig().findJoint(bone)), kDt, st));
        std::printf(" %s %5.1f", bone, math::length(st.motor));
    }
    // What the motors pull toward: each joint's real angle off Jolt's target
    // and the pose's target clamped by the range (deg); the real pelvis
    // turned off the plan's (the spring's target).
    std::printf("\n    off target / clamped deg:");
    for (const char* bone : {"thigh_l", "shin_l", "foot_l", "thigh_r", "shin_r", "foot_r"}) {
        physics::PhysicsWorld::RagdollJointState st;
        REQUIRE(world.ragdollJointState(body.ragdoll(), body.partOf(h.rig().findJoint(bone)), kDt, st));
        std::printf(" %s %.2f/%.2f", bone, math::degrees(st.tracking), math::degrees(st.clamped));
    }
    std::printf("\n    plan's own turn per tick (last 1 s), mean / most deg: thigh_l %.3f/%.3f shin_l %.3f/%.3f foot_l %.3f/%.3f",
                jitterSum[0], jitterMax[0], jitterSum[1], jitterMax[1], jitterSum[2], jitterMax[2]);
    // Each leg joint's real local turn off the plan's, by axis (the parent
    // bone's frame, at rest the character's: x left (side), y up (twist), z
    // forward), deg.
    for (const char* bone : {"thigh_l", "shin_l", "foot_l"}) {
        const int j = h.rig().findJoint(bone), pj = h.rig().parent(j);
        auto local = [&](const std::vector<math::Mat4>& m) {
            return math::normalize(math::inverse(math::toTransform(m[static_cast<size_t>(pj)]).rotation) *
                                   math::toTransform(m[static_cast<size_t>(j)]).rotation);
        };
        const math::Vec3 off = math::rotationVector(math::normalize(local(real) * math::inverse(local(plan))));
        std::printf("\n    %s real off plan: x %.2f y %.2f z %.2f deg", bone, math::degrees(off.x), math::degrees(off.y),
                    math::degrees(off.z));
    }
    {
        const math::Quat rq = math::normalize(math::inverse(h.root().rotation) * world.transform(body.parts()[0].body).rotation);
        const math::Quat pq = math::normalize(body.partPose(0, plan).rotation);
        const math::Vec3 off = math::rotationVector(math::normalize(rq * math::inverse(pq)));
        std::printf("\n    real pelvis turned off plan: %.2f deg (x %.2f y %.2f z %.2f)", math::degrees(math::length(off)),
                    math::degrees(off.x), math::degrees(off.y), math::degrees(off.z));
    }
    std::printf("\n");
    // Computed only, not applied (2026-11-21): the stance hold by each foot's
    // ground force (Winter 1995). Share between standing feet by the lever
    // rule along the line between the foot centres; the force (share x M g,
    // up) at its centre of pressure = the centre of mass below it, clamped
    // inside the foot (decider.balanceStep foot outline). A joint's hold on
    // its child side = the free-limb hold + the moment of that force about it.
    core::Config hj;
    REQUIRE(hj.load(CRICKET_CONFIG_DIR "/human_joints.json"));
    const float halfW = hj.getFloat("decider.balanceStep.footHalfWidth", 0.045f);
    const float halfL = hj.getFloat("decider.balanceStep.footHalfLength", 0.11f);
    const float ahead = hj.getFloat("decider.balanceStep.footCentreAhead", 0.05f);
    const auto& parts = body.parts();
    const math::Vec3 g = world.world().gravity;
    const float M = body.totalMass();
    math::Vec3 com(0.0f);
    for (const auto& p : parts) com += world.transform(p.body).position * p.mass;
    com /= M;
    const math::Vec3 fwdW = h.root().transformDirection(math::kForward), leftW = h.root().transformDirection(math::kRight);
    const char* footBone[2] = {"foot_l", "foot_r"};
    int footPart[2];
    bool stands[2];
    math::Vec3 centre[2], cop[2];
    float share[2] = {0.0f, 0.0f};
    for (int f = 0; f < 2; ++f) {
        footPart[f] = body.partOf(h.rig().findJoint(footBone[f]));
        stands[f] = h.bodyQueue().supportWeight[f] > 0.5f;
        const math::Vec3 a = body.boneWorld(world, footPart[f]).position;
        centre[f] = a + fwdW * ahead;
        centre[f].y = h.root().position.y;
        const math::Vec3 d = com - centre[f];
        const float x = std::clamp(math::dot(d, leftW), -halfW, halfW), z = std::clamp(math::dot(d, fwdW), -halfL, halfL);
        cop[f] = centre[f] + leftW * x + fwdW * z;
    }
    if (stands[0] && stands[1]) {
        // Along the line between the feet the shares balance the centre of
        // mass; across it each foot's centre of pressure moves with it.
        const math::Vec3 line = centre[1] - centre[0];
        const float len2 = line.x * line.x + line.z * line.z;
        const math::Vec3 d = com - centre[0];
        share[1] = len2 > 1e-6f ? std::clamp((d.x * line.x + d.z * line.z) / len2, 0.0f, 1.0f) : 0.5f;
        share[0] = 1.0f - share[1];
        const math::Vec3 across = len2 > 1e-6f ? math::Vec3(-line.z, 0.0f, line.x) / std::sqrt(len2) : fwdW;
        const float off = math::dot(d, across);
        for (int f = 0; f < 2; ++f) {
            const math::Vec3 want = centre[f] + across * off - centre[f];
            const float x = std::clamp(math::dot(want, leftW), -halfW, halfW), z = std::clamp(math::dot(want, fwdW), -halfL, halfL);
            cop[f] = centre[f] + leftW * x + fwdW * z;
        }
    } else {
        for (int f = 0; f < 2; ++f) share[f] = stands[f] ? 1.0f : 0.0f;
    }
    // What no foot can take: the centre of mass off the shared centre of pressure.
    const math::Vec3 copAll = share[0] * cop[0] + share[1] * cop[1];
    const math::Vec3 rest = math::cross(com - copAll, g * M);
    std::printf("    computed: share L %.2f R %.2f | CoM ahead of ankles L %.1f R %.1f cm | moment no foot takes %.1f N m\n",
                share[0], share[1], 100.0f * math::dot(com - body.boneWorld(world, footPart[0]).position, fwdW),
                100.0f * math::dot(com - body.boneWorld(world, footPart[1]).position, fwdW), math::length(rest));
    std::printf("    computed hold |N m|:");
    for (int f = 0; f < 2; ++f) {
        for (int u = footPart[f]; u >= 0 && parts[static_cast<size_t>(u)].parent >= 0; u = parts[static_cast<size_t>(u)].parent) {
            // The child side: this part and every part below it.
            float mb = 0.0f;
            math::Vec3 mc(0.0f);
            for (size_t i = 0; i < parts.size(); ++i) {
                int k = static_cast<int>(i);
                while (k >= 0 && k != u) k = parts[static_cast<size_t>(k)].parent;
                if (k != u) continue;
                mb += parts[i].mass;
                mc += world.transform(parts[i].body).position * parts[i].mass;
            }
            const math::Vec3 pivot = body.boneWorld(world, u).position;
            math::Vec3 hold = mb > 0.0f ? -math::cross(mc / mb - pivot, g * mb) : math::Vec3(0.0f);
            hold -= math::cross(cop[f] - pivot, -g * (share[f] * M));
            std::printf(" %s %5.1f", h.rig().jointName(parts[static_cast<size_t>(u)].joint).c_str(), math::length(hold));
        }
    }
    std::printf("\n");
}
} // namespace

TEST_CASE("measure: stance legs carry the weight" * doctest::skip()) {
    weightCase(0, "standing");
    weightCase(1, "foot L held 15 cm up");
    weightCase(2, "hand L at the toes");
}

// Measurement (on demand): where the physics body does not follow the plan,
// joint by joint. For a held goal: each joint's real local rotation off the
// plan's (deg), its motor's torque and limit, and the torque its range's
// limits take (non-zero: Jolt's joint limit blocks it). Joints off by more
// than 3 deg.
namespace {
void jointGaps(int which, const char* name) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const std::vector<math::Mat4>& plan0 = *h.bodyQueue().plan;
    const math::Vec3 shoulderL = h.root().transformPoint(math::Vec3(plan0[static_cast<size_t>(h.rig().findJoint("upper_arm_l"))][3]));
    const math::Vec3 a0 = h.ankle(true, plan0, h.root());
    math::Vec3 goal(0.0f);
    bool hand = true;
    switch (which) {
    case 0: goal = shoulderL + h.root().transformDirection(math::Vec3(-0.45f, 0.0f, 0.3f)); break;  // across, ahead
    case 1: goal = shoulderL + h.root().transformDirection(math::Vec3(0.05f, 0.55f, 0.1f)); break;  // up overhead
    case 2: goal = shoulderL + h.root().transformDirection(math::Vec3(0.0f, 0.0f, 0.6f)); break;   // ahead, shoulder height
    case 3: goal = h.root().transformPoint(math::Vec3(0.15f, 0.05f, 0.25f)); break;               // at the toes
    default: goal = a0 + h.root().transformDirection(math::Vec3(0.0f, 0.2f, 0.3f)); hand = false; break;  // foot up ahead
    }
    for (int i = 0; i < 360; ++i) {
        if (hand) h.setPoseHand(true, goal);
        else h.setPoseFoot(motion::Foot::Left, goal);
        h.tick(kDt);
        world.step(kDt);
    }
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    REQUIRE(h.physicalModels(world, 1.0f, real));
    const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
    auto end = [&](const std::vector<math::Mat4>& m) { return hand ? h.wrist(true, m, h.root()) : h.ankle(true, m, h.root()); };
    std::printf("--- %s: end real off plan %.1f cm, plan off goal %.1f cm\n", name,
                100.0f * math::distance(end(real), end(plan)), 100.0f * math::distance(end(plan), goal));
    const agents::HumanBody& body = h.humanBody();
    std::printf("    ground contacts:");
    for (const char* bone : {"foot_l", "toe_l", "foot_r", "toe_r"}) {
        math::Vec3 points[8];
        const int part = body.partOf(h.rig().findJoint(bone));
        std::printf(" %s %d", bone, world.groundContacts(body.parts()[static_cast<size_t>(part)].body, points, 8));
    }
    std::printf("\n");
    for (size_t i = 0; i < body.parts().size(); ++i) {
        const auto& p = body.parts()[i];
        if (p.parent < 0) continue;
        const int j = p.joint, pj = body.parts()[static_cast<size_t>(p.parent)].joint;
        auto local = [&](const std::vector<math::Mat4>& m) {
            return math::inverse(math::toTransform(m[static_cast<size_t>(pj)]).rotation) * math::toTransform(m[static_cast<size_t>(j)]).rotation;
        };
        const math::Quat d = math::inverse(local(plan)) * local(real);
        const float gap = math::degrees(2.0f * std::acos(std::clamp(std::abs(d.w), 0.0f, 1.0f)));
        if (gap < 3.0f) continue;
        physics::PhysicsWorld::RagdollJointState st;
        REQUIRE(world.ragdollJointState(body.ragdoll(), static_cast<int>(i), kDt, st));
        std::printf("    %-12s off %5.1f deg (Jolt clamp %5.1f, tracking %5.1f) | motor %6.1f N m (limits X %5.1f..%5.1f "
                    "Y %5.1f..%5.1f Z %5.1f..%5.1f) | range limits %6.1f N m | hold %6.1f\n",
                    h.rig().jointName(j).c_str(), gap, math::degrees(st.clamped), math::degrees(st.tracking), math::length(st.motor),
                    st.motorMin.x, st.motorMax.x, st.motorMin.y, st.motorMax.y, st.motorMin.z, st.motorMax.z,
                    math::length(st.range), math::length(body.holdTorque(static_cast<int>(i))));
    }
    float worstClamp = 0.0f;
    std::string worstJoint = "-";
    for (size_t i = 0; i < body.parts().size(); ++i) {
        physics::PhysicsWorld::RagdollJointState st;
        if (!world.ragdollJointState(body.ragdoll(), static_cast<int>(i), kDt, st) || st.clamped <= worstClamp) continue;
        worstClamp = st.clamped;
        worstJoint = h.rig().jointName(body.parts()[i].joint);
    }
    std::printf("    Jolt clamp worst over every joint %.2f deg (%s)\n", math::degrees(worstClamp), worstJoint.c_str());
}
} // namespace

TEST_CASE("measure: where the physics body misses the plan" * doctest::skip()) {
    jointGaps(0, "hand L across, ahead");
    jointGaps(1, "hand L overhead");
    jointGaps(2, "hand L 0.6 m ahead");
    jointGaps(3, "hand L at the toes");
    jointGaps(4, "foot L up and ahead");
}

// Measurement (on demand): a moving goal (the hand at 0.5 m/s, 1 s, each
// way): per joint the worst tracking gap (Jolt's target to the joint, deg),
// the damper's predicted lag there (D / K x the plan's joint speed: a
// position spring with a damper toward no motion trails a moving target by
// that), whether the motor was at its torque limit, and Jolt's clamp.
namespace {
void movingGaps(math::Vec3 dir, const char* name) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const agents::HumanBody& body = h.humanBody();
    const size_t n = body.parts().size();
    std::vector<math::Quat> last(n, math::identityQuat());
    std::vector<float> worst(n, 0.0f), predicted(n, 0.0f), clamp(n, 0.0f);
    std::vector<int> saturated(n, 0);
    math::Vec3 goal = h.wrist(true, *h.bodyQueue().plan, h.root());
    const math::Vec3 step = h.root().transformDirection(dir) * (0.5f * kDt);
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    float endWorst = 0.0f;
    for (int t = 0; t < 120; ++t) {
        goal += step;
        h.setPoseHand(true, goal);
        h.tick(kDt);
        world.step(kDt);
        const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
        REQUIRE(h.physicalModels(world, 1.0f, real));
        endWorst = std::max(endWorst, math::distance(h.wrist(true, real, h.root()), h.wrist(true, plan, h.root())));
        for (size_t i = 0; i < n; ++i) {
            const auto& p = body.parts()[i];
            if (p.parent < 0 || !p.muscle) continue;
            const int pj = body.parts()[static_cast<size_t>(p.parent)].joint;
            const math::Quat local = math::inverse(math::toTransform(plan[static_cast<size_t>(pj)]).rotation) *
                                     math::toTransform(plan[static_cast<size_t>(p.joint)]).rotation;
            const math::Quat d = math::inverse(last[i]) * local;
            const float speed = 2.0f * std::acos(std::clamp(std::abs(d.w), 0.0f, 1.0f)) / kDt;
            last[i] = local;
            if (t < 2) continue;
            physics::PhysicsWorld::RagdollJointState st;
            REQUIRE(world.ragdollJointState(body.ragdoll(), static_cast<int>(i), kDt, st));
            clamp[i] = std::max(clamp[i], st.clamped);
            if (st.tracking > worst[i]) {
                worst[i] = st.tracking;
                predicted[i] = 0.06f * speed;  // D / K = 2 zeta / w, w = 1 / 0.03 s, zeta 1
            }
            bool atLimit = false;  // a motor axis at its torque limit the way it pulls
            for (int a = 0; a < 3; ++a) {
                const float limit = st.motor[a] >= 0.0f ? st.motorMax[a] : -st.motorMin[a];
                atLimit = atLimit || (limit > 0.0f && std::abs(st.motor[a]) > 0.95f * limit);
            }
            if (atLimit) ++saturated[i];
        }
    }
    size_t worstClamp = 0;
    for (size_t i = 0; i < n; ++i)
        if (clamp[i] > clamp[worstClamp]) worstClamp = i;
    std::printf("--- hand L %s at 0.5 m/s: wrist real off plan worst %.1f cm | Jolt clamp worst over every joint %.2f deg (%s)\n",
                name, 100.0f * endWorst, math::degrees(clamp[worstClamp]), h.rig().jointName(body.parts()[worstClamp].joint).c_str());
    for (size_t i = 0; i < n; ++i) {
        if (worst[i] < math::radians(2.0f) && clamp[i] < math::radians(2.0f)) continue;
        std::printf("    %-12s tracking worst %5.1f deg (damper predicts %5.1f) | at the torque limit %3d ticks | Jolt clamp worst %5.1f deg\n",
                    h.rig().jointName(body.parts()[i].joint).c_str(), math::degrees(worst[i]), math::degrees(predicted[i]),
                    saturated[i], math::degrees(clamp[i]));
    }
}
} // namespace

TEST_CASE("measure: the drive with a moving goal" * doctest::skip()) {
    movingGaps(math::Vec3(0, 0, 1), "forward");
    movingGaps(math::Vec3(-1, 0, 0), "across");
    movingGaps(math::Vec3(0, 1, 0), "up");
    movingGaps(math::Vec3(0, -1, 0), "down");
}

// Measurement (on demand): the cost of one human per physics tick (release
// build): the decider and the muscles (Human::tick) and Jolt's step, held
// still and with the hand moving.
#include <chrono>
TEST_CASE("measure: cost per human per physics tick" * doctest::skip()) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    math::Vec3 goal = h.wrist(true, *h.bodyQueue().plan, h.root());
    using Clock = std::chrono::steady_clock;
    double human = 0.0, step = 0.0;
    constexpr int kTicks = 2400;
    for (int t = 0; t < kTicks; ++t) {
        const float phase = static_cast<float>(t) * kDt;
        goal += h.root().transformDirection(math::Vec3(0.0f, 0.0f, std::sin(phase * 2.0f))) * (0.5f * kDt);
        h.setPoseHand(true, goal);
        const auto a = Clock::now();
        h.tick(kDt);
        const auto b = Clock::now();
        world.step(kDt);
        const auto c = Clock::now();
        human += std::chrono::duration<double, std::micro>(b - a).count();
        step += std::chrono::duration<double, std::micro>(c - b).count();
    }
    std::printf("--- per tick (%d ticks): Human::tick %.1f us, world.step %.1f us, total %.1f us\n", kTicks, human / kTicks,
                step / kTicks, (human + step) / kTicks);
}
