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
    const math::Vec3 sideAxis = h.root().transformDirection(math::Vec3(1.0f, 0.0f, 0.0f));  // +X left, at the start
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
            if (i * kDt > 0.6f && i * kDt < turnAt) {
                // Walking straight: the pelvis's side sway in the world, across
                // the starting heading (not from the root: that follows the hips).
                const float side = math::dot(p, sideAxis);
                sideMin = std::min(sideMin, side);
                sideMax = std::max(sideMax, side);
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
    int againstPlan = 0, againstReal = 0, ticks = 0, stepTicks = 0;
    for (int i = 0; i < 180; ++i) {  // 1.5 s: 0.75 m of goal travel
        goal += step;
        if (hand) h.setPoseHand(left, goal);
        else h.setPoseFoot(left ? motion::Foot::Left : motion::Foot::Right, goal);
        h.tick(kDt);
        world.step(kDt);
        if (h.stepFoot() >= 0) ++stepTicks;  // a balance step under way
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
                "final miss plan %4.1f cm, stepping %d ticks\n",
                name, againstPlan, worstPlan, againstReal, worstReal, 100.0f * math::distance(lastPlan, goal), stepTicks);
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

// Measurement (on demand): when a knee goes (nearly) straight while walking
// at the top pace, which leg, planted or not, its span, and the hips' bounds.
TEST_CASE("measure: straight knee while walking" * doctest::skip()) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    for (int i = 0; i < 5; ++i) h.changeWalkSpeed(1);
    const char* legs[2][3] = {{"thigh_l", "shin_l", "foot_l"}, {"thigh_r", "shin_r", "foot_r"}};
    math::Vec3 lastAnkle[2];
    int straight[2] = {0, 0}, straightPlanted[2] = {0, 0}, printed = 0, ticks = 0;
    for (int i = 0; i < 480; ++i) {
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        for (int l = 0; l < 2; ++l) {
            const int a = h.rig().findJoint(legs[l][0]), b = h.rig().findJoint(legs[l][1]), c = h.rig().findJoint(legs[l][2]);
            const math::Vec3 hip(p[static_cast<size_t>(a)][3]), knee(p[static_cast<size_t>(b)][3]), ankle(p[static_cast<size_t>(c)][3]);
            const float speed = i > 0 ? math::distance(ankle, lastAnkle[l]) / kDt : 0.0f;
            lastAnkle[l] = ankle;
            if (i < 120) continue;
            const float knee0 = flexionDeg(p, h, legs[l][0], legs[l][1], legs[l][2]);
            if (knee0 >= 1.0f) continue;
            const bool planted = speed < 0.05f;
            ++straight[l];
            if (planted) ++straightPlanted[l];
            if (printed < 8) {
                ++printed;
                const float len = math::distance(hip, knee) + math::distance(knee, ankle);
                std::printf("  t %.3f s: %s knee %.1f deg, foot %s (%.2f m/s), span %.4f of leg, support %.2f, "
                            "height clamped %s\n",
                            i * kDt, l == 0 ? "left" : "right", knee0, planted ? "planted" : "moving", speed,
                            math::distance(hip, ankle) / len, h.bodyQueue().supportWeight[l],
                            h.bodyQueue().heightClamped ? "yes" : "no");
            }
        }
        if (i >= 120) ++ticks;
    }
    std::printf("knee under 1 deg: left %d ticks (%d planted), right %d ticks (%d planted), of %d\n", straight[0],
                straightPlanted[0], straight[1], straightPlanted[1], ticks);
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

// Measurement (on demand): W held for 3 s; the capsule against the real
// hips, the real hips against the plan's and the feet, the side jerk.
TEST_CASE("measure: capsule and hips through W steps" * doctest::skip()) {
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
    const math::Vec3 fwd = h.root().transformDirection(math::kForward), left = h.root().transformDirection(math::Vec3(1, 0, 0));
    float capMax = 0.0f, capSum = 0.0f, lagPlanMax = 0.0f, behindFeetMax = 0.0f, sideAccel = 0.0f;
    float sideMin = 9.0f, sideMax = -9.0f, frontLagMax = 0.0f;
    float yawP[2] = {99, -99}, rollP[2] = {99, -99}, yawR[2] = {99, -99}, rollR[2] = {99, -99};
    auto angles = [&](const math::Mat4& m, float* yaw, float* roll) {
        const math::Vec3 f = math::normalize(math::Vec3(m[2])), l = math::normalize(math::Vec3(m[0]));  // character space
        const float y = math::degrees(std::atan2(f.x, f.z)), r = math::degrees(std::asin(std::clamp(l.y, -1.0f, 1.0f)));
        yaw[0] = std::min(yaw[0], y); yaw[1] = std::max(yaw[1], y); roll[0] = std::min(roll[0], r); roll[1] = std::max(roll[1], r);
    };
    int n = 0;
    math::Vec3 hist[7];
    for (int i = 0; i < 360; ++i) {
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        math::Vec3 capFeet, centre, hips;
        float ht = 0.0f, rad = 0.0f;
        if (!h.capsuleDebug(capFeet, ht, rad, centre, hips) || !h.bodyQueue().plan) continue;
        const math::Vec3 plan = h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3]));
        if (!h.physicalModels(world, 1.0f, real)) continue;
        const float ankL = math::dot(h.ankle(true, real, h.root()), fwd), ankR = math::dot(h.ankle(false, real, h.root()), fwd);
        for (int k = 6; k > 0; --k) hist[k] = hist[k - 1];
        hist[0] = hips;
        if (i < 60) continue;  // steady walking only
        const math::Vec3 c = capFeet - hips;
        const float cap = std::sqrt(c.x * c.x + c.z * c.z);
        capMax = std::max(capMax, cap);
        capSum += cap;
        lagPlanMax = std::max(lagPlanMax, math::dot(plan - hips, fwd));
        behindFeetMax = std::max(behindFeetMax, math::dot(centre - hips, fwd));
        frontLagMax = std::max(frontLagMax, std::max(ankL, ankR) - math::dot(hips, fwd));
        const float s = math::dot(hips, left);
        sideMin = std::min(sideMin, s);
        sideMax = std::max(sideMax, s);
        const math::Vec3 a = (hist[0] - 2.0f * hist[3] + hist[6]) / (9.0f * kDt * kDt);
        sideAccel = std::max(sideAccel, std::abs(math::dot(a, left)));
        angles((*h.bodyQueue().plan)[0], yawP, rollP);
        angles(real[0], yawR, rollR);
        ++n;
        if (i % 30 == 0)
            std::printf("  t %.2f s: capsule-hips %4.1f cm | hips behind plan %5.1f cm, behind feet centre %5.1f cm, "
                        "front ankle ahead %5.1f cm | hips fwd %.3f m\n",
                        i * kDt, 100.0f * cap, 100.0f * math::dot(plan - hips, fwd), 100.0f * math::dot(centre - hips, fwd),
                        100.0f * (std::max(ankL, ankR) - math::dot(hips, fwd)), math::dot(hips, fwd));
    }
    std::printf("capsule from real hips: max %.1f cm, mean %.1f cm | real hips behind plan: max %.1f cm | behind feet "
                "centre: max %.1f cm | front ankle ahead of hips: max %.1f cm | side sway %.1f cm, side accel %.1f m/s^2 | "
                "dropped %s\n",
                100.0f * capMax, n ? 100.0f * capSum / n : 0.0f, 100.0f * lagPlanMax, 100.0f * behindFeetMax,
                100.0f * frontLagMax, 100.0f * (sideMax - sideMin), sideAccel, h.supported() ? "no" : "yes");
    std::printf("pelvis turn (yaw) range: plan %.1f deg, real %.1f deg | pelvis drop (roll) range: plan %.1f deg, real %.1f deg\n",
                yawP[1] - yawP[0], yawR[1] - yawR[0], rollP[1] - rollP[0], rollR[1] - rollR[0]);
}

// Measurement (run on demand): the balance step under the hips target. For
// each input (held 2.5 s, then let go for 1.5 s) every step's foot, length
// (from the other foot, forward / left in the starting frame) and time, the
// hips against the feet's centre, a real foot's slide while its plan stands.
TEST_CASE("measure: balance steps for W A S D" * doctest::skip()) {
    struct Case {
        const char* name;
        math::Vec2 input;
    };
    for (const Case& c : {Case{"W", {0.0f, 1.0f}}, Case{"S", {0.0f, -1.0f}}, Case{"A", {1.0f, 0.0f}}, Case{"D", {-1.0f, 0.0f}}}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        std::printf("--- %s\n", c.name);
        for (int i = 0; i < 60; ++i) {
            h.tick(kDt);
            world.step(kDt);
            const motion::StepAdvice& a = h.bodyQueue().step;
            if (a.wanted && h.stepFoot() >= 0 && h.stepPhase() < 0.02f)
                std::printf("    settle tick %d: step %d, capture outside %.1f cm, velocity %.2f %.2f m/s\n", i, a.foot,
                            100.0f * a.outside, a.velocity.x, a.velocity.z);
        }
        const math::Vec3 fwd = h.root().transformDirection(math::kForward), left = h.root().transformDirection(math::Vec3(1, 0, 0));
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        int lastFoot = -1, steps = 0;
        float lastPhase = 0.0f, slideAt = 0.0f;
        Range sway;
        int slideFoot = -1;
        float stepStart = 0.0f, hipsAheadMax = -9.0f, hipsAheadMin = 9.0f, slide = 0.0f, chest = 0.0f;
        math::Vec3 still[2];
        bool holding[2] = {false, false};
        math::Vec3 lastPlan[2];
        bool have = false;
        const int chestJ = h.rig().findJoint("chest");
        for (int i = 0; i < 480; ++i) {
            const float time = i * kDt;
            h.setStepInput(time < 2.5f ? c.input : math::Vec2(0.0f));
            h.tick(kDt);
            world.step(kDt);
            if (!h.bodyQueue().plan || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
            if (c.name[0] == 'W' && time > 2.4f && time < 3.6f && i % 6 == 0) {
                const motion::StepAdvice& a = h.bodyQueue().step;
                const math::Vec3 xc = h.root().transformPoint(a.capturePoint);
                const math::Vec3 pl = h.root().transformPoint(math::Vec3(plan[0][3]));
                const math::Vec3 tg = h.root().transformPoint(h.bodyTargets().hips.position);
                auto fl = [&](math::Vec3 p) { return 100.0f * math::dot(p, left); };
                auto ff = [&](math::Vec3 p) { return 100.0f * math::dot(p, fwd); };
                std::printf("    t %.2f: xcom fwd %5.1f left %5.1f | outside %4.1f cm | wanted %d foot %d | plan hips "
                            "%5.1f/%5.1f target %5.1f/%5.1f | feet L %5.1f/%5.1f R %5.1f/%5.1f\n",
                            time, ff(xc), fl(xc), 100.0f * a.outside, a.wanted, a.foot, ff(pl), fl(pl), ff(tg), fl(tg),
                            ff(h.ankle(true, plan, h.root())), fl(h.ankle(true, plan, h.root())),
                            ff(h.ankle(false, plan, h.root())), fl(h.ankle(false, plan, h.root())));
            }
            const int foot = h.stepFoot();
            // A step ended: the foot changed, or the same foot began again.
            const bool restarted = foot >= 0 && foot == lastFoot && h.stepPhase() < lastPhase;
            lastPhase = h.stepPhase();
            if (lastFoot >= 0 && (foot != lastFoot || restarted)) {
                const math::Vec3 a = h.ankle(lastFoot == 0, plan, h.root()), o = h.ankle(lastFoot != 0, plan, h.root());
                const math::Vec3 r = h.ankle(lastFoot == 0, real, h.root());
                std::printf("  step %d %s at %.2f-%.2f s: from other foot fwd %5.1f cm, left %5.1f cm | real ankle off "
                            "plan at landing %4.1f cm (height %4.1f cm)\n",
                            ++steps, lastFoot == 0 ? "L" : "R", stepStart, time, 100.0f * math::dot(a - o, fwd),
                            100.0f * math::dot(a - o, left), 100.0f * math::length(math::Vec3(r.x - a.x, 0.0f, r.z - a.z)),
                            100.0f * (r.y - a.y));
            }
            if (foot >= 0 && (foot != lastFoot || restarted)) stepStart = time;
            lastFoot = foot;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(real[0][3]));
            const math::Vec3 centre = 0.5f * (h.ankle(true, real, h.root()) + h.ankle(false, real, h.root()));
            const math::Vec3 dir = math::length2(c.input) > 0.0f
                                       ? math::normalize(fwd * c.input.y + left * c.input.x)
                                       : fwd;
            if (time > 0.5f && time < 2.5f) {
                const float ahead = math::dot(hips - centre, dir);
                hipsAheadMax = std::max(hipsAheadMax, ahead);
                hipsAheadMin = std::min(hipsAheadMin, ahead);
                if (time > 1.0f) {  // across the way it moves: the side sway (real hips, world)
                    const math::Vec3 across = math::cross(math::kUp, dir);
                    sway.add(math::dot(hips, across));
                }
            }
            const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(chestJ)][1]));
            chest = std::max(chest, math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f))));
            for (int f = 0; f < 2; ++f) {
                const math::Vec3 p = h.ankle(f == 0, plan, h.root()), a = h.ankle(f == 0, real, h.root());
                const bool stands = have && math::length(p - lastPlan[f]) / kDt < 0.05f;
                if (stands && !holding[f]) still[f] = a;
                holding[f] = stands;
                if (stands) {
                    const float d = math::length(math::Vec3(a.x - still[f].x, 0.0f, a.z - still[f].z));
                    if (d > slide) {
                        slide = d;
                        slideAt = time;
                        slideFoot = f;
                    }
                }
                lastPlan[f] = p;
            }
            have = true;
        }
        std::printf("  %d steps | hips ahead of the feet's centre (moving) %.1f .. %.1f cm | foot slide %.1f cm | chest "
                    "lean %.1f deg | dropped %s | slide foot %d at %.2f s | hips side sway %.1f cm\n",
                    steps, 100.0f * hipsAheadMin, 100.0f * hipsAheadMax, 100.0f * slide, chest, h.supported() ? "no" : "yes",
                    slideFoot, slideAt, 100.0f * sway.span());
    }
}

// TEMP (remove before commit): a standing jump, measured.
TEST_CASE("TEMP jump" * doctest::skip()) {
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
    REQUIRE(h.physicalModels(world, 1.0f, real));
    const float standY = h.root().transformPoint(math::Vec3(real[0][3])).y;
    float lowY = 9, highY = -9, ankleHigh = -9, vUp = 0, lastY = standY, chestMax = 0, kneeMax = 0, landKnee = 0;
    math::Vec3 hist[7];
    float accel = 0, accelPhase[5] = {0, 0, 0, 0, 0};
    int n = 0;
    for (int i = 0; i < 360; ++i) {
        h.setJumpInput(i < (std::getenv("CRICKET_PRESS") ? std::atoi(std::getenv("CRICKET_PRESS")) : 120));
        h.tick(kDt);
        world.step(kDt);
        if (!h.physicalModels(world, 1.0f, real)) continue;
        const math::Vec3 p = h.root().transformPoint(math::Vec3(real[0][3]));
        const float y = p.y;
        lowY = std::min(lowY, y);
        highY = std::max(highY, y);
        ankleHigh = std::max(ankleHigh, std::min(h.ankle(true, real, h.root()).y, h.ankle(false, real, h.root()).y));
        vUp = std::max(vUp, (y - lastY) / kDt);
        lastY = y;
        kneeMax = std::max(kneeMax, flexionDeg(real, h, "thigh_l", "shin_l", "foot_l"));
        if (static_cast<int>(h.jumpPhase()) == 4) landKnee = std::max(landKnee, flexionDeg(real, h, "thigh_l", "shin_l", "foot_l"));
        const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(h.rig().findJoint("chest"))][1]));
        chestMax = std::max(chestMax, math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f))));
        for (int k = 6; k > 0; --k) hist[k] = hist[k - 1];
        hist[0] = p;
        if (++n >= 7) {
            const float a = math::length((hist[0] - 2.0f * hist[3] + hist[6]) / (9.0f * kDt * kDt));
            const int ph = static_cast<int>(h.jumpPhase());
            if (ph != 3) accel = std::max(accel, a);  // not in flight (there it is gravity)
            accelPhase[ph] = std::max(accelPhase[ph], a);
            if (ph >= 3 && std::getenv("CRICKET_TRACE"))
                std::printf("  t %.3f ph %d pelvis %+.3f vy %+.2f accel %5.1f (y %+6.1f) | ankles %.3f %.3f | knee L %3.0f plan %3.0f | held %d\n",
                            i * kDt, ph, y - standY, (hist[0].y - hist[1].y) / kDt, a,
                            (hist[0].y - 2.0f * hist[3].y + hist[6].y) / (9.0f * kDt * kDt), h.ankle(true, real, h.root()).y,
                            h.ankle(false, real, h.root()).y, flexionDeg(real, h, "thigh_l", "shin_l", "foot_l"),
                            flexionDeg(*h.bodyQueue().plan, h, "thigh_l", "shin_l", "foot_l"), h.supported() ? 1 : 0);
        }
    }
    std::printf("TEMP landing knee max %.0f deg\n", landKnee);
    std::printf("TEMP jump: crouch %.1f cm (knee max %.0f deg), peak %+.1f cm above standing, feet up %.3f m, up speed %.2f m/s, "
                "chest lean max %.0f deg | pelvis accel max (not flight) %.1f m/s^2; by phase none %.1f crouch %.1f push %.1f "
                "flight %.1f land %.1f\n",
                100 * (standY - lowY), kneeMax, 100 * (highY - standY), ankleHigh, vUp, chestMax, accel, accelPhase[0],
                accelPhase[1], accelPhase[2], accelPhase[3], accelPhase[4]);
}
