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
        Range sway, pelvisYawPlan, pelvisYawReal, chestYawReal, armPlan, armReal, bobPlan, bobReal, speedReal;
        // Pelvis height in double support (a step's weight shift) and in single support (mid-swing).
        double hDouble = 0.0, hSingle = 0.0;
        int nDouble = 0, nSingle = 0;
        math::Vec3 lastHips(0.0f);
        bool haveHips = false;
        double sumPC = 0.0, sumPP = 0.0, sumCC = 0.0;
        // A bone's turn about the vertical (deg) from its lateral axis, against
        // the same axis standing (character space: the facing stays fixed).
        auto yawOf = [](const math::Mat4& m, math::Vec3 ref) {
            math::Vec3 v(m[0]);
            v.y = 0.0f;
            v = math::normalize(v);
            return math::degrees(std::atan2(math::cross(ref, v).y, math::dot(ref, v)));
        };
        auto flatX = [](const math::Mat4& m) {
            math::Vec3 v(m[0]);
            v.y = 0.0f;
            return math::normalize(v);
        };
        math::Vec3 pelvisRef(1, 0, 0), chestRef(1, 0, 0);
        bool refs = false;
        int slideFoot = -1;
        float stepStart = 0.0f, hipsAheadMax = -9.0f, hipsAheadMin = 9.0f, slide = 0.0f, chest = 0.0f;
        math::Vec3 still[2];
        bool holding[2] = {false, false};
        math::Vec3 lastPlan[2];
        bool have = false;
        const int chestJ = h.rig().findJoint("chest");
        for (int i = 0; i < 720; ++i) {
            const float time = i * kDt;
            h.setStepInput(time < 4.5f ? c.input : math::Vec2(0.0f));
            h.tick(kDt);
            world.step(kDt);
            if (!h.bodyQueue().plan || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
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
            if (time > 0.5f && time < 4.5f) {
                const float ahead = math::dot(hips - centre, dir);
                hipsAheadMax = std::max(hipsAheadMax, ahead);
                hipsAheadMin = std::min(hipsAheadMin, ahead);
                if (time > 2.5f) {  // across the way it moves: the side sway (real hips, world)
                    const math::Vec3 across = math::cross(math::kUp, dir);
                    sway.add(math::dot(hips, across));
                }
            }
            if (!refs) {
                pelvisRef = flatX(real[0]);
                chestRef = flatX(real[static_cast<size_t>(chestJ)]);
                refs = true;
            }
            if (time > 2.5f && time < 4.5f) {
                const float pp = yawOf(plan[0], pelvisRef), pr = yawOf(real[0], pelvisRef);
                const float cr = yawOf(real[static_cast<size_t>(chestJ)], chestRef);
                pelvisYawPlan.add(pp);
                pelvisYawReal.add(pr);
                chestYawReal.add(cr);
                // The left shoulder's swing fore-aft (deg, forward +): the upper arm from straight down.
                auto swing = [&](const std::vector<math::Mat4>& m) {
                    const math::Vec3 a(m[static_cast<size_t>(h.rig().findJoint("upper_arm_l"))][3]);
                    const math::Vec3 b(m[static_cast<size_t>(h.rig().findJoint("forearm_l"))][3]);
                    const math::Vec3 d = b - a;
                    return math::degrees(std::atan2(math::dot(d, math::kForward), -d.y));
                };
                armPlan.add(swing(plan));
                armReal.add(swing(real));
                const math::Vec3 hipsNow = h.root().transformPoint(math::Vec3(real[0][3]));
                bobPlan.add(plan[0][3].y);
                bobReal.add(hipsNow.y);
                if (haveHips) speedReal.add(math::dot(hipsNow - lastHips, dir) / kDt);
                lastHips = hipsNow;
                haveHips = true;
                const float share = h.stepPhase();
                if (h.stepFoot() >= 0 && share < 0.15f) { hDouble += plan[0][3].y; ++nDouble; }
                if (h.stepFoot() >= 0 && share > 0.5f && share < 0.7f) { hSingle += plan[0][3].y; ++nSingle; }
                sumPC += pr * cr;
                sumPP += pr * pr;
                sumCC += cr * cr;
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
        // Peak to peak; chest with pelvis: +1 turning together, -1 opposite.
        const double corr = sumPP > 0.0 && sumCC > 0.0 ? sumPC / std::sqrt(sumPP * sumCC) : 0.0;
        std::printf("  turn (peak to peak): pelvis plan %.1f deg, real %.1f deg | chest real %.1f deg | chest with pelvis "
                    "%+.2f\n",
                    pelvisYawPlan.span(), pelvisYawReal.span(), chestYawReal.span(), corr);
        std::printf("  left shoulder swing (peak to peak): plan %.1f deg, real %.1f deg\n", armPlan.span(), armReal.span());
        std::printf("  pelvis bob: plan %.1f cm, real %.1f cm | plan height double support %.1f cm vs single %.1f cm "
                    "(mean) | real speed along the way %.2f .. %.2f m/s\n",
                    100.0f * bobPlan.span(), 100.0f * bobReal.span(), nDouble ? 100.0 * hDouble / nDouble : 0.0,
                    nSingle ? 100.0 * hSingle / nSingle : 0.0, speedReal.lo, speedReal.hi);
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

// Measurement (on demand): where the chest's lean comes from in a W-then-A
// run. Each variant changes config text in a temporary copy (the game's files
// stay as they are).
#include <filesystem>
#include <fstream>
#include <sstream>
namespace {
std::string readText(const std::string& path) {
    std::ifstream f(path);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}
bool replaceOnce(std::string& text, const std::string& from, const std::string& to) {
    const size_t at = text.find(from);
    if (at == std::string::npos) return false;
    text.replace(at, from.size(), to);
    return true;
}
} // namespace

TEST_CASE("measure: chest lean, W then A, by cause" * doctest::skip()) {
    struct Variant {
        const char* name;
        float hands;      // passive hands share (now 0)
        float sideSpeed;  // walk.sideSpeed (now 0.35)
        float maxStep = 0.5f;  // decider.balanceStep.maxStepSide (now 0.5)
    };
    const std::string dir = (std::filesystem::temp_directory_path() / "cricket_lean").string();
    std::filesystem::create_directories(dir);
    for (const Variant& v : {Variant{"before (side step 0.6, side 0.45)", 0.0f, 0.45f, 0.6f},
                             Variant{"now (side step 0.5, side 0.35)", 0.0f, 0.35f, 0.5f}}) {
        std::string joints = readText(CRICKET_CONFIG_DIR "/human_joints.json");
        std::string loco = readText(CRICKET_CONFIG_DIR "/locomotion.json");
        char buf[64];
        std::snprintf(buf, sizeof buf, "\"hands\": %.2f,", v.hands);
        REQUIRE(replaceOnce(joints, "\"passiveInfluence\": { \"hands\": 0.0,", std::string("\"passiveInfluence\": { ") + buf));
        REQUIRE(replaceOnce(loco, "\"passive\": { \"hands\": 0.0,", std::string("\"passive\": { ") + buf));
        std::snprintf(buf, sizeof buf, "\"sideSpeed\": %.2f,", v.sideSpeed);
        REQUIRE(replaceOnce(loco, "\"sideSpeed\": 0.35,", buf));
        std::snprintf(buf, sizeof buf, "\"maxStepSide\": %.2f", v.maxStep);
        REQUIRE(replaceOnce(joints, "\"maxStepSide\": 0.5", buf));
        std::ofstream(dir + "/human_joints.json") << joints;
        std::ofstream(dir + "/locomotion.json") << loco;

        static core::Config skeleton, jc, lc;  // alive: Human keeps the joints config
        REQUIRE(skeleton.load(CRICKET_CONFIG_DIR "/human_skeleton.json"));
        REQUIRE(jc.load(dir + "/human_joints.json"));
        REQUIRE(lc.load(dir + "/locomotion.json"));
        rig::SkeletonDef def;
        REQUIRE(rig::loadSkeletonDef(skeleton, def));
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(h.init(def, def.height, jc, lc));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        const int chestJ = h.rig().findJoint("chest");
        float lean[2] = {0.0f, 0.0f}, leanSide[2] = {0.0f, 0.0f}, leanFwd[2] = {0.0f, 0.0f}, pelvisLean[2] = {0.0f, 0.0f};
        Range heightA, leadA;
        float leanAt[2] = {0.0f, 0.0f}, planPelvisLean[2] = {0.0f, 0.0f}, widest[2] = {0.0f, 0.0f}, shortMax[2] = {0.0f, 0.0f};
        for (int i = 0; i < 480; ++i) {
            const float time = i * kDt;
            const int phase = time < 2.0f ? 0 : 1;
            h.setStepInput(phase == 0 ? math::Vec2(0.0f, 1.0f) : math::Vec2(1.0f, 0.0f));
            h.tick(kDt);
            world.step(kDt);
            if (!h.physicalModels(world, 1.0f, real)) continue;
            const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(chestJ)][1]));
            const float a = math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f)));
            if (a > lean[phase]) {
                lean[phase] = a;
                leanAt[phase] = time;
                leanSide[phase] = math::degrees(std::atan2(up.x, up.y));
                leanFwd[phase] = math::degrees(std::atan2(up.z, up.y));
            }
            const math::Vec3 pu = math::normalize(math::Vec3(real[0][1]));
            pelvisLean[phase] = std::max(pelvisLean[phase], math::degrees(std::acos(std::clamp(pu.y, -1.0f, 1.0f))));
            if (h.bodyQueue().plan) {
                const math::Vec3 qu = math::normalize(math::Vec3((*h.bodyQueue().plan)[0][1]));
                planPelvisLean[phase] = std::max(planPelvisLean[phase], math::degrees(std::acos(std::clamp(qu.y, -1.0f, 1.0f))));
                if (time > 2.5f) {  // A, past its start: the plan's hips height and how far they lead the feet
                    heightA.add((*h.bodyQueue().plan)[0][3].y);
                    const std::vector<math::Mat4>& pl = *h.bodyQueue().plan;
                    const math::Vec3 mid = 0.5f * (h.ankle(true, pl, h.root()) + h.ankle(false, pl, h.root()));
                    const math::Vec3 hp = h.root().transformPoint(math::Vec3(pl[0][3]));
                    leadA.add(math::dot(hp - mid, h.root().transformDirection(math::Vec3(1, 0, 0))));
                }
                // The widest stance (ankles apart, plan) and a step's foot short of its target (out of reach).
                const float apart = math::distance(h.ankle(true, *h.bodyQueue().plan, h.root()), h.ankle(false, *h.bodyQueue().plan, h.root()));
                widest[phase] = std::max(widest[phase], apart);
                for (int e = 2; e < 4; ++e) shortMax[phase] = std::max(shortMax[phase], h.bodyQueue().reach[e].shortfall);
            }
        }
        std::printf("%-26s W: chest lean %4.1f deg at %.2f s (fwd %5.1f, left %5.1f), pelvis %4.1f | A: chest %4.1f deg at "
                    "%.2f s (fwd %5.1f, left %5.1f), pelvis %4.1f\n",
                    v.name, lean[0], leanAt[0], leanFwd[0], leanSide[0], pelvisLean[0], lean[1], leanAt[1], leanFwd[1],
                    leanSide[1], pelvisLean[1]);
        std::printf("%-26s plan pelvis lean W %4.1f A %4.1f deg | feet apart max W %4.1f A %4.1f cm | a foot short of its "
                    "target max W %4.1f A %4.1f cm\n",
                    "", planPelvisLean[0], planPelvisLean[1], 100.0f * widest[0], 100.0f * widest[1], 100.0f * shortMax[0],
                    100.0f * shortMax[1]);
        std::printf("%-26s A after 0.5 s: plan hips height %.1f .. %.1f cm (bob %.1f) | hips ahead of the feet's centre "
                    "%.1f .. %.1f cm\n",
                    "", 100.0f * heightA.lo, 100.0f * heightA.hi, 100.0f * heightA.span(), 100.0f * leadA.lo,
                    100.0f * leadA.hi);
    }
}

// Measurement (on demand): jogging and running forward (pace 1 / 2), 4.5 s;
// the steady part (last 2 s): steps per minute, step length, the time with
// both feet off the ground (plan), the hips' bob and where it is lowest, how
// far the hips lead the feet, the real speed, and whether the body fell.
TEST_CASE("measure: jog and run" * doctest::skip()) {
    for (int pace = 1; pace <= 2; ++pace) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        const math::Vec3 fwd = h.root().transformDirection(math::kForward);
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        const int ankleL = h.rig().findJoint("foot_l"), ankleR = h.rig().findJoint("foot_r");
        const float standAnkle = std::min((*h.bodyQueue().plan)[static_cast<size_t>(ankleL)][3].y,
                                          (*h.bodyQueue().plan)[static_cast<size_t>(ankleR)][3].y);
        int steps = 0, lastFoot = -1, flight = 0, ticks = 0, contacts = 0;
        float lastPhase = 0.0f, stepLen = 0.0f, contactTime = 0.0f, landedAt[2] = {-1.0f, -1.0f};
        Range bob, lead, planLead, speed, runShare, realY, realAll;
        math::Vec3 lastHips(0.0f), lastLanding(0.0f);
        bool have = false, dropped = false, landedOnce = false;
        for (int i = 0; i < 540; ++i) {
            const float time = i * kDt;
            h.setStepInput(math::Vec2(0.0f, 1.0f));
            h.setPace(pace);
            h.tick(kDt);
            world.step(kDt);
            if (!h.supported()) dropped = true;
            if (!h.bodyQueue().plan || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
            const bool steady = time > 2.5f;
            // Landings: the step length between one landing and the next (along
            // the way), their count the cadence; a foot's contact from its
            // landing to its next step's start.
            const int foot = h.stepFoot();
            const bool restarted = foot >= 0 && foot == lastFoot && h.stepPhase() < lastPhase;
            lastPhase = h.stepPhase();
            if (foot >= 0 && (foot != lastFoot || restarted) && landedAt[foot] >= 0.0f && steady) {
                contactTime += time - landedAt[foot];
                ++contacts;
            }
            lastFoot = foot;
            for (int f = 0; f < 2; ++f) {
                if (!h.bodyQueue().landed[f]) continue;
                const math::Vec3 spot = h.root().transformPoint(h.bodyQueue().landedSpot[f]);
                if (steady && landedOnce) {
                    ++steps;
                    stepLen += math::dot(spot - lastLanding, fwd);
                }
                lastLanding = spot;
                landedOnce = true;
                landedAt[f] = time;
            }
            realAll.add(h.root().transformPoint(math::Vec3(real[0][3])).y);
            if (std::getenv("CRICKET_REAL") && pace == std::atoi(std::getenv("CRICKET_REAL")) && time > 3.0f &&
                time < 3.6f && i % 2 == 0) {
                // On demand: plan vs real body, character space (z ahead): pelvis, its
                // pitch, the ankles from their own pelvis, each thigh's and shin's pitch.
                auto pos = [&](const std::vector<math::Mat4>& m, const char* j) {
                    return math::Vec3(m[static_cast<size_t>(h.rig().findJoint(j))][3]);
                };
                auto pitch = [&](const std::vector<math::Mat4>& m, const char* a, const char* b) {
                    const math::Vec3 d = pos(m, b) - pos(m, a);
                    return math::degrees(std::atan2(d.z, -d.y));  // + : the lower end ahead
                };
                auto pelvisPitch = [&](const std::vector<math::Mat4>& m) {
                    const math::Vec3 up = math::normalize(math::Vec3(m[0][1]));
                    return math::degrees(std::atan2(up.z, up.y));  // + : leaning forward
                };
                const std::vector<math::Mat4>* both[2] = {&plan, &real};
                for (const std::vector<math::Mat4>* m : both) {
                    const std::vector<math::Mat4>& p = *m;
                    const math::Vec3 pv = pos(p, "pelvis");
                    std::printf("  t %.3f %s pelvis z %+.2f y %.2f pitch %+4.0f | ankle z L %+.2f R %+.2f y L %.2f R %.2f | thigh L "
                                "%+4.0f R %+4.0f | shin L %+4.0f R %+4.0f | support %.2f %.2f | landed %d%d | toe y L %.2f R %.2f\n",
                                time, m == &plan ? "plan" : "real", pv.z, pv.y, pelvisPitch(p), pos(p, "foot_l").z - pv.z,
                                pos(p, "foot_r").z - pv.z, pos(p, "foot_l").y, pos(p, "foot_r").y, pitch(p, "thigh_l", "shin_l"),
                                pitch(p, "thigh_r", "shin_r"), pitch(p, "shin_l", "foot_l"), pitch(p, "shin_r", "foot_r"),
                                h.bodyQueue().supportWeight[0], h.bodyQueue().supportWeight[1], h.bodyQueue().landed[0] ? 1 : 0,
                                h.bodyQueue().landed[1] ? 1 : 0, pos(p, "toe_l").y, pos(p, "toe_r").y);
                }
            }
            const char* traceFrom = std::getenv("CRICKET_FROM");
            const float from = traceFrom ? static_cast<float>(std::atof(traceFrom)) : 3.0f;
            if (std::getenv("CRICKET_TRACE") && pace == std::atoi(std::getenv("CRICKET_TRACE")) && time > from &&
                time < from + 0.7f) {  // on demand: one cycle, plan (CRICKET_TRACE=<pace> CRICKET_FROM=<s>)
                auto at = [&](const char* j) { return math::Vec3(plan[static_cast<size_t>(h.rig().findJoint(j))][3]); };
                const float lenL = math::distance(at("thigh_l"), at("foot_l")), lenR = math::distance(at("thigh_r"), at("foot_r"));
                std::printf("  t %.3f pelvis %.3f | ankle L %.3f R %.3f | leg L %.3f R %.3f | ankle z L %+.2f R %+.2f hips z "
                            "%+.2f | step foot %d %.2f | landed %d%d | run %.2f\n",
                            time, plan[0][3].y, at("foot_l").y, at("foot_r").y, lenL, lenR, at("foot_l").z, at("foot_r").z,
                            plan[0][3].z, h.stepFoot(), h.stepPhase(), h.bodyQueue().landed[0] ? 1 : 0,
                            h.bodyQueue().landed[1] ? 1 : 0, h.runShare());
                const math::Vec3 hp(plan[0][3]);
                for (int f = 0; f < 2; ++f) {
                    const motion::StepPlanner& sp = h.decider().step(f);
                    if (!sp.running()) continue;
                    std::printf("      %s phase %d share %.2f along %.2f left %.3f | target y %.3f z %+.2f | aim z "
                                "%+.2f (from hips)\n",
                                f == 0 ? "L" : "R", static_cast<int>(sp.phase()), sp.share(), sp.along(),
                                sp.timeLeft(h.decider().stepTiming(f)), sp.target().y, sp.target().z - hp.z,
                                sp.landing().z - hp.z);
                }
            }
            if (!steady) continue;
            runShare.add(h.runShare());
            ++ticks;
            const float ground = std::min(plan[static_cast<size_t>(ankleL)][3].y, plan[static_cast<size_t>(ankleR)][3].y);
            if (ground > standAnkle + 0.03f) ++flight;  // both ankles off the ground (3 cm above standing)
            bob.add(plan[0][3].y);
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(real[0][3]));
            realY.add(hips.y);
            const math::Vec3 mid = 0.5f * (h.ankle(true, real, h.root()) + h.ankle(false, real, h.root()));
            lead.add(math::dot(hips - mid, fwd));
            {
                const math::Vec3 planHips = h.root().transformPoint(math::Vec3(plan[0][3]));
                const math::Vec3 planMid = 0.5f * (h.ankle(true, plan, h.root()) + h.ankle(false, plan, h.root()));
                planLead.add(math::dot(planHips - planMid, fwd));
            }
            if (have) speed.add(math::dot(hips - lastHips, fwd) / kDt);
            lastHips = hips;
            have = true;
        }
        const float seconds = ticks * kDt;
        std::printf("pace %d (run share %.2f..%.2f): %d steps in %.1f s = %.0f steps/min, step %.2f m | contact %.3f s | "
                    "flight %.0f %% | plan hips bob %.1f cm | hips ahead of the feet %.0f .. %.0f cm | real speed %.2f .. "
                    "%.2f m/s | dropped %s\n",
                    pace, runShare.lo, runShare.hi, steps, seconds, seconds > 0 ? 60.0f * steps / seconds : 0.0f,
                    steps ? stepLen / steps : 0.0f, contacts ? contactTime / contacts : 0.0f,
                    ticks ? 100.0f * flight / ticks : 0.0f, 100.0f * bob.span(), 100.0f * lead.lo, 100.0f * lead.hi, speed.lo,
                    speed.hi, dropped ? "yes" : "no");
        std::printf("        real pelvis height: steady %.2f .. %.2f m, whole run (from standing) %.2f .. %.2f m | plan hips ahead of the feet "
                    "%.0f .. %.0f cm\n",
                    realY.lo, realY.hi, realAll.lo, realAll.hi, 100.0f * planLead.lo, 100.0f * planLead.hi);
    }
}

// Measurement (on demand): the limb test's hand pushed far ahead (0.1 m per
// press every 0.3 s, as the arrow keys do), then held: every balance step
// (time, foot, where it landed along the way) and whether the steps stop.
TEST_CASE("measure: hand far ahead, the steps settle" * doctest::skip()) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    const math::Vec3 start = h.root().position;
    math::Vec3 goal = h.wrist(true, *h.bodyQueue().plan, h.root());
    int presses = 0, held = 0;
    Range hipsHeld;
    for (int i = 0; i < 120 * 7; ++i) {
        const float time = i * kDt;
        if (presses < (std::getenv("CRICKET_PRESSES") ? std::atoi(std::getenv("CRICKET_PRESSES")) : 8) && i % 36 == 0) {
            goal += fwd * 0.1f;
            ++presses;
        }
        h.setPoseHand(true, goal);
        const int stepBefore = h.stepFoot();
        h.tick(kDt);
        world.step(kDt);
        if (stepBefore < 0 && h.stepFoot() >= 0) {
            // Why a step starts: the balance step's advice now (character space).
            const motion::StepAdvice& a = h.bodyQueue().step;
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            std::printf("  %.2f s step %d starts | capture point x %+.3f z %+.3f, outside %.3f | spot z %+.3f | ankles z "
                        "L %+.3f R %+.3f | pelvis z %+.3f | reach short hand L %.3f\n",
                        time, h.stepFoot(), a.capturePoint.x, a.capturePoint.z, a.outside, a.spot.z,
                        h.ankle(true, p, math::Transform{}).z, h.ankle(false, p, math::Transform{}).z, p[0][3].z,
                        h.bodyQueue().reach[0].shortfall);
        }
        for (int f = 0; f < 2; ++f) {
            if (!h.bodyQueue().landed[f]) continue;
            const math::Vec3 spot = h.root().transformPoint(h.bodyQueue().landedSpot[f]);
            std::printf("  %.2f s step %s lands %+.2f m along | plan hips %+.2f m | goal %+.2f m\n", time, f == 0 ? "L" : "R",
                        math::dot(spot - start, fwd),
                        math::dot(h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3])) - start, fwd),
                        math::dot(goal - start, fwd));
            if (time > 4.0f) ++held;
        }
        if (time > 4.0f) hipsHeld.add(math::dot(h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3])) - start, fwd));
    }
    std::printf("hand far ahead: steps in the last 3 s (goal held) %d | plan hips along %.2f .. %.2f m | dropped %s\n", held,
                hipsHeld.lo, hipsHeld.hi, h.supported() ? "no" : "yes");
}

// Measurement (on demand): the limb test's hips moved once (one press) by a
// set way; the sync formula: the step's foot lands when the hips arrive, the
// hips end between the feet. Prints when the plan's hips reach the goal and
// when each step starts and lands, and how far the foot and the hips went.
TEST_CASE("measure: hips pressed once, the step with the hips" * doctest::skip()) {
    struct Case {
        const char* name;
        math::Vec3 way;  // character space: +Z forward, +X left
    };
    for (const Case& c : {Case{"fwd 0.10", {0.0f, 0.0f, 0.10f}}, Case{"fwd 0.20", {0.0f, 0.0f, 0.20f}},
                          Case{"fwd 0.40", {0.0f, 0.0f, 0.40f}}, Case{"back 0.20", {0.0f, 0.0f, -0.20f}},
                          Case{"left 0.15", {0.15f, 0.0f, 0.0f}}, Case{"left 0.25", {0.25f, 0.0f, 0.0f}},
                          Case{"left 0.35", {0.35f, 0.0f, 0.0f}}, Case{"right 0.25", {-0.25f, 0.0f, 0.0f}}}) {
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
        const math::Vec3 goal = hips0 + h.root().transformDirection(c.way);
        const math::Vec3 dir = math::normalize(h.root().transformDirection(c.way));
        math::Vec3 ankle0[2];
        for (int f = 0; f < 2; ++f) ankle0[f] = h.ankle(f == 0, *h.bodyQueue().plan, h.root());
        std::printf("--- hips %s\n", c.name);
        float arrived = -1.0f;
        int lastFoot = -1;
        for (int i = 0; i < 360; ++i) {
            const float time = i * kDt;
            h.setPoseGoal(motion::Goal::Hips, goal, math::identityQuat(), false);
            h.tick(kDt);
            world.step(kDt);
            const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
            const math::Vec3 flat(hips.x - goal.x, 0.0f, hips.z - goal.z);
            if (arrived < 0.0f && math::length(flat) < 0.01f) {
                arrived = time;
                std::printf("  %.2f s plan hips within 1 cm of the goal\n", time);
            }
            const int foot = h.stepFoot();
            if (foot >= 0 && std::getenv("CRICKET_TRACE"))
                std::printf("    %.3f s foot %d share %.3f duration %.3f time left %.3f\n", time, foot,
                            h.decider().step(foot).share(), h.decider().stepTiming(foot).duration,
                            h.decider().step(foot).timeLeft(h.decider().stepTiming(foot)));
            if (foot >= 0 && foot != lastFoot) std::printf("  %.2f s step %s starts\n", time, foot == 0 ? "L" : "R");
            lastFoot = foot;
            for (int f = 0; f < 2; ++f) {
                if (!h.bodyQueue().landed[f]) continue;
                const math::Vec3 spot = h.root().transformPoint(h.bodyQueue().landedSpot[f]);
                std::printf("  %.2f s step %s lands: foot went %.1f cm along, hips %.1f cm along so far\n", time,
                            f == 0 ? "L" : "R", 100.0f * math::dot(spot - ankle0[f], dir),
                            100.0f * math::dot(hips - hips0, dir));
            }
        }
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
        const math::Vec3 l = h.ankle(true, p, h.root()), r = h.ankle(false, p, h.root());
        const math::Vec3 mid = 0.5f * (l + r);
        std::printf("  end: hips %.1f cm along; hips from the feet's middle: along %.1f cm, across %.1f cm | dropped %s\n",
                    100.0f * math::dot(hips - hips0, dir), 100.0f * math::dot(hips - mid, dir),
                    100.0f * math::dot(hips - mid, math::cross(math::kUp, dir)), h.supported() ? "no" : "yes");
        const motion::StepAdvice& a = h.bodyQueue().step;
        const math::Vec3 cp = h.root().transformPoint(a.capturePoint);
        std::printf("  end: capture point from the feet's middle along %.1f cm, outside the feet %.1f cm | ankles from "
                    "the start along L %.1f R %.1f cm\n",
                    100.0f * math::dot(cp - mid, dir), 100.0f * a.outside, 100.0f * math::dot(l - ankle0[0], dir),
                    100.0f * math::dot(r - ankle0[1], dir));
    }
}

// Measurement (on demand): fast walking, jogging and running, then letting go:
// the steps, the real speed, the widest foot slide, and how the body stops
// (the stopping steps, the hips from the feet's middle at the end).
TEST_CASE("measure: fast walk, jog, run, then stop" * doctest::skip()) {
    struct Case {
        const char* name;
        int presses, pace;  // walk pace presses (x speedStep each), 0 walk / 1 jog / 2 run
    };
    for (const Case& c : {Case{"walk x1", 0, 0}, Case{"walk fast", 6, 0}, Case{"jog", 0, 1}, Case{"run", 0, 2}}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        for (int i = 0; i < c.presses; ++i) h.changeWalkSpeed(1);
        h.setPace(c.pace);
        const math::Vec3 fwd = h.root().transformDirection(math::kForward);
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        Range speed;
        math::Vec3 lastHips(0.0f), still[2], lastPlan[2];
        bool have = false, holding[2] = {false, false};
        float slideMove = 0.0f, slideStop = 0.0f, stoppedAt = -1.0f;
        int stepsMoving = 0, stepsStopping = 0;
        constexpr float kGo = 3.0f;
        for (int i = 0; i < 120 * 6; ++i) {
            const float time = i * kDt;
            h.setStepInput(time < kGo ? math::Vec2(0.0f, 1.0f) : math::Vec2(0.0f));
            h.tick(kDt);
            world.step(kDt);
            for (int f = 0; f < 2; ++f)
                if (h.bodyQueue().landed[f]) {
                    if (time < kGo) ++stepsMoving;
                    else {
                        ++stepsStopping;
                        stoppedAt = time;
                    }
                }
            if (!h.bodyQueue().plan || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(real[0][3]));
            if (have && time > 1.5f && time < kGo) speed.add(math::dot(hips - lastHips, fwd) / kDt);
            lastHips = hips;
            for (int f = 0; f < 2; ++f) {
                const math::Vec3 p = h.ankle(f == 0, plan, h.root()), a = h.ankle(f == 0, real, h.root());
                const bool stands = have && math::length(p - lastPlan[f]) / kDt < 0.05f;
                if (stands && !holding[f]) still[f] = a;
                holding[f] = stands;
                if (stands) {
                    const float d = math::length(math::Vec3(a.x - still[f].x, 0.0f, a.z - still[f].z));
                    (time < kGo ? slideMove : slideStop) = std::max(time < kGo ? slideMove : slideStop, d);
                }
                lastPlan[f] = p;
            }
            have = true;
        }
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
        const math::Vec3 l = h.ankle(true, p, h.root()), r = h.ankle(false, p, h.root());
        const math::Vec3 mid = 0.5f * (l + r);
        std::printf("%-10s moving: %2d steps, real speed %.2f .. %.2f m/s, foot slide %.1f cm | stopping: %d steps, last "
                    "lands %.2f s after letting go, foot slide %.1f cm | end: feet %.1f cm apart along, hips from their "
                    "middle %.1f cm along | dropped %s\n",
                    c.name, stepsMoving, speed.lo, speed.hi, 100.0f * slideMove, stepsStopping,
                    stoppedAt >= 0.0f ? stoppedAt - kGo : 0.0f, 100.0f * slideStop, 100.0f * math::dot(l - r, fwd),
                    100.0f * math::dot(hips - mid, fwd), h.supported() ? "no" : "yes");
    }
}

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
TEST_CASE("measure: hand pressed 0.1 m at a time" * doctest::skip()) {
    struct Case {
        const char* name;
        math::Vec3 press;
        int presses;
    };
    for (const Case& c : {Case{"fwd", {0.0f, 0.0f, 0.1f}, 6}, Case{"fwd far", {0.0f, 0.0f, 0.1f}, 12},
                          Case{"down", {0.0f, -0.1f, 0.0f}, 6},
                          Case{"out", {0.1f, 0.0f, 0.0f}, 5}, Case{"across", {-0.1f, 0.0f, 0.0f}, 5},
                          Case{"up", {0.0f, 0.1f, 0.0f}, 6}}) {
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
        std::printf("--- hand L %s, %d presses of 0.1 m\n", c.name, c.presses);
        int steps = 0;
        for (int i = 0; i < 60 * (c.presses + 3); ++i) {
            const float time = i * kDt;
            if (i % 60 == 0 && i / 60 < c.presses) goal += h.root().transformDirection(c.press);
            h.setPoseHand(true, goal);
            const int before = h.stepFoot();
            h.tick(kDt);
            world.step(kDt);
            if (before < 0 && h.stepFoot() >= 0) {
                const motion::StepAdvice& a = h.bodyQueue().step;
                std::printf("  %.2f s (press %d) step %s starts: capture point outside %.1f cm, spot from its foot %.1f cm, "
                            "reach short %.1f cm\n",
                            time, std::min(i / 60 + 1, c.presses), h.stepFoot() == 0 ? "L" : "R", 100.0f * a.outside,
                            100.0f * math::distance(a.spot, h.ankle(h.stepFoot() == 0, *h.bodyQueue().plan, math::Transform{})),
                            100.0f * h.bodyQueue().reach[0].shortfall);
            }
            for (int f = 0; f < 2; ++f) steps += h.bodyQueue().landed[f] ? 1 : 0;
        }
        std::printf("  %d steps | hand miss %.1f cm | dropped %s\n", steps,
                    100.0f * math::distance(h.wrist(true, *h.bodyQueue().plan, h.root()), goal), h.supported() ? "no" : "yes");
    }
}

// Measurement (on demand): the user's report (2026-11-15): the left hand
// pressed down, then forward (0.1 m a press, as in the game): steps go back
// and forth. Prints each step's cause and the plan's hips, centre of mass
// and hand through it.
TEST_CASE("measure: hand down then forward, the steps" * doctest::skip()) {
    const int downs = std::getenv("CRICKET_DOWNS") ? std::atoi(std::getenv("CRICKET_DOWNS")) : 7;
    const int fwds = std::getenv("CRICKET_FWDS") ? std::atoi(std::getenv("CRICKET_FWDS")) : 6;
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    const math::Vec3 start = h.root().position;
    math::Vec3 goal = h.wrist(true, *h.bodyQueue().plan, h.root());
    const int presses = downs + fwds;
    for (int i = 0; i < 60 * (presses + 6); ++i) {
        const float time = i * kDt;
        if (i % 60 == 0 && i / 60 < presses) {
            const bool down = i / 60 < downs;
            goal += h.root().transformDirection(down ? math::Vec3(0.0f, -0.1f, 0.0f) : math::Vec3(0.0f, 0.0f, 0.1f));
            goal.y = std::max(goal.y, 0.05f);
            std::printf("%.2f s press %s: goal %.2f m along, %.2f m up\n", time, down ? "down" : "fwd",
                        math::dot(goal - start, fwd), goal.y);
        }
        h.setPoseHand(true, goal);
        const int before = h.stepFoot();
        h.tick(kDt);
        world.step(kDt);
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        auto along = [&](math::Vec3 v) { return 100.0f * math::dot(v - start, fwd); };
        const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
        const math::Vec3 com = h.root().transformPoint(h.decider().planCentreOfMass());
        const math::Vec3 l = h.ankle(true, p, h.root()), r = h.ankle(false, p, h.root());
        if (before < 0 && h.stepFoot() >= 0) {
            const motion::StepAdvice& a = h.bodyQueue().step;
            std::printf("  %.2f s step %s starts: spot %.1f | capture %.1f, outside %.1f cm | hips %.1f, CoM %.1f | feet L %.1f "
                        "R %.1f | hand short %.1f cm\n",
                        time, h.stepFoot() == 0 ? "L" : "R", along(h.root().transformPoint(a.spot)),
                        along(h.root().transformPoint(a.capturePoint)), 100.0f * a.outside, along(hips), along(com),
                        along(l), along(r), 100.0f * h.bodyQueue().reach[0].shortfall);
        }
        for (int f = 0; f < 2; ++f)
            if (h.bodyQueue().landed[f])
                std::printf("  %.2f s step %s lands | hips %.1f, CoM %.1f\n", time, f == 0 ? "L" : "R", along(hips), along(com));
        if (i % 30 == 0)
            std::printf("    %.2f s hips %.1f (%.1f cm up), CoM %.1f | feet L %.1f R %.1f | hand %.1f, short %.1f cm\n", time,
                        along(hips), 100.0f * hips.y, along(com), along(l), along(r),
                        along(h.wrist(true, p, h.root())), 100.0f * h.bodyQueue().reach[0].shortfall);
    }
    std::printf("dropped %s\n", h.supported() ? "no" : "yes");
}

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
TEST_CASE("measure: foot moved on the ground, it steps" * doctest::skip()) {
    const int presses = std::getenv("CRICKET_PRESSES") ? std::atoi(std::getenv("CRICKET_PRESSES")) : 4;
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
    const math::Vec3 hips0 = h.root().transformPoint(math::Vec3((*h.bodyQueue().plan)[0][3]));
    math::Vec3 goal = a0;
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    for (int i = 0; i < 60 * (presses + 3); ++i) {
        const float time = i * kDt;
        if (i % 60 == 0 && i / 60 < presses) goal += fwd * 0.1f;
        h.setPoseFoot(motion::Foot::Left, goal);
        const int before = h.stepFoot();
        h.tick(kDt);
        world.step(kDt);
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        auto along = [&](math::Vec3 v, math::Vec3 o) { return 100.0f * math::dot(v - o, fwd); };
        const math::Vec3 hips = h.root().transformPoint(math::Vec3(p[0][3]));
        if (before < 0 && h.stepFoot() >= 0) std::printf("  %.2f s step %s starts\n", time, h.stepFoot() == 0 ? "L" : "R");
        for (int f = 0; f < 2; ++f)
            if (h.bodyQueue().landed[f])
                std::printf("  %.2f s step %s lands | hips %.1f cm along\n", time, f == 0 ? "L" : "R", along(hips, hips0));
        if (i % 30 == 29 && h.physicalModels(world, 1.0f, real)) {
            const math::Vec3 lp = h.ankle(true, p, h.root()), lr = h.ankle(true, real, h.root());
            const math::Vec3 rp = h.ankle(false, p, h.root());
            std::printf("    %.2f s goal %.1f | left ankle plan %.1f real %.1f (up %.1f) | right %.1f | hips plan %.1f, %.1f cm "
                        "from the feet's middle | dropped %s\n",
                        time, along(goal, a0), along(lp, a0), along(lr, a0), 100.0f * (lr.y - a0.y), along(rp, a0),
                        along(hips, hips0), along(hips, 0.5f * (lp + rp)), h.supported() ? "no" : "yes");
        }
    }
}

// Measurement (on demand): the limb test's left foot held up, then put down
// on the ground elsewhere: it lands there by a step (never jumps there).
TEST_CASE("measure: foot held up, put down ahead, it steps down" * doctest::skip()) {
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    const math::Vec3 a0 = h.ankle(true, *h.bodyQueue().plan, h.root());
    math::Vec3 last = a0;
    float worst = 0.0f, worstAt = 0.0f;
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    for (int i = 0; i < 240; ++i) {
        const float time = i * kDt;
        const math::Vec3 goal = time < 0.8f ? a0 + math::Vec3(0.0f, 0.15f, 0.0f) : a0 + fwd * 0.2f;
        h.setPoseFoot(motion::Foot::Left, goal);
        const int before = h.stepFoot();
        h.tick(kDt);
        world.step(kDt);
        const math::Vec3 a = h.ankle(true, *h.bodyQueue().plan, h.root());
        if (time > 0.8f && math::distance(a, last) > worst) {
            worst = math::distance(a, last);
            worstAt = time;
        }
        last = a;
        if (before < 0 && h.stepFoot() >= 0) std::printf("  %.2f s step %s starts\n", time, h.stepFoot() == 0 ? "L" : "R");
        for (int f = 0; f < 2; ++f)
            if (h.bodyQueue().landed[f]) std::printf("  %.2f s step %s lands\n", time, f == 0 ? "L" : "R");
    }
    REQUIRE(h.physicalModels(world, 1.0f, real));
    const math::Vec3 a = h.ankle(true, *h.bodyQueue().plan, h.root()), r = h.ankle(true, real, h.root());
    std::printf("put down: biggest plan ankle move in a tick %.1f cm at %.2f s | end plan %.1f cm along (up %.1f), real %.1f "
                "(up %.1f) | dropped %s\n",
                100.0f * worst, worstAt, 100.0f * math::dot(a - a0, fwd), 100.0f * (a.y - a0.y), 100.0f * math::dot(r - a0, fwd),
                100.0f * (r.y - a0.y), h.supported() ? "no" : "yes");
}

// Measurement (on demand): the user's report (2026-11-15): walking after the
// limb test. The left hand (or foot: CRICKET_FOOT) pressed forward 3 times,
// then R (clearPose + resetPhysicsBody, as the game), then W for 3 s.
TEST_CASE("measure: walk after the limb test" * doctest::skip()) {
    const bool foot = std::getenv("CRICKET_FOOT") != nullptr;
    const bool reset = std::getenv("CRICKET_NORESET") == nullptr;
    core::Config pw;
    REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
    physics::PhysicsWorld world;
    REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
    agents::Human h;
    REQUIRE(makeLiveHuman(h));
    REQUIRE(h.attachPhysics(world));
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    math::Vec3 goal = foot ? h.ankle(true, *h.bodyQueue().plan, h.root()) : h.wrist(true, *h.bodyQueue().plan, h.root());
    for (int i = 0; i < 180; ++i) {
        if (i % 60 == 0) goal += fwd * 0.1f;
        if (foot) h.setPoseFoot(motion::Foot::Left, goal);
        else h.setPoseHand(true, goal);
        h.tick(kDt);
        world.step(kDt);
    }
    if (reset) {
        h.clearPose();
        h.resetPhysicsBody();
    }
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    Range speed, chest;
    int steps = 0;
    math::Vec3 last = h.root().position;
    const int chestJ = h.rig().findJoint("chest");
    for (int i = 0; i < 360; ++i) {
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        for (int f = 0; f < 2; ++f) steps += h.bodyQueue().landed[f] ? 1 : 0;
        if (!h.physicalModels(world, 1.0f, real)) continue;
        const math::Vec3 hips = h.root().transformPoint(math::Vec3(real[0][3]));
        if (i > 120) speed.add(math::dot(hips - last, fwd) / kDt);
        last = hips;
        const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(chestJ)][1]));
        if (i > 60) chest.add(math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f))));
    }
    std::printf("walk after the limb test (%s, %s): %d steps in 3 s, real speed %.2f .. %.2f m/s, chest lean %.1f .. %.1f deg, "
                "dropped %s\n",
                foot ? "foot" : "hand", reset ? "R pressed" : "no R", steps, speed.lo, speed.hi, chest.lo, chest.hi,
                h.supported() ? "no" : "yes");
}

// Measurement (on demand): every pace from the same body mechanism (the sync
// formula in the decider): walk (presses x speedStep), jog (pace 1), run
// (pace 2). Steady part 2.5-5 s: cadence from the landings' times, step
// length between landings, real mean speed, contact, flight, bob (plan and
// real), chest lean, the widest real foot slide while its plan stands.
TEST_CASE("measure: gaits by speed" * doctest::skip()) {
    struct Case {
        const char* name;
        int presses, pace;
    };
    for (const Case& c : {Case{"walk", 0, 0}, Case{"walk +3", 3, 0}, Case{"walk +6", 6, 0}, Case{"jog", 0, 1},
                          Case{"run", 0, 2}}) {
        core::Config pw;
        REQUIRE(pw.load(CRICKET_CONFIG_DIR "/physics_world.json"));
        physics::PhysicsWorld world;
        REQUIRE(world.init(physics::readWorldSettings(pw), physics::readPhysicsSettings(pw)));
        agents::Human h;
        REQUIRE(makeLiveHuman(h));
        REQUIRE(h.attachPhysics(world));
        constexpr float kDt = 1.0f / 120.0f;
        for (int i = 0; i < 60; ++i) { h.tick(kDt); world.step(kDt); }
        for (int i = 0; i < c.presses; ++i) h.changeWalkSpeed(1);
        h.setPace(c.pace);
        if (std::getenv("CRICKET_STRENGTH"))  // measuring: every muscle this many times its strength
            world.setRagdollStrength(h.humanBody().ragdoll(), static_cast<float>(std::atof(std::getenv("CRICKET_STRENGTH"))));
        const math::Vec3 fwd = h.root().transformDirection(math::kForward);
        std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
        const int chestJ = h.rig().findJoint("chest");
        const int ankleL = h.rig().findJoint("foot_l"), ankleR = h.rig().findJoint("foot_r");
        const float standAnkle = std::min((*h.bodyQueue().plan)[static_cast<size_t>(ankleL)][3].y,
                                          (*h.bodyQueue().plan)[static_cast<size_t>(ankleR)][3].y);
        Range bobPlan, bobReal;
        float firstLand = -1.0f, lastLand = -1.0f, stepSum = 0.0f, chest = 0.0f, slide = 0.0f, contactSum = 0.0f;
        int lands = 0, ticks = 0, flight = 0, contacts = 0;
        float landedAt[2] = {-1.0f, -1.0f};
        bool wasStepping[2] = {false, false};
        math::Vec3 lastSpot(0.0f), hipsStart(0.0f), still[2], lastPlan[2];
        bool haveSpot = false, haveStart = false, holding[2] = {false, false}, havePlan = false, dropped = false;
        float tStart = 0.0f, tEnd = 0.0f;
        math::Vec3 hipsEnd(0.0f);
        for (int i = 0; i < 600; ++i) {
            const float time = i * kDt;
            h.setStepInput(math::Vec2(0.0f, 1.0f));
            h.tick(kDt);
            world.step(kDt);
            if (!h.supported()) dropped = true;
            const bool steady = time > 2.5f;
            for (int f = 0; f < 2; ++f) {
                const bool stepping = h.decider().step(f).running();
                if (steady && stepping && !wasStepping[f] && landedAt[f] >= 0.0f) {
                    contactSum += time - landedAt[f];
                    ++contacts;
                }
                wasStepping[f] = stepping;
                if (!h.bodyQueue().landed[f]) continue;
                landedAt[f] = time;
                const math::Vec3 spot = h.root().transformPoint(h.bodyQueue().landedSpot[f]);
                if (steady) {
                    if (firstLand < 0.0f) firstLand = time;
                    else if (haveSpot) stepSum += math::dot(spot - lastSpot, fwd);
                    lastLand = time;
                    ++lands;
                }
                lastSpot = spot;
                haveSpot = true;
            }
            if (!h.bodyQueue().plan || !h.physicalModels(world, 1.0f, real)) continue;
            const std::vector<math::Mat4>& plan = *h.bodyQueue().plan;
            const math::Vec3 hips = h.root().transformPoint(math::Vec3(real[0][3]));
            for (int f = 0; f < 2; ++f) {
                const math::Vec3 p = h.ankle(f == 0, plan, h.root()), a = h.ankle(f == 0, real, h.root());
                const bool stands = havePlan && math::length(p - lastPlan[f]) / kDt < 0.05f;
                if (stands && !holding[f]) still[f] = a;
                holding[f] = stands;
                if (stands && steady) slide = std::max(slide, math::length(math::Vec3(a.x - still[f].x, 0.0f, a.z - still[f].z)));
                lastPlan[f] = p;
            }
            havePlan = true;
            if (!steady) continue;
            if (!haveStart) {
                hipsStart = hips;
                tStart = time;
                haveStart = true;
            }
            hipsEnd = hips;
            tEnd = time;
            ++ticks;
            const float ground = std::min(plan[static_cast<size_t>(ankleL)][3].y, plan[static_cast<size_t>(ankleR)][3].y);
            if (ground > standAnkle + 0.03f) ++flight;
            bobPlan.add(plan[0][3].y);
            bobReal.add(hips.y);
            const math::Vec3 up = math::normalize(math::Vec3(real[static_cast<size_t>(chestJ)][1]));
            chest = std::max(chest, math::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f))));
        }
        const float speed = tEnd > tStart ? math::dot(hipsEnd - hipsStart, fwd) / (tEnd - tStart) : 0.0f;
        const float cadence = lands > 1 && lastLand > firstLand ? 60.0f * (lands - 1) / (lastLand - firstLand) : 0.0f;
        std::printf("%-8s speed %.2f m/s | %3.0f steps/min, step %.2f m | contact %.2f s | flight %2.0f %% | bob plan %.1f real "
                    "%.1f cm | chest lean %.1f deg | foot slide %.1f cm | run share %.2f | dropped %s\n",
                    c.name, speed, cadence, lands > 1 ? stepSum / (lands - 1) : 0.0f, contacts ? contactSum / contacts : 0.0f,
                    ticks ? 100.0f * flight / ticks : 0.0f, 100.0f * bobPlan.span(), 100.0f * bobReal.span(), chest,
                    100.0f * slide, h.runShare(), dropped ? "yes" : "no");
    }
}

// Measurement (on demand): where the real swing foot falls behind its plan
// (pace via CRICKET_PACE 0/1/2, walk presses CRICKET_PRESSES). For each swing
// of the left foot in the steady part: the real ankle's lag behind the plan
// (along the way, and up) at lift-off, mid-swing, landing, and 0.1 s after;
// the plan's and the real ankle's top speed; and the hip and knee joints'
// angle gap (plan minus real) at mid-swing.
TEST_CASE("measure: swing foot, plan against real" * doctest::skip()) {
    const int pace = std::getenv("CRICKET_PACE") ? std::atoi(std::getenv("CRICKET_PACE")) : 1;
    const int presses = std::getenv("CRICKET_PRESSES") ? std::atoi(std::getenv("CRICKET_PRESSES")) : 0;
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
    h.setPace(pace);
    if (std::getenv("CRICKET_STRENGTH"))  // measuring: every muscle this many times its strength
        world.setRagdollStrength(h.humanBody().ragdoll(), static_cast<float>(std::atof(std::getenv("CRICKET_STRENGTH"))));
    const math::Vec3 fwd = h.root().transformDirection(math::kForward);
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    const int thigh = h.rig().findJoint("thigh_l"), shin = h.rig().findJoint("shin_l"), ankle = h.rig().findJoint("foot_l");
    bool was = false;
    float start = 0.0f, planTop = 0.0f, realTop = 0.0f, afterAt = -1.0f;
    math::Vec3 lastP(0.0f), lastR(0.0f);
    bool printedMid = false;
    int swings = 0;
    auto lag = [&](const char* when) {
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const math::Vec3 a = h.ankle(true, p, h.root()), r = h.ankle(true, real, h.root());
        const math::Vec3 pp = h.root().transformPoint(math::Vec3(p[0][3])), pr = h.root().transformPoint(math::Vec3(real[0][3]));
        std::printf("    %-9s real behind plan %5.1f cm, below %5.1f cm | pelvis: real behind plan %5.1f cm, below %5.1f cm\n",
                    when, 100.0f * math::dot(a - r, fwd), 100.0f * (a.y - r.y), 100.0f * math::dot(pp - pr, fwd),
                    100.0f * (pp.y - pr.y));
    };
    auto angle = [&](const std::vector<math::Mat4>& m, int a, int b, int c) {
        const math::Vec3 pa(m[static_cast<size_t>(a)][3]), pb(m[static_cast<size_t>(b)][3]), pc(m[static_cast<size_t>(c)][3]);
        const math::Vec3 d1 = math::normalize(pb - pa), d2 = math::normalize(pc - pb);
        return math::degrees(std::acos(std::clamp(math::dot(d1, d2), -1.0f, 1.0f)));
    };
    auto pitch = [&](const std::vector<math::Mat4>& m, int a, int b) {
        const math::Vec3 d = math::Vec3(m[static_cast<size_t>(b)][3]) - math::Vec3(m[static_cast<size_t>(a)][3]);
        return math::degrees(std::atan2(math::dot(d, math::inverse(h.root()).transformDirection(fwd)), -d.y));
    };
    for (int i = 0; i < 600 && swings < 3; ++i) {
        const float time = i * kDt;
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        if (!h.physicalModels(world, 1.0f, real)) continue;
        const std::vector<math::Mat4>& p = *h.bodyQueue().plan;
        const math::Vec3 a = h.ankle(true, p, h.root()), r = h.ankle(true, real, h.root());
        const motion::StepPlanner& sp = h.decider().step(0);
        const bool swinging = sp.running() && sp.phase() == motion::StepPlanner::Phase::Up;
        if (time > 2.5f) {
            if (swinging && !was) {
                start = time;
                planTop = realTop = 0.0f;
                printedMid = false;
                std::printf("  swing %d at %.2f s (%.2f s long)\n", ++swings, time, sp.timeLeft(h.decider().stepTiming(0)));
                lag("lift-off");
            }
            if (swinging) {
                planTop = std::max(planTop, math::length(a - lastP) / kDt);
                realTop = std::max(realTop, math::length(r - lastR) / kDt);
                if (!printedMid && sp.along() >= 0.5f) {
                    lag("mid-swing");
                    std::printf("              thigh pitch plan %5.1f real %5.1f deg | knee plan %5.1f real %5.1f deg\n",
                                pitch(p, thigh, shin), pitch(real, thigh, shin), angle(p, thigh, shin, ankle),
                                angle(real, thigh, shin, ankle));
                    printedMid = true;
                }
            }
            if (!swinging && was) {
                lag("landing");
                std::printf("              ankle top speed plan %.1f real %.1f m/s (%.2f s swing)\n", planTop, realTop, time - start);
                afterAt = time + 0.1f;
            }
            if (afterAt > 0.0f && time >= afterAt) {
                lag("+0.1 s");
                afterAt = -1.0f;
            }
        }
        was = swinging;
        lastP = a;
        lastR = r;
    }
}


// Measurement (on demand): the left hip through one swing at a pace
// (CRICKET_PACE 0/1/2, walk presses CRICKET_PRESSES): the thigh's pitch, plan
// against real, the hip motor's torque against its limit, and the torque its
// range's limits took: is the swing short of strength, of range, or neither.
TEST_CASE("measure: hip through a swing" * doctest::skip()) {
    const int pace = std::getenv("CRICKET_PACE") ? std::atoi(std::getenv("CRICKET_PACE")) : 2;
    const int presses = std::getenv("CRICKET_PRESSES") ? std::atoi(std::getenv("CRICKET_PRESSES")) : 0;
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
    h.setPace(pace);
    if (std::getenv("CRICKET_STRENGTH"))  // measuring: every muscle this many times its strength
        world.setRagdollStrength(h.humanBody().ragdoll(), static_cast<float>(std::atof(std::getenv("CRICKET_STRENGTH"))));
    const math::Vec3 fwdW = h.root().transformDirection(math::kForward);
    std::vector<math::Mat4> real(static_cast<size_t>(h.rig().numJoints()));
    const int thigh = h.rig().findJoint("thigh_l"), shin = h.rig().findJoint("shin_l");
    const int part = h.humanBody().partOf(thigh);
    REQUIRE(part >= 0);
    auto pitch = [&](const std::vector<math::Mat4>& m) {
        const math::Vec3 d = math::Vec3(m[static_cast<size_t>(shin)][3]) - math::Vec3(m[static_cast<size_t>(thigh)][3]);
        const math::Vec3 f = math::inverse(h.root()).transformDirection(fwdW);
        return math::degrees(std::atan2(math::dot(d, f), -d.y));  // + : the knee ahead of the hip
    };
    int swings = 0;
    bool was = false;
    for (int i = 0; i < 600 && swings < 3; ++i) {
        const float time = i * kDt;
        h.setStepInput(math::Vec2(0.0f, 1.0f));
        h.tick(kDt);
        world.step(kDt);
        const motion::StepPlanner& sp = h.decider().step(0);
        const bool swinging = sp.running() && sp.phase() == motion::StepPlanner::Phase::Up;
        if (time > 2.5f && swinging && !was) ++swings;
        was = swinging;
        if (time < 2.5f || swings != 2 || i % 2 != 0 || !h.physicalModels(world, 1.0f, real)) continue;
        physics::PhysicsWorld::RagdollJointState st, knee;
        if (!world.ragdollJointState(h.humanBody().ragdoll(), part, kDt, st)) continue;
        if (std::getenv("CRICKET_KNEE") && world.ragdollJointState(h.humanBody().ragdoll(), h.humanBody().partOf(shin), kDt, knee)) {
            const int ankleJ = h.rig().findJoint("foot_l");
            auto kneeAngle = [&](const std::vector<math::Mat4>& m) {
                const math::Vec3 a(m[static_cast<size_t>(thigh)][3]), b(m[static_cast<size_t>(shin)][3]), c(m[static_cast<size_t>(ankleJ)][3]);
                return math::degrees(std::acos(std::clamp(math::dot(math::normalize(b - a), math::normalize(c - b)), -1.0f, 1.0f)));
            };
            std::printf("  %.3f s %s along %.2f | thigh plan %6.1f real %6.1f | knee plan %6.1f real %6.1f deg | knee motor x %6.1f y "
                        "%6.1f z %6.1f (limit twist %5.1f swing %5.1f)\n",
                        time, swinging ? "swing " : "stance", sp.along(), pitch(*h.bodyQueue().plan), pitch(real),
                        kneeAngle(*h.bodyQueue().plan), kneeAngle(real), knee.motor.x, knee.motor.y, knee.motor.z, knee.twistLimit,
                        knee.swingLimit);
            continue;
        }
        std::printf("  %.3f s %s along %.2f | thigh pitch plan %6.1f real %6.1f deg | motor x %6.1f y %6.1f z %6.1f N m (limit "
                    "twist %5.1f swing %5.1f) | range x %6.1f y %6.1f z %6.1f\n",
                    time, swinging ? "swing " : "stance", sp.along(), pitch(*h.bodyQueue().plan), pitch(real), st.motor.x,
                    st.motor.y, st.motor.z, st.twistLimit, st.swingLimit, st.range.x, st.range.y, st.range.z);
    }
}
