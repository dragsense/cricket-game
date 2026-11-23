#pragma once
// The body's decider (DECISIONS 2026-11-06, reworked 2026-11-10 "Whole-body
// decider on targets"). Given the seven targets (TargetSet) each tick it
// decides how every body part moves: which part, when, how much, the way a
// real body adapts, and gives the muscles a stable, smooth target pose. It
// knows nothing of actions (a step, a walk, a reach are target paths the
// planner makes). It never moves a part itself and never reads the physics
// body back each tick (a plan from the lagging body fed the lag back and
// jumped from tick to tick, measured 2026-11-08): it is seeded from the real
// pose once (seed) and from then on carries its own plan forward.
//
// Each tick, in priority order, on one desired pose:
//  1 Pinned feet: anchors where they stand (never slide).
//  2 Support: a weight per foot (0..1), blended over supportBlendTime, never
//    switched; the hips' reference over the weighted support feet (Winter
//    1995's load/unload), at standing height.
//  3 Driven targets, by their blended weights (mode changes blend in and out
//    over blendTime; a path starts from where the plan is).
//  4 Reach: limbs by two-bone IK (reach clamped to the limb's span with
//    the knee or elbow at limbMinBend: never fully straight); a hand or lifted
//    foot out of reach brings the trunk in, in real body order with shares
//    by place (BIOMECHANICS 9.1, 9.3; Esola 1996's lumbopelvic rhythm);
//    still short, it is reported (reach[]), never snapped.
//  5 Posture: the chest back toward upright, the head level in the world
//    (Pozzo 1990), free arms swinging to cancel the body's turning (Collins
//    2009) and settling to their rest in the chest's frame.
// Then the output pose follows the desired one with bounded speed and
// acceleration (the hips), and bounded joint rates (trunk, head, limbs):
// commanded motion is bounded by muscle activation, so nothing in the plan
// can jump. The planted legs are solved last, on the output pelvis, so a
// pinned foot stays put whatever the hips do. Hard constraint on the output
// pelvis: no higher than every grounded leg reaches.
// No allocations after init().

#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/arm_fit.h"
#include "motion/step_planner.h"
#include "motion/target_set.h"
#include "rig/rig.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ik {
class JointLimits;
}

namespace motion {

enum class Goal { HandL, HandR, FootL, FootR, Hips, Chest, Head };
enum class PartMove { Turn, Move };

// One part of a reach queue: which way it goes, never how far (the plan
// holds how far). Kept for tests and debugging: the muscles follow the plan.
struct PartOrder {
    int joint = -1;                      // rig joint (the pelvis for the hips)
    PartMove kind = PartMove::Turn;      // turn about `direction`, or move along it (hips only)
    math::Vec3 direction{0.0f};          // unit, character space: rotation axis (right-hand rule) or travel direction
    math::Vec3 targetVelocity{0.0f};     // the target's own speed (m/s for a move or point target, rad/s for a turn target)
    Goal goal = Goal::HandL;             // the target it serves
    bool active = false;                 // it moves now
    float share = 1.0f;                  // its share of a motion its queue carries together (a queue's shares sum to 1)
};

// A reach end's state: how far short of its target it stays once the body
// has given what it can (the planner then shifts support or steps).
struct ReachReport {
    bool driven = false;    // the end has a target this tick
    bool reached = true;    // within the clamped reach
    float shortfall = 0.0f; // m still to go (0 when reached)
    math::Vec3 direction{0.0f};  // unit, from the end toward its target (character space)
};

// The balance step (the capture point): where the body's motion takes its
// centre of mass, a foot steps there. Whatever moved the body (a hips
// target, a push, a reach) the same rule: more motion, a bigger step or
// more steps, any direction. The decider makes the step itself (its own
// StepPlanner: a foot moves only by a step, never slides).
struct StepAdvice {
    bool wanted = false;             // a step is needed (a step under way: its spot now)
    int foot = -1;                   // 0 left, 1 right
    math::Vec3 spot{0.0f};           // where its ankle should land (character space)
    math::Vec3 capturePoint{0.0f};   // the extrapolated centre of mass now, on the ground
    math::Vec3 velocity{0.0f};       // m/s, the body's level motion it comes from
    float outside = 0.0f;            // m the capture point is out of the support (0: inside)
    // s the step takes to land: the hips' path's time (the sync formula: the
    // foot lands as the hips arrive); negative: the gait's for the speed.
    float time = -1.0f;
};

struct BodyQueue {
    static constexpr int kCapacity = 48;
    static constexpr int kMaxJoints = 128;
    std::array<PartOrder, kCapacity> orders{};
    int count = 0;
    // The output pose: every rig joint's bone (character space; valid until
    // the next decide) and which joints the decider planned (non-zero).
    const std::vector<math::Mat4>* plan = nullptr;
    std::array<uint8_t, kMaxJoints> planned{};
    // What the decider decided this tick, for the planner and debug drawing.
    float supportWeight[2] = {1.0f, 1.0f};  // left, right foot
    math::Vec3 hipsReference{0.0f};         // where support puts the hips (character space)
    bool heightClamped = false;             // a grounded leg holds the hips lower than wanted
    ReachReport reach[4];                   // hand L, hand R, foot L, foot R
    StepAdvice step;                        // the balance step (none: wanted false)
    // A step landed this tick, per foot (left, right), and where (ankle,
    // character space); the planner keeps the foot there from now on.
    bool landed[2] = {false, false};
    math::Vec3 landedSpot[2]{};
};

struct BodyDeciderJoints {
    int pelvis = -1;
    int spine[3] = {-1, -1, -1};  // spine_01, spine_02, chest
    int neck = -1, head = -1;
    int clavicles[2] = {-1, -1};
    ik::TwoBoneChain legs[2];  // left, right: thigh, shin, foot
    ik::TwoBoneChain arms[2];  // left, right: upper arm, forearm, hand
};

// human_joints.json "decider" (see the file for the sources).
struct BodyDeciderSettings {
    math::Vec3 kneePole{0.0f, 0.0f, 1.0f};    // knees point this way (pelvis frame)
    // The measured lumbopelvic rhythm (Esola et al. 1996): lumbar : hip ratio
    // per band of trunk flexion (upper bounds, rad).
    float lumbarHipBand[3] = {0.5236f, 1.0472f, 1.5708f};  // 30, 60, 90 deg
    float lumbarHipRatio[3] = {1.9f, 0.9f, 0.4f};
    // m/s^2 down, from the physics world: a free arm hangs on it as a
    // pendulum. 0: free arms only cancel the turning.
    float gravity = 0.0f;
    float armCancel = 0.63f;  // the share of the body's turning the free arms cancel
    // The pelvis drops on the side of the leg without weight, about the hip
    // that carries it (pelvic list), this much (rad) with all the weight on
    // one leg, by the weight's share (the weight shift: it leads a lift).
    float pelvisList = 0.0698f;  // 4 deg
    // The chest's turn against the pelvis stays within this (rad) each way:
    // the thoracic turn's range (BIOMECHANICS 2); decider.trunkTurn.maxDeg.
    float trunkMaxTwist = 0.5236f;  // 30 deg
    // A Driven foot this close (m) above where it was pinned still stands on
    // the ground (it has not lifted yet: the weight shift before a step).
    float onGround = 0.02f;
    // A limb reaches at most its span with the knee or elbow bent this much
    // (rad): quiet standing knee 0-5 deg (BIOMECHANICS 2); the two-bone
    // solve's soften keeps it from snapping.
    float limbMinBend = 0.0873f;  // 5 deg
    // The chain rule (body_chain.h): a joint past this share of its range
    // hands the rest on to the next one, smoothly (the trunk joins a reach
    // at ~0.8-0.9 of the arm's, Moriyama 2022).
    float chainOnset = 0.8f;
    // s: a target's mode or weight change blends over this; a foot's support
    // weight over supportBlendTime (the weight shift of a step).
    float blendTime = 0.1f, supportBlendTime = 0.12f;
    // The output pose's bounds: the hips' speed (m/s) and acceleration
    // (m/s^2), their turn rate, the trunk's, the head's and a limb's joint
    // rates (rad/s).
    float hipsMaxSpeed = 2.0f, hipsMaxAccel = 8.0f, hipsMaxTurnRate = 4.0f;
    float trunkMaxRate = 4.0f, headMaxRate = 6.0f, limbMaxRate = 12.0f;
    // A lifted leg's joint rate (rad/s): a fast swing's knee is far faster
    // than a reach's joints.
    float legMaxRate = 20.0f;
    // The lowest hips: no grounded knee bends past this (rad) on two feet
    // (the semi-squat, BIOMECHANICS 9.4) or on one (the largest mean stance
    // knee of a maximal one-leg foot reach, 9.7).
    float twoLegKneeMax = 1.5708f, oneLegKneeMax = 1.1694f;  // 90, 67 deg
    // A reach's pelvis tilt from standing, rotation vector (rad, character
    // space): forward tilt (hip flexion, +X), turn (Y), side tilt (Z);
    // decider.pelvisMinDeg / pelvisMaxDeg (BIOMECHANICS 9.3).
    math::Vec3 pelvisMin{-0.1745f, -0.5236f, -0.3491f}, pelvisMax{1.3963f, 0.5236f, 0.3491f};
    float stepMargin = 0.02f;       // m
    float footHalfWidth = 0.045f;   // m, the foot's outline: half its width
    float footHalfLength = 0.11f;   // m, half its length
    float footCentreAhead = 0.05f;  // m, the foot's centre ahead of the ankle
    float landingBehind = 0.05f;    // m
    float minFootGap = 0.1f;        // m
    float maxStepForward = 0.6f, maxStepSide = 0.5f;  // m
    // The decider's own step: its shape (locomotion.json "step": height,
    // apex, toe-off, ball of the foot; its timing is decided per step).
    StepPlannerSettings stepShape;
};

class BodyDecider {
public:
    // Reads the rest pose's limb lengths and hips. `limits` (kept): the
    // joints' real ranges the plan stays within (none: unlimited). Load-time
    // only. The plan starts standing (rest); seed() restarts it elsewhere.
    bool init(const rig::Rig& rig, const BodyDeciderJoints& joints, const ik::JointLimits* limits = nullptr);
    // The body's mass per rig joint (kg, 0: none) and its centre in the
    // bone's frame: the plan keeps its centre of mass over the feet when
    // standing, and the free arms swing by their mass. Load-time only.
    void setMasses(const std::vector<float>& mass, const std::vector<math::Vec3>& centre);
    // Restarts the plan from a real pose (character space): a reset, a
    // get-up from the landed pose. The feet pin where they are, the hips
    // start at rest (no speed), every blend starts over. No allocations.
    void seed(const rig::Pose& pose);
    // The character root moved (`toNew`: old character space to new): the
    // plan (feet, hips, everything) stays put in the world.
    void rebase(const math::Transform& toNew);
    math::Vec3 planCentreOfMass() const;
    // For measuring: the desired pelvis (before the bounded motion) and the
    // output hips' velocity (character space).
    math::Vec3 outputHipsVelocity() const { return hipsVel_; }

    // One tick: from the targets and the plan so far, fills `out`. No
    // allocations.
    void decide(const TargetSet& targets, const BodyDeciderSettings& s, BodyQueue& out);

    // Foot i's turn about the vertical on the ground (rad, character space).
    float footTurn(int i) const { return anchorYaw_[i]; }

    // The output pose (character space), valid after decide().
    const rig::Pose& outputPose() const { return output_.pose; }
    const std::vector<math::Mat4>& outputModels() const { return output_.models; }

private:
    // A pose and its bones (character space): the desired one (from the
    // targets each tick) and the output one (carried forward).
    struct Plan {
        rig::Pose pose;
        std::vector<math::Mat4> models;
    };
    static constexpr int kEnds = 4;  // hands L R, feet L R
    enum Which { HandLW, HandRW, FootLW, FootRW, HipsW, ChestW, HeadW, kTargets };

    void blendWeights(const TargetSet& t, const BodyDeciderSettings& s);
    // The feet's mean turn about the vertical (a step under way: by its progress).
    float stanceNow() const;
    // The pelvis's turn about the vertical, from the feet's mean turn, within
    // its own range and within every grounded leg's hip twist (each foot's turn
    // plus its hip's range): `s`'s pelvis range narrowed so (ranged_).
    void rangePelvis(const BodyDeciderSettings& s);
    // Target k's pose for the blend: the given one while Driven, else the
    // last given (fading out).
    const Target& given(const TargetSet& t, int k) const;
    void pinFeet(const TargetSet& t, const BodyDeciderSettings& s);
    void supportAndHips(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void desiredPose(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void limitOutput(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void solveLegs(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void solveArms(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void report(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    math::Quat pelvisBase() const { return math::fromAxisAngle(math::kUp, stanceYaw_) * restPelvis_.rotation; }
    math::Vec3 centreOfMass(const Plan& p) const;

    // Hand h's arm off plan p's chest, for its range (arm_fit.h).
    ArmShape armShape(int h, const Plan& p, const BodyDeciderSettings& s) const;
    ArmHang armHang(int h, const Plan& p) const;
    math::Quat armFrame(int h, const Plan& p) const;
    // What joint j asks past chainOnset of its range (before its clamp), as a
    // world rotation vector (the chain's start, passUp).
    math::Vec3 beyondRange(const Plan& p, int j, const BodyDeciderSettings& s) const;
    // The chain rule up `chain` (each joint the next one's child; the pelvis
    // may come last), the foot's rule: `rest` is the rotation (world rotation
    // vector) the joint before the chain cannot keep; each joint takes its
    // own part of it on its range axes (chainOwnPart, its own limits) and
    // hands the rest on, the same way about the same axis. Returns what none
    // could take; `pelvisTurned` set if the pelvis moved (legs to solve).
    math::Vec3 passUp(Plan& p, const int* chain, int count, math::Vec3 rest, const BodyDeciderSettings& s,
                      bool& pelvisTurned);
    int endRoot(int e) const;
    // How far end e (hand L, R, foot L, R) reaches from its root: its span
    // with the middle joint at limbMinBend.
    float limbReach(int e, const BodyDeciderSettings& s) const;
    // The head level in the world, facing the chest's way (desired pose).
    void levelHead(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);

    // Plan editing (desired_ unless given).
    void turnPlanned(Plan& p, int j, math::Quat turn);
    void placePelvis(Plan& p, const math::Transform& t);
    bool refresh(Plan& p, int from = -1);
    math::Vec3 planPos(const Plan& p, int joint) const;
    math::Quat planRot(const Plan& p, int joint) const;
    math::Transform pelvisOf(const Plan& p) const;
    void markPlanned(int joint, BodyQueue& out) const;
    // A local rotation moved from `from` toward `to` by at most `rate` x dt.
    static math::Quat limitTurn(math::Quat from, math::Quat to, float rate, float dt);
    void limitJoint(int j, float rate, float dt);
    // Joint j's local rotation held within its range in `p` (the physics
    // body's joint has the same: ik::clampRotation, Jolt's clamp). True if
    // it moved.
    bool clampJoint(Plan& p, int j) const;
    // A limb's joints held within their ranges after its solve (parents
    // first); a limb end that then misses `aim` (where the solve sent it) is
    // reported short (`rangeMiss_`, the output plan only), never snapped.
    void clampLimb(Plan& p, int e, const int* joints, int count, math::Vec3 aim);
    math::Quat restFootLevel(int i) const;  // a foot's standing rotation on `anchorNormal_`

    const rig::Rig* rig_ = nullptr;
    const ik::JointLimits* limits_ = nullptr;
    BodyDeciderJoints joints_;
    Plan desired_, output_;
    bool seeded_ = false;
    bool warnedNoGravity_ = false;  // the "no gravity: no balance step" warning, logged once
    float thighLength_[2]{}, shinLength_[2]{};
    float upperArmLength_[2]{}, forearmLength_[2]{};
    math::Quat footToPelvis_[2] = {math::identityQuat(), math::identityQuat()};
    math::Quat restFoot_[2] = {math::identityQuat(), math::identityQuat()};  // each foot's bone standing (character space)
    float restFootY_[2] = {0.0f, 0.0f};  // each ankle's height standing
    math::Vec3 restShoulder_[2]{};   // each shoulder standing (character space)
    math::Vec3 restHandChest_[2]{};  // each hand standing, hanging, in the chest's frame
    math::Vec3 hangDir_[2]{}, hangBend_[2]{};  // the relaxed arm in the chest's frame (rest)
    math::Transform restPelvis_{};   // the pelvis standing (rest, character space)
    // Feet on the ground: where each is pinned (ankle), its ground normal and
    // its flat rotation there (character space).
    math::Vec3 anchor_[2]{};
    math::Vec3 anchorNormal_[2] = {math::kUp, math::kUp};
    math::Quat anchorRot_[2] = {math::identityQuat(), math::identityQuat()};
    bool grounded_[2] = {true, true};  // on the ground this tick (Pinned, or Driven not yet lifted)
    // Where each grounded leg reaches to (its ankle, character space): the
    // anchor, or on a foot's way still on the ground (the weight shift) its
    // target as the leg is solved to it, the heel risen by the toe-off: the
    // hips' height bound counts the leg from there.
    math::Vec3 reachFrom_[2]{};
    // Blended states: each target's activation (0..1 toward its mode) and
    // each foot's support weight.
    float act_[kTargets] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // Each target as last given while Driven: a target let go blends back
    // from it (its activation fading), never from a stale or empty one.
    Target last_[kTargets];
    float support_[2] = {1.0f, 1.0f};
    // The hips' reference (support) and the output hips' motion state.
    math::Vec3 hipsRef_{0.0f}, hipsRefVel_{0.0f};
    math::Vec3 hipsVel_{0.0f};
    float rangeMiss_[kEnds] = {0.0f, 0.0f, 0.0f, 0.0f};  // m: the output limb's end off its aim by its joints' ranges
    // Each foot's turn about the vertical on the ground (rad), the turn of
    // its step under way (from, to), and the feet's mean turn this tick: the
    // pelvis's range counts from it.
    float anchorYaw_[2] = {0.0f, 0.0f};
    float stanceYaw_ = 0.0f;
    BodyDeciderSettings ranged_;  // this decide's settings, the pelvis range narrowed by the grounded hips
    std::vector<float> mass_;          // per rig joint (kg)
    std::vector<math::Vec3> centre_;   // per rig joint: its mass's centre in the bone's frame
    float totalMass_ = 0.0f;
    // Free arms: each rig joint's arm (-1 none, 0 left, 1 right), the
    // planned mass centres last decide (character space), each arm's swing
    // angle (rad, forward +) about the shoulder.
    std::vector<int8_t> armOf_;
    TargetSet work_;
    bool legChained_ = false;  // the lifted legs' chain done this tick (solveLegs)
    int trunkTicks_ = 0;  // ticks since the plan started (no rates before 2)
};

} // namespace motion
