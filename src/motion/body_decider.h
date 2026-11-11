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
// pinned foot stays put whatever the hips do. Hard constraints on the output
// pelvis: within hipsOffsetMax of the root (the capsule leads; the pelvis
// moves around it) and no higher than every grounded leg reaches.
// No allocations after init().

#include "ik/limb_ik.h"
#include "math/math.h"
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
    math::Vec3 pelvisOffset{0.0f};          // the output pelvis from the root, level (character space)
    bool offsetClamped = false;             // the pelvis sits on its bound around the root
    bool heightClamped = false;             // a grounded leg holds the hips lower than wanted
    ReachReport reach[4];                   // hand L, hand R, foot L, foot R
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
    // Each next part in a reach queue carries this times the share of the
    // part before it.
    float queueShare = 0.5f;
    // The measured lumbopelvic rhythm (Esola et al. 1996): lumbar : hip ratio
    // per band of trunk flexion (upper bounds, rad).
    float lumbarHipBand[3] = {0.5236f, 1.0472f, 1.5708f};  // 30, 60, 90 deg
    float lumbarHipRatio[3] = {1.9f, 0.9f, 0.4f};
    // m/s^2 down, from the physics world: a free arm hangs on it as a
    // pendulum. 0: free arms only cancel the turning.
    float gravity = 0.0f;
    float armCancel = 0.63f;  // the share of the body's turning the free arms cancel
    // Every target is always there (the part's own frame); a Driven one
    // pulls by its weight, the others by these shares, the body's own motion
    // the rest (the user's design 2026-11-09).
    float passiveHead = 0.7f, passiveHands = 0.5f, passiveHips = 0.5f, passiveChest = 0.5f;
    // m: through a foot's timed swing the hips, between the feet, go this
    // far toward the standing foot at mid-swing and back (centre of mass
    // side to side 3.9-7 cm over a stride, Orendurff et al. 2004).
    float hipsSway = 0.027f;
    // A limb reaches at most its span with the knee or elbow bent this much
    // (rad): quiet standing knee 0-5 deg (BIOMECHANICS 2); the two-bone
    // solve's soften keeps it from snapping.
    float limbMinBend = 0.0873f;  // 5 deg
    // s: a target's mode or weight change blends over this; a foot's support
    // weight over supportBlendTime (the weight shift of a step).
    float blendTime = 0.1f, supportBlendTime = 0.12f;
    // The output pose's bounds: the hips' speed (m/s) and acceleration
    // (m/s^2), their turn rate, the trunk's, the head's and a limb's joint
    // rates (rad/s).
    float hipsMaxSpeed = 2.0f, hipsMaxAccel = 8.0f, hipsMaxTurnRate = 4.0f;
    float trunkMaxRate = 4.0f, headMaxRate = 6.0f, limbMaxRate = 12.0f;
    // m: the output pelvis stays within this of the root, level (the capsule
    // leads; the pelvis moves around it).
    float hipsOffsetMax = 0.35f;
    // The lowest hips: no grounded knee bends past this (rad) on two feet
    // (the semi-squat, BIOMECHANICS 9.4) or on one (the largest mean stance
    // knee of a maximal one-leg foot reach, 9.7).
    float twoLegKneeMax = 1.5708f, oneLegKneeMax = 1.1694f;  // 90, 67 deg
    // A reach's pelvis tilt from standing, rotation vector (rad, character
    // space): forward tilt (hip flexion, +X), turn (Y), side tilt (Z);
    // reach.pelvisMinDeg / pelvisMaxDeg (BIOMECHANICS 9.3).
    math::Vec3 pelvisMin{-0.1745f, -0.5236f, -0.3491f}, pelvisMax{1.3963f, 0.5236f, 0.3491f};
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

    // One tick: from the targets and the plan so far, fills `out`. No
    // allocations.
    void decide(const TargetSet& targets, const BodyDeciderSettings& s, BodyQueue& out);

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
    // Target k's pose for the blend: the given one while Driven, else the
    // last given (fading out).
    const Target& given(const TargetSet& t, int k) const;
    void pinFeet(const TargetSet& t);
    void supportAndHips(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void desiredPose(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void limitOutput(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void solveLegs(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void solveArms(Plan& p, const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void report(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);

    // A hand's trunk: out of reach, the queue (by where the target is) and
    // its plan on the desired pose.
    void handTrunk(int h, math::Vec3 target, const BodyDeciderSettings& s, BodyQueue& out);
    void trunkOrders(int h, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s, BodyQueue& out);
    // A lifted foot out of reach: the stance leg lowers the hips, the pelvis
    // tilts on the stance hip.
    void footTrunk(int i, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s, BodyQueue& out);
    int endRoot(int e) const;
    float endReach(int e) const;
    // How far end e (hand L, R, foot L, R) reaches from its root: its span
    // with the middle joint at limbMinBend.
    float limbReach(int e, const BodyDeciderSettings& s) const;
    math::Vec3 tiltPivot(int e) const;
    // The planned trunk for an end out of reach: each part of the queue
    // [first, end) turns (or the hips move) by its share of what brings the
    // end's root within reach, in passes, within its range.
    void planTrunk(int e, math::Vec3 target, int first, int end, const BodyDeciderSettings& s, BodyQueue& out);
    // A queue of joints turning one way together, shared by place.
    void turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                   const BodyDeciderSettings& s, BodyQueue& out);
    void shareQueue(int first, const BodyDeciderSettings& s, BodyQueue& out) const;
    // The desired pelvis under the hands' and lifted feet's trunk: the
    // centre of mass over the support (standing only).
    void balanceStanding(const TargetSet& t, const BodyDeciderSettings& s);
    // Free arms (no goal) swing to cancel the plan's turning and settle to
    // their rest in the chest's frame.
    void freeArms(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void passiveHands(const bool free[2], const BodyDeciderSettings& s, BodyQueue& out);
    // The head level in the world, facing the chest's way (desired pose).
    void levelHead(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);

    // Plan editing (desired_ unless given).
    void turnPlanned(Plan& p, int j, math::Quat turn);
    void placePelvis(Plan& p, const math::Transform& t);
    void movePelvis(Plan& p, math::Vec3 delta);
    bool refresh(Plan& p, int from = -1);
    math::Vec3 planPos(const Plan& p, int joint) const;
    math::Quat planRot(const Plan& p, int joint) const;
    math::Transform pelvisOf(const Plan& p) const;
    void markPlanned(int joint, BodyQueue& out) const;
    // A local rotation moved from `from` toward `to` by at most `rate` x dt.
    static math::Quat limitTurn(math::Quat from, math::Quat to, float rate, float dt);
    void limitJoint(int j, float rate, float dt);
    math::Quat restFootLevel(int i) const;  // a foot's standing rotation on `anchorNormal_`
    // The highest the hips may be for grounded foot i (its leg at its reach).
    float reachableHeight(const Plan& p, int i, const BodyDeciderSettings& s, bool& valid) const;
    // The lowest the pelvis may be over the grounded feet: no grounded knee
    // past its bound (two feet or one). `any`: some grounded leg bounds it.
    float lowestHeight(const Plan& p, const BodyDeciderSettings& s, bool& any) const;

    const rig::Rig* rig_ = nullptr;
    const ik::JointLimits* limits_ = nullptr;
    BodyDeciderJoints joints_;
    Plan desired_, output_;
    bool seeded_ = false;
    float armLength_[2]{};  // upper arm + forearm
    float legLength_[2]{};  // thigh + shin
    float thighLength_[2]{}, shinLength_[2]{};
    float upperArmLength_[2]{}, forearmLength_[2]{};
    math::Vec3 hipOffset_[2]{};  // each hip joint from the pelvis (pelvis frame, rest)
    math::Quat footToPelvis_[2] = {math::identityQuat(), math::identityQuat()};
    math::Quat restFoot_[2] = {math::identityQuat(), math::identityQuat()};  // each foot's bone standing (character space)
    float restFootY_[2] = {0.0f, 0.0f};  // each ankle's height standing
    math::Quat restHead_ = math::identityQuat();
    math::Vec3 restHandChest_[2]{};  // each hand standing, hanging, in the chest's frame
    math::Vec3 hipCentre_{0.0f};     // between the hips, from the pelvis (rest)
    math::Vec3 hangDir_[2]{}, hangBend_[2]{};  // the relaxed arm in the chest's frame (rest)
    math::Transform restPelvis_{};   // the pelvis standing (rest, character space)
    // Feet on the ground: where each is pinned (ankle), its ground normal and
    // its flat rotation there (character space).
    math::Vec3 anchor_[2]{};
    math::Vec3 anchorNormal_[2] = {math::kUp, math::kUp};
    math::Quat anchorRot_[2] = {math::identityQuat(), math::identityQuat()};
    bool grounded_[2] = {true, true};  // on the ground this tick (Pinned, or Driven not yet lifted)
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
    // Each end's trunk orders this decide: [first, end) in the queue (-1: none).
    int trunkFirst_[kEnds] = {-1, -1, -1, -1}, trunkEnd_[kEnds] = {-1, -1, -1, -1};
    std::vector<float> mass_;          // per rig joint (kg)
    std::vector<math::Vec3> centre_;   // per rig joint: its mass's centre in the bone's frame
    float totalMass_ = 0.0f;
    // Free arms: each rig joint's arm (-1 none, 0 left, 1 right), the
    // planned mass centres last decide (character space), each arm's swing
    // angle (rad, forward +) about the shoulder.
    std::vector<int8_t> armOf_;
    std::vector<math::Vec3> lastCentre_;
    bool lastValid_ = false;
    float armAngle_[2] = {0.0f, 0.0f};
};

} // namespace motion
