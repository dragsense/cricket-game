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
    // A lifted knee turns toward its foot's way, the way counted from this
    // share of the leg ahead of the hip (a foot right under the hip: the knee
    // forward). Design value.
    float kneeForwardBias = 0.25f;
    // A body standing reaches a hand this share of its height past the arm
    // (forward, level; Moriyama 2022's functional reach, young adults 0.22-
    // 0.24 H); a hand further takes the whole body after it (a step).
    float standingReachShare = 0.23f;
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
    // The pelvis turns about the vertical with the legs: a share of the turn
    // of the line between the feet (a foot ahead turns its hip forward), so
    // longer steps turn it more (BIOMECHANICS 4.3: 11 deg peak to peak at
    // 0.40 m/s, 19 at 1.47 m/s, Yang 2013; larger with big steps, Liang 2014).
    // The pelvis drops on the side of the leg without weight, about the hip
    // that carries it (pelvic list), this much (rad) with all the weight on
    // one leg, by the weight's share (the weight shift: it leads a lift).
    float pelvisList = 0.0698f;  // 4 deg
    // The trunk's axial spring-damper (Prins et al. 2019): the chest's turn
    // follows the pelvis's through it (natural frequency rad/s, damping
    // ratio). Driven faster than its frequency (quicker steps) it lags and
    // turns less, opposite to the pelvis; slowly, with it (BIOMECHANICS 4.3).
    float trunkTurnFrequency = 2.5f, trunkTurnDamping = 0.5f;
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
    // The balance step (human_joints.json "decider.balanceStep"): the
    // extrapolated centre of mass XCoM = CoM + v / w0, w0 = sqrt(g / l)
    // (Hof, Gazendam & Sinke 2005). Out of the support (the feet's outline)
    // by more than stepMargin: a step, its foot landing where the XCoM will
    // be when it lands, landingBehind short of it along the motion (so the
    // body carries on), under its own hip to the side; never crossing the
    // other foot (minFootGap) nor further from it than a step reaches: an
    // ellipse, maxStepForward fore-aft, maxStepSide across (the hip flexes
    // and extends much further than it abducts, BIOMECHANICS 2).
    bool balanceStepOn = true;
    float stepMargin = 0.02f;       // m
    float footHalfWidth = 0.045f;   // m, the foot's outline: half its width
    float footHalfLength = 0.11f;   // m, half its length
    float footCentreAhead = 0.05f;  // m, the foot's centre ahead of the ankle
    float landingBehind = 0.05f;    // m
    float minFootGap = 0.1f;        // m
    float maxStepForward = 0.6f, maxStepSide = 0.5f;  // m
    // Where the hips stop (no speed there) the foot lands so they end between
    // the feet (the user's rule: a normal stance); moving on, by the capture
    // point. Blended by the hips' speed there, all the capture point's from
    // this speed (m/s).
    float stopBlendSpeed = 0.3f;
    // A foot left behind steps too: once, by the time a step could land, its
    // leg at its reach would hold the hips more than this below standing (m).
    float reachDrop = 0.04f;
    // The longest weight shift before a lift (s): a body that needs longer
    // steps anyway (gait initiation's APA lasts ~0.5 s at natural speed).
    float weightShiftMost = 0.5f;
    // A step's swing is an aimed foot move: Fitts' law, a + b log2(2D / W)
    // (the limb test's goals' values, human_joints.json "muscles").
    float fittsA = 0.1f, fittsB = 0.1f, fittsWidth = 0.1f;
    // The decider's own step: its shape (locomotion.json "step": height,
    // apex, toe-off, ball of the foot; its timing is decided per step), and
    // the parts' passive shares while it runs ("step.passive", unless the
    // action gives its own).
    StepPlannerSettings stepShape;
    PassiveShares stepPassive;
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

    // The decider's step under way (the latest started): its foot (-1: none)
    // and its clock share.
    int stepFoot() const;
    float stepShare() const;
    // Foot i's step (tests, debugging) and its timing.
    const StepPlanner& step(int i) const { return steps_[i]; }
    const StepPlannerSettings& stepTiming(int i) const { return stepNow_[i]; }

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
    // The balance step on the output pose (StepAdvice).
    void balanceStep(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    // The next step's timing (nextStep_): the step's shape's.
    void timeStep(const BodyDeciderSettings& s) { nextStep_ = s.stepShape; }
    // Every swing's landing time, planned (not chased each tick): on a foot's
    // event (a step starts or lands) or a change of the hips target's motion,
    // from the body's own prediction of the hips (stanceLeft), keeping the
    // swing's progress (StepPlanner::setTimeLeft).
    void timeSwings(const TargetSet& t, const BodyDeciderSettings& s);
    // The body's prediction of grounded leg o's stance: the time until it can
    // no longer hold the hips: until the hips are so far past it that its
    // reach holds them reachDrop below standing (the balance step's bound).
    float stanceLeft(int o, const BodyDeciderSettings& s) const;
    // How far a ground point (character space) is out of the outline of
    // feet with their ankles at `ankles` (n of them; 0: inside).
    static float outsideFeet(math::Vec3 p, const math::Vec3* ankles, int n, const BodyDeciderSettings& s);
    // Where foot i lands for capture point `xcom` moving with `velocity`, the
    // hips (centre of mass, on the ground) ending at `hipsEnd` (`hipsGoal`:
    // a hips target sets that place; free hips: the capture point); `miss`: how
    // far it stays from the ideal spot (crossing, reach), `ideal` that spot,
    // `beyondReach` whether a step's reach cut it short (more steps go on).
    math::Vec3 stepSpot(int i, math::Vec3 xcom, math::Vec3 velocity, math::Vec3 hipsEnd, bool hipsGoal,
                        const BodyDeciderSettings& s, float& miss, math::Vec3* ideal = nullptr,
                        bool* beyondReach = nullptr) const;
    // A step of foot i from `from` to `spot` on the hips target's path: the
    // time it takes (the sync formula), `steps` steps sharing the way, at
    // least the leg's swing; negative: no path.
    float pathStepTime(const TargetSet& t, int i, math::Vec3 from, math::Vec3 spot, int steps,
                       const BodyDeciderSettings& s) const;
    // A step's swing of foot i from `from` to `spot` (s): Fitts' law for the
    // way, at least the leg's swing (swingTimeAtLeast).
    float swingTime(int i, math::Vec3 from, math::Vec3 spot, const BodyDeciderSettings& s) const;
    // The shortest swing of leg i from `from` to `spot` (s): at rest at both
    // ends it peaks at 1.875 D / T, the foot at most legMaxRate x the leg.
    float swingTimeAtLeast(int i, math::Vec3 from, math::Vec3 spot, const BodyDeciderSettings& s) const;
    // The body as a linear inverted pendulum over ground at `groundY`: its
    // rate w0 = sqrt(g / l), l the plan's centre of mass height (0: no gravity).
    float pendulumRate(float groundY, const BodyDeciderSettings& s) const;
    // Foot f's step just set up (stepNow_[f], to `spot`): its weight shift
    // takes the shortest time that keeps the centre of pressure within the
    // feet as the capture point moves to where the swing needs it (DCM,
    // Englsberger 2015); `fixedTotal`: the step's whole time is given (a
    // path), the swing keeps at least its leg's time.
    void timeWeightShift(int f, math::Vec3 spot, bool fixedTotal, const BodyDeciderSettings& s);
    // The hips' level place on a step of the decider's own (foot `timed`,
    // the other standing): the capture point's path (DCM) and the hips
    // following it, x' = -w0 (x - xi).
    math::Vec3 stepHips(int timed, float dt, const BodyDeciderSettings& s);
    math::Vec3 centreOfMass(const Plan& p) const;

    // A hand's trunk: out of reach, the queue (by where the target is) and
    // its plan on the desired pose.
    void handTrunk(int h, math::Vec3 target, const BodyDeciderSettings& s, BodyQueue& out);
    void trunkOrders(int h, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s, BodyQueue& out);
    // A lifted foot out of reach: the stance leg lowers the hips, the pelvis
    // tilts on the stance hip.
    void footTrunk(int i, math::Vec3 target, math::Vec3 velocity, const BodyDeciderSettings& s, BodyQueue& out);
    // A lifted foot further out to the side than the hip abducts: the pelvis
    // hikes on that side about the standing hip by the rest (its range).
    void footHike(int i, math::Vec3 target, const BodyDeciderSettings& s);
    // Leg i's knee reaching from `hip` to `target`, bent toward `pole`; the
    // thigh's abduction (rad, out +) in the pelvis's frame.
    math::Vec3 kneeToward(int i, math::Vec3 hip, math::Vec3 target, math::Vec3 pole) const;
    float thighAbduction(int i, math::Quat pelvisRot, math::Vec3 hip, math::Vec3 knee) const;
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
    // The queue's shares by least effort (lever^2 / inertia) toward `end` (none: lever 1).
    void shareQueue(int first, const math::Vec3* end, const BodyDeciderSettings& s, BodyQueue& out) const;
    // The desired pelvis under the hands' and lifted feet's trunk: the
    // centre of mass over the support (standing only).
    void balanceStanding(const TargetSet& t, const BodyDeciderSettings& s);
    // The pelvis's turn about the vertical from the feet (rad, + to the left).
    float pelvisTurn(const TargetSet& t, const BodyDeciderSettings& s) const;
    // The trunk's spring-damper one tick on: the chest's turn against the
    // pelvis (trunkTwist_) from the pelvis's turn.
    void trunkSpring(float pelvisYaw, float dt, const BodyDeciderSettings& s);
    // Free arms (no goal) swing to cancel the plan's turning and settle to
    // their rest in the chest's frame.
    void freeArms(const TargetSet& t, const BodyDeciderSettings& s, BodyQueue& out);
    void passiveHands(const bool free[2], float share, BodyQueue& out);
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
    // Joint j's local rotation held within its range in `p` (the physics
    // body's joint has the same: ik::clampRotation, Jolt's clamp). True if
    // it moved.
    bool clampJoint(Plan& p, int j) const;
    // A limb's joints held within their ranges after its solve (parents
    // first); a limb end that then misses `aim` (where the solve sent it) is
    // reported short (`rangeMiss_`, the output plan only), never snapped.
    void clampLimb(Plan& p, int e, const int* joints, int count, math::Vec3 aim);
    math::Quat restFootLevel(int i) const;  // a foot's standing rotation on `anchorNormal_`
    // The highest the hips may be for grounded foot i (its leg at its reach).
    float reachableHeight(const Plan& p, int i, const BodyDeciderSettings& s, bool& valid) const;
    // The lowest the pelvis may be over the grounded feet: no grounded knee
    // past its bound (two feet or one). `any`: some grounded leg bounds it.
    float lowestHeight(const Plan& p, const BodyDeciderSettings& s, bool& any) const;
    // How far the grounded hips are past their ranges (rad, the worst) with
    // the pelvis at `pelvis`: each grounded leg solved onto its anchor as
    // solveLegs does. Places p's pelvis and those legs.
    float groundedHipExcess(Plan& p, const TargetSet& t, const math::Transform& pelvis, const BodyDeciderSettings& s);
    // Where leg i's solve aims from `hip`: its anchor, or on its way off it
    // (a driven foot by its weight), within its reach.
    math::Vec3 legTarget(const TargetSet& t, int i, math::Vec3 hip, const BodyDeciderSettings& s) const;
    // `pelvis` turned about each grounded hip past its range until that
    // thigh is on its range (the thigh carries the pelvis). Places p's pelvis.
    math::Transform hipsInRange(Plan& p, const TargetSet& t, math::Transform pelvis, const BodyDeciderSettings& s);
    // The pelvis from `from` (first carried onto the ranges, hipsInRange)
    // toward `to` as far as no grounded hip goes past its range, on the way
    // between them. Places p's pelvis.
    math::Transform boundHips(Plan& p, const TargetSet& t, const math::Transform& from, const math::Transform& to,
                              const BodyDeciderSettings& s);

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
    // Through a step and its settling (dcmOn_): the hips' level place, which
    // follows the capture point (dcmXi_), and the capture point where the
    // step began (dcmStart_). Off: the hips are where the support puts them.
    math::Vec3 dcmHips_{0.0f}, dcmXi_{0.0f}, dcmStart_{0.0f};
    bool dcmOn_ = false;
    // Each end's trunk orders this decide: [first, end) in the queue (-1: none).
    int trunkFirst_[kEnds] = {-1, -1, -1, -1}, trunkEnd_[kEnds] = {-1, -1, -1, -1};
    float rangeMiss_[kEnds] = {0.0f, 0.0f, 0.0f, 0.0f};  // m: the output limb's end off its aim by its joints' ranges
    // How far (level) a hand beyond the standing reach takes the hips toward
    // it this decide: standing balance keeps the centre of mass over the
    // support moved on by it (the body leans out and steps after the hand).
    math::Vec3 reachShift_{0.0f};
    std::vector<float> mass_;          // per rig joint (kg)
    std::vector<math::Vec3> centre_;   // per rig joint: its mass's centre in the bone's frame
    float totalMass_ = 0.0f;
    // A foot's share that the pelvis's turn takes of its fore-aft way, by
    // least effort (h^2 / I_turn against L^2 / I_leg at rest, h the hip's
    // offset from the pelvis, L the leg; the muscle plan B2), and h.
    float pelvisTurnShare_ = 0.0f, hipHalfWidth_ = 0.0f;
    // Free arms: each rig joint's arm (-1 none, 0 left, 1 right), the
    // planned mass centres last decide (character space), each arm's swing
    // angle (rad, forward +) about the shoulder.
    std::vector<int8_t> armOf_;
    std::vector<math::Vec3> lastCentre_;
    bool lastValid_ = false;
    float armAngle_[2] = {0.0f, 0.0f};
    // The balance step: the foot that stepped last (-1: none yet) and which
    // foot was on a timed swing last decide.
    int lastStepped_ = -1;
    bool wasSwinging_[2] = {false, false};
    // The decider's own steps, one per foot, each one's timing, the
    // next step's, the foot that started last, and this tick's targets with
    // the stepping feet's in place.
    StepPlanner steps_[2];
    StepPlannerSettings stepNow_[2], nextStep_;
    int lastStarted_ = -1;
    TargetSet work_;
    // A foot's event this tick (the swings' times are planned again) and the
    // hips target's level velocity last tick.
    bool footEvent_ = false;
    math::Vec3 lastGoalVelocity_{0.0f};
    // The hips target's path last tick (its end, time left; negative: none):
    // a new or moved path is an event (the swings are timed again).
    math::Vec3 lastPathEnd_{0.0f};
    float lastPathLeft_ = -1.0f;
    // How many steps share the hips' path, for each foot's step under way.
    int pathSteps_[2] = {1, 1};
    // The trunk's spring-damper: the pelvis's turn and its rate last tick,
    // the chest's turn against the pelvis and its rate (rad, rad/s).
    float lastPelvisYaw_ = 0.0f, lastPelvisRate_ = 0.0f;
    float trunkTwist_ = 0.0f, trunkTwistRate_ = 0.0f;
    // The pelvic list this decide (rad about the pelvis's forward, +: the
    // left side up) and that forward: the trunk turns back from it.
    float listAngle_ = 0.0f;
    math::Vec3 listAxis_{0.0f, 0.0f, 1.0f};
    // The list the output pelvis carries and its pivot (pelvis frame: the
    // weighted hips), taken off before its bounds each tick and put back on.
    float appliedList_ = 0.0f;
    math::Vec3 appliedPivot_{0.0f};
    int trunkTicks_ = 0;  // ticks since the plan started (no rates before 2)
};

} // namespace motion
