#pragma once
// The body's decider (DECISIONS 2026-11-06; replaces BodyReposition once the
// physics body runs on it). From the body's current pose (the physics body's,
// character space) and its targets it decides only numbers: which parts must
// move, in real body order, and which way. It never moves a part, never
// applies joint limits and never decides how far: the muscles (Jolt motors)
// turn each part that way, the joint's limit or the muscle's strength stops
// it, and the caller reports that joint as blocked, which brings in the next
// part of the queue. Each order carries its target's own speed; turning that
// into a motor speed is the muscles' job.
// Real body order (BIOMECHANICS 3.1, 9.1; trunk-assisted reaching, Kaminski
// et al. 1995):
//  Hand: shoulder and elbow (two-bone IK, elbow back and out). Out of the
//   arm's reach the body joins from the start: every part of the queue moves
//   together (real parts overlap, BIOMECHANICS 9.6), each with a share by its
//   place in the queue, the first the most, the last very little (queueShare);
//   a blocked part's share goes to the rest. The queue is led by where the
//   target is (BIOMECHANICS 9.1, 9.3). Where research measured a split it
//   replaces the place share: in a bend the lumbar spine and the hips share by
//   the lumbopelvic rhythm at the trunk's flexion (Esola et al. 1996): the
//   back more early, the hips more as the bend deepens. Below the hips the pelvis tilt, the
//   hips lowering (legs bend), spine_01, spine_02, chest, clavicle; behind the
//   shoulders the chest, spine_02, spine_01, clavicle, pelvis, hips; ahead or
//   to the side the clavicle, chest, spine_02, spine_01, pelvis tilt, hips.
//  Chest and head chains share the same way.
//  Chest: chest, spine_02, spine_01, pelvis. Head: head, neck.
//  Foot (an active foot target): hip and knee (two-bone IK, knee forward);
//   the hip blocked: the pelvis tilts; a foot on the ground beyond the leg's
//   length: the hips move toward it.
//  Push-off: the unloading ankle points its foot (a muscle action).
//  Hips: the hips move toward their target (the legs carry them).
// No allocations after init().

#include "ik/limb_ik.h"
#include "math/math.h"
#include "rig/rig.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ik {
class JointLimits;
}

namespace motion {

// The body's 7 targets (character space). Only active ones are followed.
struct BodyGoals {
    struct Point {
        bool active = false;
        math::Vec3 position{0.0f};
        math::Vec3 velocity{0.0f};  // m/s, the target's own motion
        // A foot on a timed swing (a step): the share of its time off the
        // ground gone, from toe-off (0) to heel strike (1). Negative: no
        // timing (a foot just held up).
        float swing = -1.0f;
    };
    struct Turn {
        bool active = false;
        math::Quat rotation = math::identityQuat();  // the part's orientation (identity = upright, facing forward)
        math::Vec3 angularVelocity{0.0f};           // rad/s, the target's own motion
    };
    Point hand[2];  // left, right: wrist
    Point foot[2];  // left, right: ankle
    Point hips;     // pelvis
    Turn chest, head;
    bool down[2] = {true, true};   // left, right foot on the ground
    float push[2] = {0.0f, 0.0f};  // left, right push-off share [0, 1]: the unloading ankle points the foot
    float dt = 0.0f;  // s since the last decide: the hips travel to a new place over time (0: they jump)
};

enum class Goal { HandL, HandR, FootL, FootR, Hips, Chest, Head };
enum class PartMove { Turn, Move };

// One part of the queue: which way it must go, never how far.
struct PartOrder {
    int joint = -1;                      // rig joint (the pelvis for the hips)
    PartMove kind = PartMove::Turn;      // turn about `direction`, or move along it (hips only)
    math::Vec3 direction{0.0f};          // unit, character space: rotation axis (right-hand rule) or travel direction
    math::Vec3 targetVelocity{0.0f};     // the target's own speed (m/s for a move or point target, rad/s for a turn target)
    Goal goal = Goal::HandL;             // the target it serves
    bool active = false;                 // its muscle pulls now (else it waits for the parts it needs blocked first)
    float share = 1.0f;                  // its share of a motion its queue carries together (a queue's shares sum to 1)
};

struct BodyQueue {
    static constexpr int kCapacity = 48;
    static constexpr int kMaxJoints = 128;
    std::array<PartOrder, kCapacity> orders{};
    int count = 0;
    // The brain's target pose (DECISIONS 2026-11-07): the planned bone of every
    // rig joint (character space, valid until the next decide) and which
    // joints follow it (non-zero): their muscles pull them to it as springs
    // (Jolt's position motors). Others keep their orders or hold.
    const std::vector<math::Mat4>* plan = nullptr;
    std::array<uint8_t, kMaxJoints> planned{};
};

struct BodyDeciderJoints {
    int pelvis = -1;
    int spine[3] = {-1, -1, -1};  // spine_01, spine_02, chest
    int neck = -1, head = -1;
    int clavicles[2] = {-1, -1};
    ik::TwoBoneChain legs[2];  // left, right: thigh, shin, foot
    ik::TwoBoneChain arms[2];  // left, right: upper arm, forearm, hand
};

struct BodyDeciderSettings {
    math::Vec3 kneePole{0.0f, 0.0f, 1.0f};    // knees point this way (pelvis frame)
    // human_joints.json "decider.queueShare": each next part in a queue
    // carries this times the share of the part before it.
    float queueShare = 0.5f;
    // human_joints.json "decider": the measured lumbopelvic rhythm (Esola et
    // al. 1996): lumbar : hip ratio per band of trunk flexion (upper bounds,
    // rad); it splits the lumbar spine's and the pelvis tilt's combined share.
    float lumbarHipBand[3] = {0.5236f, 1.0472f, 1.5708f};  // 30, 60, 90 deg
    float lumbarHipRatio[3] = {1.9f, 0.9f, 0.4f};
    // The hips' move to a new place takes Fitts' time (human_joints.json
    // "reach": a + b log2(2D / width), s) on a minimum-jerk path.
    float fittsA = 0.1f, fittsB = 0.1f, fittsWidth = 0.1f;
    // m/s^2 down, from the physics world (physics_world.json): a free arm
    // hangs on it as a pendulum. 0: free arms only cancel the turning.
    float gravity = 0.0f;
    // human_joints.json "decider.armCancel": the share of that turning the
    // free arms cancel.
    float armCancel = 0.63f;
    // human_joints.json "decider.hipsSway" (m): through a foot's timed swing
    // the hips, between the feet, go toward the standing foot and back, this
    // far at mid-swing (centre of mass side-to-side 3.9-7 cm in all over a
    // stride, Orendurff et al. 2004; a sine over the single support).
    float hipsSway = 0.027f;
    // human_joints.json "decider.passiveInfluence": every one of the seven
    // targets is always there (where the part is standing, moving with the
    // root); one with a goal pulls fully, the others by their share (the
    // user's design 2026-11-09), the body's own motion the rest.
    float passiveHead = 0.7f, passiveHands = 0.5f, passiveHips = 0.5f, passiveChest = 0.5f;
};

class BodyDecider {
public:
    // Reads the rest pose's limb lengths and hips. `limits` (kept): the
    // joints' real ranges the planned trunk stays within (none: unlimited).
    // Load-time only.
    bool init(const rig::Rig& rig, const BodyDeciderJoints& joints, const ik::JointLimits* limits = nullptr);
    // The body's mass per rig joint (kg, 0: none) and its centre in the
    // bone's frame: the plan keeps its centre of mass over the planted feet.
    // Load-time only.
    void setMasses(const std::vector<float>& mass, const std::vector<math::Vec3>& centre);
    // After decide: the plan places the pelvis (a hand's reach), and the
    // plan's centre of mass (character space; needs setMasses).
    bool planning() const { return pelvisPlanned_; }
    // The character root moved (`toNew`: old character space to new): what
    // the decider holds (planted feet, the hips' path) stays put in the world.
    void rebase(const math::Transform& toNew);
    math::Vec3 planCentreOfMass() const;

    // Fills `out` from the body's current `pose` (character space) and its
    // goals. `blocked` has one entry per rig joint: non-zero when that joint
    // cannot go further the way it is pulled (its limit or its strength; the
    // pelvis: the hips cannot tilt further). No allocations.
    void decide(const rig::Pose& pose, const BodyGoals& goals, const std::vector<uint8_t>& blocked,
                const BodyDeciderSettings& s, BodyQueue& out);

private:
    void handOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                    const BodyDeciderSettings& s, BodyQueue& out);
    // A hand's trunk: planned upright, bent by the queue when out of reach.
    void handTrunk(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                   const BodyDeciderSettings& s, BodyQueue& out);
    // A hand out of reach: the trunk's queue (by where the target is) and
    // its plan.
    void trunkOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                     const BodyDeciderSettings& s, BodyQueue& out);
    void footOrders(int i, const BodyGoals& goals, const std::vector<uint8_t>& blocked, const BodyDeciderSettings& s,
                    BodyQueue& out);
    // A lifted foot out of reach: the stance leg lowers the hips, the pelvis
    // tilts on the stance hip (the queue and its plan).
    void footTrunk(int i, const BodyGoals::Point& goal, const BodyDeciderSettings& s, BodyQueue& out);
    // A reach end e: 0, 1 the hands, 2, 3 the feet. Its root (shoulder,
    // hip), its limb's length, and where its pelvis tilt turns (plan).
    static constexpr int kEnds = 4;
    int endRoot(int e) const;
    float endReach(int e) const;
    math::Vec3 tiltPivot(int e) const;
    // Shoulder and elbow (or hip and knee) of `chain` toward `target`.
    void limbOrders(const ik::TwoBoneChain& chain, math::Vec3 target, math::Vec3 pole, math::Vec3 velocity, Goal goal,
                    BodyQueue& out);
    // A queue of joints turning one way together, shared by place.
    void turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                   const std::vector<uint8_t>& blocked, const BodyDeciderSettings& s, BodyQueue& out);
    // Shares of the queue orders [first, out.count): by place (queueShare),
    // blocked ones none, normalised to 1; a part with a share is active.
    void shareQueue(int first, const BodyDeciderSettings& s, BodyQueue& out) const;
    // The planned trunk for an end out of reach: from upright, each part of
    // the queue [first, end) turns (or the hips move) by its share of what
    // brings the end's root within reach, in passes, within its range.
    void planTrunk(int e, math::Vec3 target, int first, int end, BodyQueue& out);
    // The planned legs under a planned pelvis: each planted foot stays where
    // it is (two-bone solve from the planned hip, knee forward).
    // balance: the hips go where the centre of mass is over the feet; else
    // they stay where they are and only the legs reach the feet.
    void planLegs(const BodyGoals& goals, const BodyDeciderSettings& s, BodyQueue& out, bool balance = true);
    // Arms without a goal: not held where the body left them; each hangs
    // from its shoulder (a pendulum) and swings to cancel the rest of the
    // plan's turning about the vertical, by its mass and lever.
    // Free hands: the passive share toward their own (hanging) targets.
    void passiveHands(const bool free[2], const BodyDeciderSettings& s, BodyQueue& out);
    void freeArms(const BodyGoals& goals, const BodyDeciderSettings& s, BodyQueue& out);
    // The plan's joint `j` (character space) turned by `turn`, within its range.
    void turnPlanned(int j, math::Quat turn);
    // The plan's pelvis placed at `t` (character space).
    void placePlannedPelvis(const math::Transform& t);
    // The plan's pelvis (and all below it) moved by `delta`, not turned.
    void movePlannedPelvis(math::Vec3 delta);
    void markPlanned(int joint, BodyQueue& out) const;

    math::Vec3 pos(int joint) const;
    math::Quat rot(int joint) const;
    // The plan's model matrices from its locals (scratch_ -> solved_): all
    // (-1), or only `from` and the joints below it (the rest stay as they are).
    bool refreshPlan(int from = -1);
    math::Vec3 planPos(int joint) const;   // in the plan (solved_)
    math::Quat planRot(int joint) const;
    bool isBlocked(const std::vector<uint8_t>& blocked, int joint) const;

    const rig::Rig* rig_ = nullptr;
    const ik::JointLimits* limits_ = nullptr;
    bool pelvisPlanned_ = false;  // this decide moved or tilted the planned pelvis
    BodyDeciderJoints joints_;
    float armLength_[2]{};  // upper arm + forearm
    float legLength_[2]{};  // thigh + shin
    // Each foot's bone to the pelvis's, standing (rest): a planted leg's knee
    // pole is the standing one carried with its foot.
    math::Quat footToPelvis_[2] = {math::identityQuat(), math::identityQuat()};
    math::Quat restFoot_[2] = {math::identityQuat(), math::identityQuat()};  // each foot's bone standing (character space)
    math::Quat restHead_ = math::identityQuat();  // the head's bone standing (character space)
    math::Vec3 restHand_[2]{};  // each hand standing, hanging (character space): its passive target
    math::Vec3 hipCentre_{0.0f};  // between the hips, from the pelvis (rest)
    // The relaxed arm, in the chest's frame (rest): shoulder-to-wrist
    // direction and the way its elbow points when it bends from there (back:
    // the forearm flexes forward).
    math::Vec3 hangDir_[2]{}, hangBend_[2]{};
    math::Transform restPelvis_{};  // the pelvis standing (rest pose, character space): where every plan starts
    // Each planted foot where it was planted (its ankle and the foot's
    // rotation, character space): taken while no plan runs, kept while the
    // foot stays planted under one (a planted foot does not move; the ankle
    // rising with the heel must not move the plan).
    math::Vec3 anchor_[2]{};
    math::Quat anchorRot_[2] = {math::identityQuat(), math::identityQuat()};
    bool anchored_[2] = {false, false};
    // The planned hips' travel to their new place (character space).
    struct HipsPath {
        math::Vec3 from{0.0f}, to{0.0f}, at{0.0f};
        float time = 0.0f, duration = 0.0f;
        bool on = false;
    };
    HipsPath hips_;
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
    rig::Pose scratch_;  // the plan (locals): the body now, with the planned parts replaced
    std::vector<math::Mat4> models_, solved_;  // the body now; the plan (character space)
};

} // namespace motion
