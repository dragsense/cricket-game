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
//   arm's reach the trunk joins from the start: the clavicle, then chest,
//   spine_02, spine_01, then the pelvis tilts at the hips, then the hips move
//   toward the hand; each joins when every part before it is blocked.
//  Chest: chest, spine_02, spine_01, then the pelvis, the same way.
//  Head: head, then neck.
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

namespace motion {

// The body's 7 targets (character space). Only active ones are followed.
struct BodyGoals {
    struct Point {
        bool active = false;
        math::Vec3 position{0.0f};
        math::Vec3 velocity{0.0f};  // m/s, the target's own motion
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
    bool active = false;                 // its muscle pulls now (else it waits for the parts before it)
};

struct BodyQueue {
    static constexpr int kCapacity = 48;
    std::array<PartOrder, kCapacity> orders{};
    int count = 0;
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
    math::Vec3 elbowPole{0.3f, 0.0f, -1.0f};  // left elbow points this way (mirrored for the right; +X is left)
};

class BodyDecider {
public:
    // Reads the rest pose's limb lengths and hips. Load-time only.
    bool init(const rig::Rig& rig, const BodyDeciderJoints& joints);

    // Fills `out` from the body's current `pose` (character space) and its
    // goals. `blocked` has one entry per rig joint: non-zero when that joint
    // cannot go further the way it is pulled (its limit or its strength; the
    // pelvis: the hips cannot tilt further). No allocations.
    void decide(const rig::Pose& pose, const BodyGoals& goals, const std::vector<uint8_t>& blocked,
                const BodyDeciderSettings& s, BodyQueue& out);

private:
    void handOrders(int h, const BodyGoals::Point& goal, const std::vector<uint8_t>& blocked,
                    const BodyDeciderSettings& s, BodyQueue& out);
    void footOrders(int i, const BodyGoals& goals, const std::vector<uint8_t>& blocked, const BodyDeciderSettings& s,
                    BodyQueue& out);
    // Shoulder and elbow (or hip and knee) of `chain` toward `target`.
    void limbOrders(const ik::TwoBoneChain& chain, math::Vec3 target, math::Vec3 pole, math::Vec3 velocity, Goal goal,
                    BodyQueue& out);
    // A chain of joints turning one way, each joining once all before it are blocked.
    void turnChain(const int* joints, int count, math::Quat delta, math::Vec3 velocity, Goal goal,
                   const std::vector<uint8_t>& blocked, BodyQueue& out);

    math::Vec3 pos(int joint) const;
    math::Quat rot(int joint) const;
    bool isBlocked(const std::vector<uint8_t>& blocked, int joint) const;

    const rig::Rig* rig_ = nullptr;
    BodyDeciderJoints joints_;
    float armLength_[2]{};  // upper arm + forearm
    float legLength_[2]{};  // thigh + shin
    math::Vec3 hipCentre_{0.0f};  // between the hips, from the pelvis (rest)
    rig::Pose scratch_;           // the pose a two-bone solve would give (for its directions only)
    std::vector<math::Mat4> models_, solved_;
};

} // namespace motion
