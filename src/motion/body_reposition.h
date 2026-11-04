#pragma once
// Body repositioning from the body's targets: the feet now, the hands now,
// the hips, chest and head later. The body follows wherever a target is put;
// a foot in the air only reaches (it cannot pull the body). Real body order (BIOMECHANICS 3.1, 9.1): each
// part reaches with its own joints first; the next part in only takes what
// that left, inside its own joint limits.
//  Feet:
//   1. The moving leg reaches its foot (two-bone IK, knee forward).
//   2. Past the hip's range the pelvis tilts by the rest, over the standing
//      hip: e.g. a foot far behind with a straight leg tips the body forward.
//   3. Past the leg's length a foot on the ground draws the pelvis toward
//      it; the standing foot stays, so the body lowers (a lunge). A foot in
//      the air just reaches as far as its leg allows.
//  Chest:
//   1. The spine turns the chest toward its target: chest, spine_02, then
//      spine_01, each inside its limit.
//   2. The pelvis tilts at the hips for the rest, inside its limits.
//  Hands (reached from the trunk as it then stands):
//   1. The arm reaches (two-bone IK, elbow back and out).
//   2. Out of the arm's reach: the clavicle, then chest, spine_02, spine_01
//      swing the shoulder toward the hand, each inside its limit.
//   3. Then the pelvis tilts at the hips (hip flexion: a bend to the ground).
//   4. Then the body moves toward the hand; the planted feet stay, so the
//      legs bend (a squat or lunge).
//  Foot roll (every foot, every movement; BIOMECHANICS 12): the ankle turns
//  within its loaded range; past it the shin turns the foot about the part of
//  it on the ground, which stays put while the ankle swings round it:
//   - shin back past the plantarflexion range: on the heel, toes up;
//   - shin within the range: flat;
//   - shin forward past the dorsiflexion range: on the ball, heel up, toes bent.
//  Push-off: as a foot unloads before it lifts, the calf points it; its
//  range moves from dorsiflexion to pushOff plantarflexion, so the heel rises
//  further and the knee bends before toe-off (no straight-leg snap).
//  The ankle targets are the flat foot's ankles; the roll moves them.
// The planted feet never move. No allocations after init().

#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "rig/rig.h"

#include <vector>

namespace motion {

struct BodyRepositionSettings {
    float legReach = 1.0f;          // share of the leg's length a leg straightens to
    float armReach = 1.0f;          // share of the arm's length a hand reaches to
    math::Vec3 kneePole{0.0f, 0.0f, 1.0f};        // knees point this way (pelvis frame)
    math::Vec3 elbowPole{0.3f, 0.0f, -1.0f};      // left elbow points this way (mirrored for the right; +X is left)
    math::Vec3 pelvisMin{0.0f}, pelvisMax{0.0f};  // pelvis rotation limits (rotation vector, rad)
    int passes = 3;                 // passes per tick
    float dorsiflexion = 0.17f;     // rad, ankle's loaded dorsiflexion range (shin forward) before the heel lifts
    float plantarflexion = 0.17f;   // rad, ankle's loaded plantarflexion range (shin back) before the toes rise
    float pushOff = 0.35f;          // rad, ankle plantarflexion at full push-off (toe-off 15-20 deg)
    int rollPasses = 2;             // body-then-foot-roll passes per tick (the roll changes the ankles)
};

struct BodyRepositionJoints {
    int pelvis = -1;
    ik::TwoBoneChain legs[2];  // left, right: thigh, shin, foot
    int toes[2] = {-1, -1};
    // Sole contacts from the ankle (rest, character space; the skeleton's
    // shoe): heel = back edge, ball = under the toe joint, both on the sole.
    math::Vec3 heel[2]{}, ball[2]{};
    int spine[3] = {-1, -1, -1};  // spine_01, spine_02, chest
    int clavicles[2] = {-1, -1};
    ik::TwoBoneChain arms[2];  // left, right: upper arm, forearm, hand
};

// Where the body's targets are (character space).
struct BodyTargets {
    int planted = 1;               // 0 left, 1 right: the foot that stands
    math::Vec3 plantedAnkle{0.0f}; // its ankle
    math::Vec3 movingAnkle{0.0f};  // the other leg's ankle target
    bool down[2] = {true, true};   // left, right foot on the ground (only a foot on the ground holds the body)
    float push[2] = {0.0f, 0.0f};  // left, right push-off share [0, 1]: the unloading ankle points the foot (heel up)
    bool chest = false;            // chest target in use
    math::Quat chestRot = math::identityQuat();  // chest orientation (character space; identity = upright, facing forward)
    bool hand[2] = {false, false}; // left, right hand targets in use
    math::Vec3 handGoal[2]{};      // wrist targets
};

class BodyReposition {
public:
    // Reads the rest pose's hips, legs and arms. Load-time only.
    bool init(const rig::Rig& rig, const ik::JointLimits& limits, const BodyRepositionJoints& joints);

    // Nothing carries over between ticks yet; kept for the callers.
    void reset() {}

    // Sets the pelvis, spine, legs and arms in use in `pose` (character
    // space). The pelvis starts at its standing spot over the character root
    // (which travels smoothly with the body). `models` is scratch of the
    // rig's size. No allocations.
    void solve(rig::Pose& pose, std::vector<math::Mat4>& models, const BodyTargets& t,
               const BodyRepositionSettings& s) const;

private:
    // Joint `joint` turned by `rotation` in model space, clamped to its limit.
    void rotateInModel(rig::Pose& pose, const std::vector<math::Mat4>& models, int joint, math::Quat rotation) const;

    const rig::Rig* rig_ = nullptr;
    const ik::JointLimits* limits_ = nullptr;
    BodyRepositionJoints joints_;
    math::Vec3 hipOffset_[2]{};    // hip from the pelvis (rest)
    math::Vec3 restPelvis_{0.0f};  // pelvis standing spot (rest, character space)
    math::Vec3 legDir_[2]{};       // rest hip-to-ankle direction (pelvis frame)
    float legLength_[2]{};         // thigh + shin
    float armLength_[2]{};         // upper arm + forearm
    math::Vec3 heel_[2]{}, ball_[2]{};  // sole contacts from the ankle (BodyRepositionJoints)
    float groundY_ = 0.0f;         // ground height (the rest sole, character space)
    float maxHeelLift_[2]{};       // rad, heel lift at which the toes reach their extension limit
};

} // namespace motion
