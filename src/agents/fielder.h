#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, walk (move and
// turn the root, step the feet on the gait), reposition the body over the
// foot targets (late update), add pelvis bob, sway, lean and twist, solve two-bone
// IK for the legs, keep the feet flat, swing the arms opposite the legs,
// steady the head, reach for the hand target if there is one (standing
// only; docs/BIOMECHANICS.md section 9), then clamp every joint to its limits.

#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/body_reposition.h"
#include "motion/foot_stepper.h"
#include "motion/walk_controller.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

// IK goals in world space.
struct IkTargets {
    math::Vec3 footL{0.0f}, footR{0.0f};
};

struct IkSettings {
    math::Vec3 kneePole{0, 0, 1};
    math::Vec3 elbowPole{0.3f, 0, -1};
    float soften = 0.97f;
    float chestAimWeight = 0.3f;
    float headAimWeight = 1.0f;
};

// Which hand picks: the one on the ball's side, a fixed one, or both.
enum class HandMode { Auto, Left, Right, Both };

// Hand-driven reaching and picking (config/fielder_joints.json "reach").
struct ReachSettings {
    float armReach = 0.97f;         // share of full arm length the hand reaches; the elbow never locks
    float wristOffset = 0.08f;      // m; wrist goal back from the ball centre toward the shoulder (palm on the ball)
    // Spine stage: share of the bend and of the twist taken by spine_01,
    // spine_02 and chest (each sums to 1); iterations absorb clamping.
    math::Vec3 spineBend{0.6f, 0.2f, 0.2f};
    math::Vec3 spineTwist{0.2f, 0.4f, 0.4f};
    int spineIterations = 2;
    // Pelvis stage limits (radians), as a rotation vector in character space:
    // x = forward tilt (hip flexion; + forward), y = turn, z = side tilt.
    math::Vec3 pelvisMin{math::radians(-10.0f), math::radians(-30.0f), math::radians(-20.0f)};
    math::Vec3 pelvisMax{math::radians(80.0f), math::radians(30.0f), math::radians(20.0f)};
    // Balance: base of support = capsule this wide (m) around the foot
    // centres; the hips shift against the centre of mass's overhang x gain.
    float supportRadius = 0.12f;
    float balanceGain = 1.5f;
    float maxHipShift = 0.3f;  // m
    int balancePasses = 2;     // measured balance corrections after the leg IK
    // Elbow pole before it is made perpendicular to the arm: out (+ = away
    // from the body), down, back (- = behind), left arm; mirrored for the right.
    math::Vec3 elbowPole{0.4f, -1.0f, -0.6f};
    // Pick timing: the wrist goal eases from its rest spot to the ball over
    // reachTime, holds holdTime once on the ball (grip), then the ball is
    // carried back over returnTime (s).
    float reachTime = 0.45f;
    float holdTime = 0.15f;
    float returnTime = 0.6f;
    float gripTolerance = 0.02f;  // m; the hand counts as on the ball within this
    float giveUpTime = 0.4f;      // s at full stretch, out of reach, before letting go (the dive later)
    float ballRadius = 0.036f;    // m; two-hand mode puts a hand each side
    // Grip: full curl per joint [_01, _02, _03] once the hand is on the ball
    // and while holding it (degrees in config); relaxedCurl share otherwise.
    float relaxedCurl = 0.25f;
    // Palm: its centre is palmCentre (m) down the hand from the wrist; the
    // palm faces the side the fingers curl to (-X left, +X right); a held
    // ball sits ballRadius + palmThickness (m) out from the palm centre.
    float palmCentre = 0.05f;
    float palmThickness = 0.015f;
    math::Vec3 fingerCurl{math::radians(35.0f), math::radians(55.0f), math::radians(40.0f)};
    math::Vec3 thumbCurl{math::radians(10.0f), math::radians(30.0f), math::radians(25.0f)};
    // Lunge: out of reach at full stretch, the foot nearer the ball steps to
    // land lungeShort (m) short of it, at most maxLunge (m; 0 = off).
    float maxLunge = 0.9f;
    float lungeShort = 0.25f;
    float stepBeyond = 0.65f;     // m from the middle of the feet: step first, then reach
    float lungeDuration = 0.4f;   // s
    float lungeHeight = 0.06f;    // m swing arc apex
};

class Fielder {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/fielder_joints.json) and walk/run settings (config/locomotion.json),
    // and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion);
    // Re-reads config/locomotion.json without moving the fielder.
    void loadLocomotion(const core::Config& locomotion);
    // Rebuilds the skeleton at a new height (allocates; not per tick).
    bool setHeight(float height);

    void tick(double dt);

    const IkTargets& targets() const { return targets_; }
    // Requested walk in character space (x = left, y = forward), length 0..1
    // scales the speed; zero stops. With `face`, turns toward world heading
    // `faceYaw` (radians about +Y).
    void setMoveInput(math::Vec2 move, bool face, float faceYaw) {
        moveInput_ = move;
        face_ = face;
        faceYaw_ = faceYaw;
    }
    // Pace (config/locomotion.json "walk", "jog", "run"). The speed eases to
    // the new pace and the gait shape blends with the actual speed.
    void setGait(motion::Gait gait) { gait_ = gait; }
    motion::Gait gait() const { return gait_; }
    float speed() const { return walker_.speed(); }  // m/s

    // The ball (world space). Moving it drops it if held; it is only picked
    // on startPick().
    void setReachTarget(math::Vec3 world) {
        reachTarget_ = world;
        holding_ = false;
    }
    // Reach for the ball, grip it, carry it back to the standing pose. A lunge
    // step is taken if it is out of reach; walking input is ignored meanwhile.
    void startPick();
    // Abandon the pick: ease back to standing (keeps a held ball).
    void clearReach();
    bool reaching() const { return phase_ != Phase::Idle; }
    bool holding() const { return holding_; }
    void setHandMode(HandMode mode) { handMode_ = mode; }
    HandMode handMode() const { return handMode_; }
    // Held ball's centre in world space from a model-space pose (e.g. the
    // interpolated render pose); valid while holding().
    math::Vec3 ballInHand(const std::vector<math::Mat4>& models, const math::Transform& root) const;
    const ReachSettings& reachSettings() const { return reach_; }
    void setReachSettings(const ReachSettings& s) { reach_ = s; }
    // Distance (m) from the reaching hand's wrist to its goal after the last
    // tick (measured after the joint limits); 0 when reached or not reaching.
    float reachError() const { return reachError_; }
    // Whole-body centre of mass in world space, from the segment masses.
    math::Vec3 centreOfMass();

    const rig::SkeletonDef& def() const { return def_; }
    const rig::Rig& rig() const { return rig_; }
    const rig::Pose& pose() const { return current_; }
    // Character root (ground point under the pelvis) in world space.
    const math::Transform& root() const { return root_; }
    math::Transform interpolatedRoot(float alpha) const { return math::lerp(previousRoot_, root_, alpha); }

    // Pose blended between the last two ticks. `out` keeps its storage.
    void interpolatedPose(float alpha, rig::Pose& out) const;

private:
    void plantFeet();
    void walk(float dt);
    void repositionBody(float dt);
    void addGaitMotion(float dt);
    void swingArms(float dt);
    void stabiliseHead(float dt);
    void solveIk();
    // Reaching, one pass per stage (each takes the error left by the ones
    // before it): spine, pelvis, body before the leg IK; clavicle and arm last.
    void reachSpine();
    void reachPelvis();
    void reachBody();
    void rebalance();  // after the leg IK: measured centre of mass back over the feet
    // Pelvis move that cancels the centre of mass's overhang beyond the feet,
    // were the pelvis first moved by `pelvisMove` (models_ current).
    math::Vec3 balanceOverhang(math::Vec3 pelvisMove);
    math::Vec3 clampPelvisToLegs(math::Vec3 pelvis);  // both feet planted, no squat limit
    math::Vec3 modelCentreOfMass();  // uses models_, which must be current
    // Picking: advances the phase and sets this tick's wrist goals; after the
    // solve, grips, lunges or gives up.
    void updatePick(float dt);
    void finishPick(float dt);
    void lungeToward(math::Vec3 ball);  // character space
    // Clavicle + two-bone IK of one arm toward `goal`.
    void reachArm(bool left, math::Vec3 goal);
    void grip(bool left, float amount);  // curls the fingers and thumb, 0..1
    // Held ball's centre in the hand's joint space (in front of the palm).
    math::Vec3 ballSpot(bool left) const;
    math::Vec3 reachGoal() const { return goal_; }  // primary wrist goal, character space
    // Applies a model-space rotation to `joint` (about its own position),
    // clamped to its limit. `models_` must be current.
    void rotateInModel(int joint, math::Quat rotation);
    motion::GaitStyle gaitStyle() const;

    struct Joints {
        int pelvis = -1, chest = -1, head = -1, toeL = -1, toeR = -1, clavicleL = -1, clavicleR = -1;
        int spine[3] = {-1, -1, -1};  // spine_01, spine_02, chest
        int fingers[2][5][3] = {};     // [left/right][index, middle, ring, pinky, thumb][_01.._03]
        math::Vec3 curlAxis[2][5][3];  // local axis that turns each bone toward the palm
        ik::TwoBoneChain legL, legR, armL, armR;
    };

    rig::SkeletonDef def_;
    rig::Rig rig_;
    ik::JointLimits limits_;
    const core::Config* jointsConfig_ = nullptr;
    IkSettings ik_;
    ReachSettings reach_;
    float armLength_ = 0.6f;  // shoulder to wrist (m), from the skeleton
    math::Vec3 restShoulderL_{0.0f}, restShoulderR_{0.0f};  // character space, rest pose
    math::Vec3 restWristL_{0.0f}, restWristR_{0.0f};        // character space, rest pose
    math::Vec3 reachTarget_{0.0f};  // the ball, world space
    enum class Phase { Idle, Reach, Return };
    Phase phase_ = Phase::Idle;
    HandMode handMode_ = HandMode::Auto;
    float pickTime_ = 0.0f;   // 0..1 progress of the reach (back to 0 on return)
    float holdTimer_ = 0.0f;  // s on the ball
    float stuckTimer_ = 0.0f; // s at full stretch, out of reach, after the lunge
    bool lunged_ = false;
    bool holding_ = false;
    math::Vec3 carryFrom_{0.0f};  // wrist goal where the ball was gripped, character space
    bool reachLeft_ = true;       // this tick's (primary) reaching hand
    bool bothHands_ = false;
    math::Vec3 goal_{0.0f}, otherGoal_{0.0f};  // wrist goals, character space
    float reachError_ = 0.0f;
    motion::RepositionSettings reposition_;
    float stanceBlendHeight_ = 0.1f;
    float ballDistance_ = 0.1f;  // ankle to ball of the foot (m), from the skeleton
    void blendShape();

    motion::WalkSettings walkSettings_, jogSettings_, runSettings_;  // jog starts from walk, run from jog
    motion::WalkSettings control_;  // chosen pace: target speeds, acceleration, turn rate
    motion::WalkSettings shape_;    // gait shape blended by the actual speed
    motion::Gait gait_ = motion::Gait::Walk;
    motion::WalkController walker_;
    motion::FootStepper stepper_;
    math::Vec2 moveInput_{0.0f};
    bool face_ = false;
    float faceYaw_ = 0.0f;
    math::Vec2 lean_{0.0f};  // smoothed (forward, into-turn) lean, radians
    float accel_ = 0.0f;     // smoothed forward acceleration for the lean, m/s^2
    float legPhase_ = 0.0f;  // left leg fore-aft position, -1 back .. +1 forward
    float armAmount_ = 0.0f; // smoothed arm swing scale 0..1
    float headYaw_ = 0.0f;   // smoothed head turn ahead of the body, radians
    Joints joints_;
    IkTargets targets_;
    math::Vec3 pelvis_{0.0f};  // pelvis position in character space, kept between ticks
    rig::Pose previous_, current_;
    // Segment mass model (built with the skeleton): mass share per joint, its
    // single child (segment end) or -1, and the leaf tip in joint space.
    std::vector<float> segmentMass_;
    std::vector<int> segmentEnd_;
    std::vector<math::Vec3> segmentTip_;
    float armMassShare_ = 0.05f;     // share of the mass in one arm
    float pelvisMassShare_ = 0.75f;  // share of the mass a pelvis move carries (not the planted lower legs)
    std::vector<math::Mat4> models_;  // scratch, sized at build
    math::Transform root_, previousRoot_;
};

} // namespace agents
