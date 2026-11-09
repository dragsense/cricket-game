#pragma once
// One procedural fielder: skeleton, pose and placement on the field. Advanced
// only by fixed sim ticks; keeps the previous and current pose so rendering
// can interpolate. Each tick: start from the standing pose, walk (move and
// turn the root, step the feet on the gait), reposition the body over the
// foot targets (late update), add pelvis bob, sway, lean and twist, solve two-bone
// IK for the legs, keep the feet flat, swing the arms opposite the legs,
// steady the head, reach for the hand target if there is one (standing
// only; docs/BIOMECHANICS.md section 9), then clamp every joint to its limits.

#include "agents/human_body.h"
#include "ik/joint_limits.h"
#include "ik/limb_ik.h"
#include "math/math.h"
#include "motion/foot_stepper.h"
#include "motion/body_decider.h"
#include "motion/throw_arc.h"
#include "motion/walk_controller.h"
#include "rig/rig.h"
#include "rig/skeleton_def.h"

#include <vector>

namespace core {
class Config;
}

namespace agents {

// The old formula repositioning (moved here from motion/body_reposition_backup,
// 2026-11-06), still used by jogging, running, jumping, picking, throwing and
// bowling until they move to the physics decider (DECISIONS 2026-11-06).
// One formula, whatever moved a foot:
//  - where: between the feet by the weight each carries;
//  - how high: as high as every loaded leg reaches straight (standReach), the
//    lowest deciding; a lifted foot sets nothing;
//  - on one foot the lifted side drops stanceDrop (BIOMECHANICS 4.3);
//  - a damped spring moves it there; the leg clamp keeps both feet reachable.
struct RepositionSettings {
    float standReach = 0.95f;  // comfortable hip-to-foot distance (fraction of leg length)
    float minReach = 0.5f;     // deepest crouch (the knee fully bent; set from the joint limits)
    float maxReach = 0.98f;    // legs never fully straight
    float softBand = 0.02f;    // the max reach eases in over this share of leg length (0: hard)
    float followRate = 10.0f;  // 1/s, the spring's natural frequency
    float damping = 1.0f;      // spring damping ratio (1: settles without overshoot)
    float sideFollow = 1.0f;   // share of the sideways shift over the stance foot the body takes
    float forwardFollow = 1.0f; // share of the forward/back shift over the feet (0: stay at restZ)
    float restZ = 0.0f;        // pelvis forward position when not following the feet
    float stanceDrop = 0.07f;  // radians, the unloaded side's hip drop with all weight on one leg
    float standShare = 1.0f;   // share of the standing rules (straight-leg height, hip drop); 0 moving
    int iterations = 4;
};

struct LegSupport {
    math::Vec3 hipOffset{0.0f};  // hip relative to the pelvis (rest, unrotated)
    math::Vec3 foot{0.0f};       // foot (ankle) target
};

// The pelvis kept between ticks: position and hip drop, each with its spring
// velocity, and the position it is heading for.
struct BodyState {
    math::Vec3 position{0.0f};
    math::Vec3 velocity{0.0f};
    math::Vec3 goal{0.0f};
    float drop = 0.0f, dropRate = 0.0f;  // about +Z; positive raises the left hip

    math::Quat rotation() const { return math::fromAxisAngle(math::kForward, drop); }
};

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

// Which hand picks: the one on the ball's side (default), or a fixed one.
enum class HandMode { Auto, Left, Right };

// Hand-driven reaching and picking (config/human_joints.json "reach").
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
    // Pick timing (BIOMECHANICS 9.6): the wrist goal eases from its rest spot
    // to the ball and back over Fitts' law times a + b log2(2D / W) for the
    // distance D the hand travels.
    float fittsA = 0.1f;      // s
    float fittsB = 0.1f;      // s per bit
    float fittsWidth = 0.1f;  // m, target width (hand around the ball)
    float gripTolerance = 0.02f;  // m; the hand counts as on the ball within this
    float ballRadius = 0.036f;    // m; a held ball sits this far out from the palm (plus its thickness)
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
    // Lunge (a pick with a step): the foot nearer the ball steps to land
    // lungeShort (m) short of it, at most maxLunge (m; 0 = off).
    float maxLunge = 0.9f;
    float lungeShort = 0.25f;
    float lungeDuration = 0.4f;   // s
    float lungeHeight = 0.06f;    // m swing arc apex
};

class Human {
public:
    // Builds the skeleton at `height` metres, loads joint limits and IK settings
    // (config/human_joints.json) and walk/run settings (config/locomotion.json),
    // and stands it at the spawn point.
    bool init(const rig::SkeletonDef& def, float height, const core::Config& joints, const core::Config& locomotion);
    // The body in the shared physics world (DECISIONS 2026-11-09 "Full
    // physics body on the capsule"): a capsule (Jolt CharacterVirtual)
    // carries the player over the ground; the body is a full-physics Jolt
    // Ragdoll whose muscles (Jolt's pose drive) pull it toward this tick's
    // pose, held up by a spring from the capsule to the pelvis. Leaning past
    // the release angles lets go: the body falls, then recovers. The feet
    // stand on the world's real surface (Jolt ground query). After init();
    // again after the world is rebuilt. False if the world is full.
    bool attachPhysics(physics::PhysicsWorld& world);
    // The capsule holds the body (not let go after a lean).
    bool supported() const { return supportOn_; }
    // Debug drawing: the capsule (its feet, height and radius; world) and
    // the line the release reads, from the centre of the feet to the hips
    // (the physics body, world). False without a body.
    bool capsuleDebug(math::Vec3& feet, float& height, float& radius, math::Vec3& feetCentre, math::Vec3& hips) const;
    const HumanBody& humanBody() const { return humanBody_; }
    HumanBody& humanBody() { return humanBody_; }
    const ik::JointLimits& jointLimits() const { return limits_; }
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
    // Single steps while standing (character space, x = left, y = forward;
    // zero for none): each step swings one foot (locomotion.json "step") and
    // the body follows from the stance solve. Held, the steps go on.
    void setStepInput(math::Vec2 dir) { stepInput_ = dir; }
    float speed() const { return walker_.speed(); }  // m/s
    // Stands the fielder at `position` (world) facing `yaw` radians, feet at
    // rest, any jump cleared.
    void placeAt(math::Vec3 position, float yaw);

    // The ball (world space). Moving it drops it if held; it is only picked
    // on startPick().
    void setReachTarget(math::Vec3 world, math::Vec3 velocity = math::Vec3(0.0f)) {
        reachTarget_ = world;
        reachVelocity_ = velocity;
        holding_ = false;
    }
    // Reach for the ball at once (the hand follows it if it moves); when the
    // reach is done, a hand on the ball grips it, and either way the body
    // goes back to the standing pose (an out-of-reach ball is left). With `step`, the
    // foot nearer the ball lunges first and the reach starts as it lands.
    // Walking input is ignored meanwhile; clearReach() abandons it.
    void startPick(bool step = false);
    // Abandon the pick: ease back to standing (keeps a held ball).
    void clearReach();
    bool reaching() const { return phase_ != Phase::Idle; }
    float reachProgress() const { return pickTime_; }        // 0..1 of the reach (back to 0 on return)
    bool holding() const { return holding_; }
    // Puts the ball in the holding hand (e.g. a new ball for the bowler).
    void giveBall() { holding_ = true; }
    void setHandMode(HandMode mode) { handMode_ = mode; }
    HandMode handMode() const { return handMode_; }
    // Held ball's centre in world space from a model-space pose (e.g. the
    // interpolated render pose); valid while holding().
    math::Vec3 ballInHand(const std::vector<math::Mat4>& models, const math::Transform& root) const;
    bool holdingLeft() const { return reachLeft_; }  // the hand that holds (or last reached)
    // Shoulder of the holding arm in world space, from a model-space pose.
    math::Vec3 holdingShoulder(const std::vector<math::Mat4>& models, const math::Transform& root) const {
        const ik::TwoBoneChain& arm = reachLeft_ ? joints_.armL : joints_.armR;
        return root.transformPoint(math::Vec3(models[static_cast<size_t>(arm.start)][3]));
    }
    float armLength() const { return armLength_; }  // shoulder to wrist (m)
    // Throw, in three steps, each acting at once (with the ball held and
    // standing): beginThrow moves the hand to the slot point (`slotAngle` on
    // slotArc(), a full circle around the chest) and holds it; windThrow
    // turns the whole trunk back to `backAngle` on chestArc(slotAngle, tilt),
    // the hand keeping its place on the trunk,
    // (motion::makeSwingArc: through the slot point and the facing direction)
    // (< 0; the farther back, the more force) and holds; releaseThrow turns
    // it forward, speeding up, the hand moving out onto the circle and on
    // along it, pushing out, to `releaseAngle`,
    // where the ball leaves along the circle's tangent; then the
    // follow-through, the hand returns and the chest turns back square.
    // cancelThrow returns the hand before the release. Arcs in character
    // space.
    void beginThrow(float slotAngle);
    void windThrow(float tilt, float backAngle);
    // Before the release: leans the chest circle (`tilt`), a wound-up chest
    // and hand turning with it.
    void tiltThrow(float tilt);
    void releaseThrow(float releaseAngle);
    void cancelThrow();
    // Test (ThrowSettings::pauseAtRelease): the hand waits at the release
    // point; letGo lets the ball go and the throw carries on.
    bool throwPaused() const { return throwing() && throwStage_ == ThrowStage::Swing && holding_ && throwHeld_; }
    void letGo() { letGo_ = true; }
    motion::ThrowArc slotArc() const;
    motion::ThrowArc chestArc(float slotAngle, float tilt) const;
    bool throwing() const { return phase_ == Phase::Throw; }
    bool throwSwinging() const { return phase_ == Phase::Throw && throwStage_ == ThrowStage::Swing; }
    // True once, on the tick the ball leaves the hand: its centre and
    // velocity in world space. holding() is false from then on.
    bool takeRelease(math::Vec3& position, math::Vec3& velocity) {
        if (!released_) return false;
        released_ = false;
        position = releasePosition_;
        velocity = releaseVelocity_;
        return true;
    }
    void setThrowSettings(const motion::ThrowSettings& s) { throw_ = s; }
    // Bowl (M5): with the ball held and standing, the hand goes round the
    // full bowling circle (motion::makeBowlingArc leaned by `tilt`): from in
    // front, down, back, over the top, through `releaseAngle` and on into the
    // follow-through across the body, then returns; the arm stays nearly
    // straight and the body follows through the reach stages. The ball leaves
    // at the release point along the circle's tangent (takeRelease).
    void startBowl(float tilt, float releaseAngle);
    bool bowling() const { return phase_ == Phase::Bowl; }
    // Bowling action (M6), two clicks, each acting at once (ball held,
    // standing). Only the bowling hand is driven; the body follows it through
    // the reach stages (reverse IK). actionBowl: the action point `point`
    // (character space, on a sphere round the bowling shoulder); the hand
    // goes there from rest along actionPathPoint, overshooting by `gap` (m),
    // then is pulled back onto it (the wrist loads) and holds.
    // releaseBowl: the straight arm goes on round the bowling way (release
    // circle's angles: down past the bottom, back, over) to low behind the
    // hip, then eases onto the release circle (bowlArc(`releaseTilt`)) and
    // the ball leaves at `releaseAngle` along its tangent; the arm carries
    // on, slowing, and the body folds after it. cancelCoil returns before the release.
    // `turn`: the action circle's (radians); turned inward (negative), both
    // elbows swivel outward-up about their shoulder-to-wrist lines, fully
    // at BowlSettings::elbowUpTurn.
    void actionBowl(math::Vec3 point, float gap, float turn);
    // The hand's way from rest to `point` at u = 0..1: round the shoulder in
    // the plane of the shoulder, the resting hand and the point, the radius
    // growing from the resting hand's to the point's + `gap`. Character space.
    math::Vec3 actionPathPoint(math::Vec3 point, float gap, float u) const;
    // Round the shoulder from `from` to `to` at u = 0..1 (the short way, in
    // their plane), the radius going between theirs. Character space.
    math::Vec3 arcBetween(math::Vec3 from, math::Vec3 to, float u) const;
    // Straight below the shoulder by the arm: where the hand's way from
    // the action point ends and the release circle starts. Character space.
    math::Vec3 bottomPoint() const;
    // A shoulder as the chest now carries it (the rest one when not
    // bowling), and the resting hand carried with it. Character space.
    math::Vec3 shoulderNow(bool left) const;
    math::Vec3 restHandNow(bool left) const;
    // The front hand's full way at u = 0..1: from rest round its own
    // shoulder (via the front) to the end of the bowling hand's way to
    // `point` (with `gap`), kept apart from it by BowlSettings::frontGap per
    // axis. frontDownPoint: back down the same way. Character space.
    math::Vec3 frontPathPoint(math::Vec3 point, float gap, float u) const;
    math::Vec3 frontDownPoint(math::Vec3 point, float gap, float u) const;
    // Round `shoulder` from `from` to `to` at u = 0..1 (the short way, in
    // their plane; with `viaFront`, never setting off backward), the radius
    // going between theirs.
    static math::Vec3 arcAround(math::Vec3 shoulder, math::Vec3 from, math::Vec3 to, float u, bool viaFront = false);
    // Action circle (for picking the action point): round the bowling
    // shoulder in the forward-up plane (seen from the side), angle 0 straight
    // down, +90 straight ahead; turned about the vertical toward the
    // bowling-arm side by `turn` (radians); `radius` (m). Character space.
    motion::ThrowArc actionCircle(float turn, float radius) const;
    void releaseBowl(float releaseAngle, float releaseTilt);
    void cancelCoil();
    bool coiled() const { return phase_ == Phase::Bowl && bowlStage_ == BowlStage::Coil; }
    // At the action point and holding (the way there and the pull back done);
    // downBowl acts only now and sends the hand on down to the bottom point.
    bool actionHeld() const { return coiled() && throwTime_ >= coilReach() + bowl_.pullTime; }
    // s the hand takes up to the action point.
    float coilReach() const { return bowl_.coilTime; }
    void downBowl();
    // The whole-player turn has reached bowlYaw_ (within BowlSettings::turnDone).
    bool bodyTurned() const;
    // Facing before the bowling turn (radians): the release circle is drawn in it.
    float bowlBaseYaw() const { return bowlBaseYaw_; }
    // Down at the bottom point and holding; releaseBowl acts only now.
    bool armDown() const { return phase_ == Phase::Bowl && bowlStage_ == BowlStage::Down; }
    bool bowlSwinging() const { return phase_ == Phase::Bowl && bowlStage_ == BowlStage::Swing; }
    // Arm circle around the bowling shoulder, square to the batter, leaned
    // by `tilt`; fixed while the body moves (the action point and the
    // release point are on it; the ball leaves on its tangent). Character space.
    motion::ThrowArc bowlArc(float tilt) const;
    // Limb test (physics only, the user's call): a wrist goal for one hand or
    // an ankle goal for one foot (world space) for the physics body; only the
    // limb moved last has a target, the rest holds and balances. Nothing
    // while physics is off. clearPose() drops the goals.
    void setPoseHand(bool left, math::Vec3 world);
    void setPoseFoot(motion::Foot foot, math::Vec3 world);
    void clearPose();
    // Every part back at the planned standing pose, at rest, the hold on
    // again; the muscles pull from there.
    void resetPhysicsBody();
    bool posing() const { return phase_ == Phase::Pose; }
    // Wrist of one hand in world space, from a model-space pose.
    math::Vec3 wrist(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.armL : joints_.armR).end)][3]));
    }
    // Ankle of one foot in world space, from a model-space pose.
    math::Vec3 ankle(bool left, const std::vector<math::Mat4>& models, const math::Transform& root) const {
        return root.transformPoint(math::Vec3(models[static_cast<size_t>((left ? joints_.legL : joints_.legR).end)][3]));
    }
    void setBowlSettings(const motion::BowlSettings& s) { bowl_ = s; }
    float bowlRadius() const { return motion::bowlRadius(armLength_, bowl_.elbowBend); }
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
    // The drawn skeleton from the physics body (character space of the
    // interpolated root; `models` sized to the rig). False without a body.
    bool physicalModels(const physics::PhysicsWorld& world, float alpha, std::vector<math::Mat4>& models) const;

private:
    void plantFeet();
    void walk(float dt);
    // The root's move this tick (from `from`, where walk() found it) goes
    // through the capsule (Jolt CharacterVirtual): it slides along what it
    // hits and keeps to the ground; the root takes where it ends up, the
    // planted feet staying where they are in the world.
    void followCapsule(math::Vec3 from, float dt);
    void repositionBody(float dt);
    void addGaitMotion(float dt);
    void swingArms(float dt);
    void stabiliseHead(float dt);
    void solveIk();
    // Reaching, one pass per stage (each takes the error left by the ones
    // before it): spine, pelvis, body before the leg IK; clavicle and arm last.
    void twistChest();  // applies chestTwist_ over the pelvis and spine
    math::Vec3 heldHand(float twist) const;  // slot point carried by the trunk's turn, character space
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
    // solve, grips once the hand is on the ball.
    void updatePick(float dt);
    void finishPick(float dt);
    void updateThrow(float dt);  // sets this tick's wrist goal on the throw path
    void updateBowl(float dt);   // sets this tick's wrist goal on the bowling circle

    void setActionPoint(math::Vec3 point);  // character space
    void lungeToward(math::Vec3 ball);  // character space
    void startReturn();                  // carry back from goal_, Fitts timed
    float handTime(float distance) const;  // Fitts' law movement time
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
    math::Vec3 restChest_{0.0f};  // character space, rest pose (centre of both throw circles)
    // Shoulders as the chest carries them (character space; updated each
    // tick while bowling, at rest otherwise) and in the chest's frame.
    math::Vec3 liveShoulderL_{0.0f}, liveShoulderR_{0.0f};
    math::Vec3 chestShoulderL_{0.0f}, chestShoulderR_{0.0f};
    math::Vec3 restWristL_{0.0f}, restWristR_{0.0f};        // character space, rest pose
    math::Vec3 reachTarget_{0.0f};  // the ball, world space
    math::Vec3 reachVelocity_{0.0f};  // world, m/s
    bool poseHand_ = false;           // limb test: a hand has a wrist goal
    math::Vec3 poseGoal_{0.0f};       // limb test wrist goal, world space
    enum class Phase { Idle, Reach, Return, Throw, Bowl, Pose };
    motion::ThrowSettings throw_;
    motion::BowlSettings bowl_;
    float throwTilt_ = 0.0f, releaseAngle_ = 0.0f;  // bowling circle lean and release (radians)
    float throwTime_ = 0.0f;   // s since the throw started
    math::Vec3 throwFrom_{0.0f};  // wrist goal when the throw started, character space
    enum class BowlStage { Circle, Coil, Back, Down, Swing };
    BowlStage bowlStage_ = BowlStage::Circle;
    float bowlPace_ = 0.0f;  // 0..1 from the action point (behind = 1)
    float coil_ = 0.0f;      // chest-bone turn from the action point (radians)
    float bowlBaseYaw_ = 0.0f, bowlYaw_ = 0.0f;  // facing before the action, and the one turned to now (radians)
    float bodyTurn_ = 0.0f;  // whole-player turn at the bottom point (radians, signed)
    int strideFoot_ = -1;    // the foot that strode (motion::Foot), pointing straight; -1 none
    bool bowlTwist_ = false; // chestTwist_ is the bowling turn (its pelvis share), not the throw's
    float chestLean_ = 0.0f;  // chest-bone turn now (radians)
    float elbowSwivel_ = 0.0f;  // 0..1 share of the elbow hint moved to outward-up (inward circle turn)
    math::Vec3 leanAxis_{0.0f, 1.0f, 0.0f};  // character space (vertical)
    math::Vec3 frontGoal_{0.0f};  // front (non-bowling) wrist goal while bowling, character space
    float swingFrom_ = 0.0f, swingTo_ = 0.0f;  // action and release angles on bowlArc, unwrapped
    float releaseTilt_ = 0.0f; // release circle lean (radians)
    math::Vec3 actionOffset_{0.0f};  // action point from the bowling shoulder (it rides the shoulder)
    math::Vec3 backPoint_{0.0f};    // bottom point, where the release circle starts, character space
    float actionGap_ = 0.1f;       // the hand overshoots the action point by this (m)
    enum class ThrowStage { Ready, Wind, Swing };
    ThrowStage throwStage_ = ThrowStage::Ready;
    float throwSlot_ = 0.0f, throwBack_ = 0.0f;  // radians (beginThrow, windThrow)
    float twistFrom_ = 0.0f;      // chest turn when the current stage began
    bool strideWaiting_ = false;  // the swing waits for the stride foot to land
    bool throwHeld_ = false, letGo_ = false;  // pauseAtRelease: waiting, and asked to go on
    float chestTwist_ = 0.0f;     // current throw angle on the circle; the trunk turns turnShare of it
    math::Vec3 twistAxis_{0.0f, 1.0f, 0.0f};  // character space
    bool released_ = false;    // the ball left the hand this tick (takeRelease)
    math::Vec3 releasePosition_{0.0f}, releaseVelocity_{0.0f};  // world
    float pickDuration_ = 0.4f, returnDuration_ = 0.5f;  // s, Fitts' law per pick
    Phase phase_ = Phase::Idle;
    HandMode handMode_ = HandMode::Auto;
    float pickTime_ = 0.0f;   // 0..1 progress of the reach (back to 0 on return)
    bool holding_ = false;
    math::Vec3 carryFrom_{0.0f};  // wrist goal where the ball was gripped, character space
    bool reachLeft_ = true;       // this tick's (primary) reaching hand
    math::Vec3 goal_{0.0f};  // wrist goal, character space
    float reachError_ = 0.0f;
    RepositionSettings reposition_;
    float stanceBlendHeight_ = 0.1f;
    int posedFoot_ = -1;  // limb test: the foot last placed (motion::Foot), -1 none
    bool poseHandLast_ = false;  // limb test: the hand was moved after the foot (only it has a target)
    math::Vec3 poseFootGoal_{0.0f};  // limb test ankle goal, world space
    // A lifted foot's ankle stays at least this high (character space): its
    // standing height plus the minimum toe clearance (locomotion.json
    // "step.toeClearance"), so its target never runs along or into the ground.
    float toeClearance_ = 0.012f;
    float liftedAnkleMin_ = 0.0f;
    // Physics: a hand's or foot's commanded point travels from where it was
    // to its goal in Fitts' time on a minimum-jerk path (Flash & Hogan 1985);
    // the IK solves for it (character space). A goal moved mid-way retargets.
    struct GoalPath {
        math::Vec3 from{0.0f}, to{0.0f}, at{0.0f};
        float time = 0.0f, duration = 0.0f;
        int end = -1;  // the rig joint it moves (wrist or ankle)
        bool on = false;
    };
    GoalPath handPath_, footPath_;
    math::Vec2 stepInput_{0.0f};    // single steps: requested direction (character space)
    bool stepWalk_ = false;         // single steps in use (standing)
    motion::StepSettings stepSettings_;
    float heelLift_[2] = {0.0f, 0.0f};         // radians, smoothed heel lift past the ankle's range (solveIk)
    static constexpr float kHeelRate = 15.0f;  // 1/s, how fast the heel lift follows (no flicker)
    float tickSeconds_ = 1.0f / 120.0f;        // this tick's step
    HumanBody humanBody_;                  // the full-physics body (Jolt Ragdoll)
    physics::PhysicsWorld* physics_ = nullptr;
    physics::CharacterId capsule_ = physics::kInvalidCharacter;  // carries the root over the ground
    float restComHeight_ = 0.0f;               // m, the centre of mass above the root at rest
    float restPelvisHeight_ = 1.0f;            // m, the pelvis above the root at rest
    // The capsule's hold of the body: a hidden kinematic body riding the
    // capsule's feet, sprung to the pelvis (physics_world.json "character").
    physics::BodyId anchor_ = physics::kInvalidBody;
    physics::JointId support_ = physics::kInvalidJoint;
    bool supportOn_ = true;
    float supportShare_ = 1.0f;  // 0..1, eased in after a recovery
    float restTime_ = 0.0f;      // s the let-go body has been at rest
    bool armed_ = true;          // the lean check is on (off after a fall until the body is back up)
    // Each tick, after the pose: the muscles pull toward `target` (character
    // space), the hold follows the capsule, the capsule takes the pelvis's
    // height; a lean past the release angles lets go, rest brings it back.
    void physicsTick(const std::vector<math::Mat4>& target, float dt);
    // The limb test's goals through the decider: its plan (character space),
    // or null without goals.
    const std::vector<math::Mat4>* limbTestPlan(float dt);
    motion::BodyDecider decider_;
    motion::BodyDeciderJoints deciderJoints_;
    motion::BodyDeciderSettings deciderSettings_;
    motion::BodyQueue queue_;
    MuscleTiming muscleTiming_;
    std::vector<uint8_t> blocked_;            // per rig joint, for the decider (none blocked)
    std::vector<math::Mat4> physicsModels_;   // the physics body now (character space)
    rig::Pose physicsPose_;
    float ballDistance_ = 0.1f;  // ankle to ball of the foot (m), from the skeleton
    float ballDrop_ = 0.03f;     // ball of the foot below the ankle (m), from the skeleton

    motion::WalkSettings walkSettings_;  // the only pace (jog and run removed 2026-11-09)
    float postureLean_ = 0.087f;  // rad, walking posture: trunk held slightly forward
    motion::WalkController walker_;
    math::Vec3 releaseShift_{0.0f};  // release circle moved toward the batter during the swing, character space
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
    BodyState body_;  // pelvis position and foot-driven turn/drop in character space, kept between ticks
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
