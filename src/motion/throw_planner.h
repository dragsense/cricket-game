#pragma once
// The throw as targets for the body's decider (as RunPlanner's: it only places
// the throwing hand's target over time; the decider knows nothing of it, the
// body follows the hand). Standing (the user's design, 2026-11-22):
// - Slot: the hand goes round the height circle to the clicked point (from what
//   height it throws).
// - Pull: the mouse draws the hand back from there (how far back: the force):
//   low the arm straight back round the shoulder, raised the elbow bent and the
//   forearm laid back; further the hand goes round the body (the decider turns
//   the spine, the hips, steps where the arm cannot); Held: a click keeps it.
//   The hand's target comes with its path (end, time left), as a mover's.
// - Swing: the hand from where it is drawn back, evenly faster round the throw
//   circle, late out onto it (the whole arm's length), through the clicked
//   release point (the throw's direction is the circle's there), and slower
//   past it; as a path, so the decider turns the body for it (its chain: the
//   arm, the trunk, the hips).
// - Uncoil (a throw let go): the pull forward again. Return: the hand back to
//   hanging.
// Character space. No allocations. BIOMECHANICS 10.

#include "math/math.h"
#include "motion/target_set.h"
#include "motion/throw_arc.h"

#include <algorithm>

namespace motion {

// config/throw.json.
struct ThrowPlannerSettings {
    float slotRadius = 1.0f, swingRadius = 1.0f;  // the circles, x the arm's length
    float slotMin = -1.047f, slotMax = 0.611f;    // rad, the slots the hand goes to (underarm .. overarm)
    float releaseMin = -0.524f, releaseMax = 2.094f;  // rad, the release points on the throw circle
    float pullMax = 1.1f;    // m, the whole force
    float pullReach = 0.95f; // x the arm's length: from here on round the shoulder
    float pullSpeed = 2.0f;  // m/s, the hand no faster after the mouse
    // The coil, from coilFrom (m of pull) to the whole force: the hand's turn
    // round the body meanwhile (rad); the body turns where the arm cannot (the
    // decider's chain: the spine, the hips, a step).
    float coilFrom = 0.55f, coilHand = 0.785f;
    // Raised (the arm from hanging, rad: straight below bentFrom, bent at
    // bentAt), the forearm lays back to the shoulder's external rotation at
    // the arm 0, 45, 90 deg from hanging (rad).
    float bentFrom = 0.785f, bentAt = 1.571f;
    float layback[3] = {1.134f, 1.745f, 2.269f};
    // The back foot's share of the weight with the whole coil (even: 0.5);
    // back to even by hipsBy of the time to the release.
    float backLoad = 0.8f;
    // The swing: its time to the release (s) from no force to all of it; by
    // which share of it the weight is even and the elbow open; how far the hand
    // goes on past the release (rad).
    float timeNone = 0.5f, timeFull = 0.36f;
    float hipsBy = 0.66f, elbowBy = 1.2f;
    float follow = 1.047f, followMax = 2.443f;
    // The decider's rates meanwhile (rad/s; Human sets them): a limb's, a trunk
    // joint's, the hips'.
    float limbRate = 40.0f, trunkRate = 10.0f, hipsRate = 10.0f;
    // The hand's time to a point: Fitts' law (human_joints.json "muscles").
    float fittsA = 0.1f, fittsB = 0.1f, fittsWidth = 0.1f;
};

// The standing body (rest pose) and the hand in the decider's plan now,
// character space.
struct ThrowBody {
    math::Vec3 restChest{0.0f}, restShoulder{0.0f}, restHand{0.0f};
    float armLength = 0.0f, upperArm = 0.0f;  // m
    math::Vec3 hand{0.0f};
    bool standing = true;
};

class ThrowPlanner {
public:
    void stop() { *this = ThrowPlanner{}; }
    bool on() const { return stage_ != Stage::Off; }
    bool pulling() const { return stage_ == Stage::Slot || stage_ == Stage::Pull; }  // the mouse draws the hand back
    bool held() const { return stage_ == Stage::Held; }  // the force is set: the next click is the release
    bool swinging() const { return stage_ == Stage::Swing; }

    // Click 1: the hand to the height circle's point at `angle`.
    void toSlot(float angle, const ThrowBody& body, const ThrowPlannerSettings& s);
    // The mouse: the hand `by` (m) further back (negative: forward again).
    void pull(float by, const ThrowPlannerSettings& s);
    // Click 2: the force as it is now.
    void hold();
    // Click 3: the hand forward through the throw circle's point at `angle`
    // (once the back foot stands).
    void swing(float angle, const ThrowBody& body, const ThrowPlannerSettings& s);
    // The throw circle's lean, about the line from the shoulder to the hand.
    void setTilt(float tilt) { tilt_ = tilt; }
    // Back to standing.
    void cancel(const ThrowBody& body, const ThrowPlannerSettings& s);
    // The character root moved (`toNew`: old character space to new).
    void rebase(const math::Transform& toNew);
    // One tick: writes the hand's target (coiled or swinging: the hips' and the
    // chest's, the back foot's) into `t`.
    void tick(const ThrowBody& body, const ThrowPlannerSettings& s, TargetSet& t);

    // The circles, fixed round the standing body.
    ThrowArc slotArc(const ThrowBody& body, const ThrowPlannerSettings& s) const;
    ThrowArc swingArc(const ThrowBody& body, const ThrowPlannerSettings& s) const;
    math::Vec3 slotPoint() const { return slotPoint_; }
    math::Vec3 at() const { return at_; }  // the hand's target now
    float releaseAngle() const { return release_; }
    float force(const ThrowPlannerSettings& s) const { return s.pullMax > 0.0f ? math::clamp(pull_ / s.pullMax, 0.0f, 1.0f) : 0.0f; }
    // The slot, the release for a click at `angle` on its circle.
    static float slotOf(float angle, const ThrowPlannerSettings& s) { return math::clamp(angle, s.slotMin, s.slotMax); }
    static float releaseOf(float angle, const ThrowPlannerSettings& s) {
        return math::clamp(angle, s.releaseMin, s.releaseMax);
    }

private:
    // Where the hand is drawn back to by a pull of `pull`.
    math::Vec3 pulled(const ThrowBody& body, const ThrowPlannerSettings& s, float pull) const;
    math::Vec3 pulledStraight(const ThrowBody& body, const ThrowPlannerSettings& s, float pull) const;
    // How far (0..1) the body is coiled at a pull of `pull`.
    static float coilOf(float pull, const ThrowPlannerSettings& s) {
        return math::clamp((pull - s.coilFrom) / std::max(s.pullMax - s.coilFrom, 1e-3f), 0.0f, 1.0f);
    }
    void startSwing(const ThrowBody& body, const ThrowPlannerSettings& s);
    // A path from where the hand's target is to `goal`, round `arc`.
    void start(math::Vec3 goal, const ThrowArc& arc, const ThrowPlannerSettings& s);

    enum class Stage { Off, Slot, Pull, Held, Swing, Uncoil, Return };
    Stage stage_ = Stage::Off;
    float time_ = 0.0f, duration_ = 0.0f;            // s, this stage
    float slot_ = 0.0f, tilt_ = 0.0f, pull_ = 0.0f;  // rad, rad, m
    float drawn_ = 0.0f;       // m, the pull the hand is at (after pull_ at pullSpeed)
    bool released_ = false;    // click 3 is in, the back foot not down yet
    // The swing: the time to the release, the back foot's load where it began;
    // where round the circle it began (angle, distance, depth), the release and
    // the end (rad), the rate's gain to the release.
    float toRelease_ = 0.0f, loadFrom_ = 0.5f;
    float angle_ = 0.0f, radius_ = 0.0f, depth_ = 0.0f;
    float release_ = 0.0f, end_ = 0.0f, accel_ = 0.0f;
    math::Vec3 swung(const ThrowBody& body, const ThrowPlannerSettings& s) const;
    math::Vec3 at_{0.0f}, from_{0.0f}, goal_{0.0f};  // the hand's target now, where the stage began, where it ends
    math::Vec3 slotPoint_{0.0f};
};

} // namespace motion
