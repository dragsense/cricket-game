#pragma once
// The throw as targets for the body's decider (as RunPlanner's: it only places
// the throwing hand's target over time; the decider knows nothing of it, the
// body follows the hand). Standing (the user's design, 2026-11-22):
// - Slot: the hand goes round the height circle to the clicked point (from what
//   height it throws).
// - Pull: the mouse draws the hand back from there (how far back: the force):
//   straight back, then, the arm out at its length, round the shoulder; Held: a
//   click keeps it.
// - Swing: the hips turn forward, then the chest; the hand rides the chest, the
//   arm still bent, and late comes out onto the throw circle (the whole arm's
//   length) through the clicked release point (the throw's direction is the
//   circle's there), and slows down past it (proximal to distal, BIOMECHANICS
//   10.5).
// - Return: the hand back to hanging.
// Character space. No allocations. BIOMECHANICS 10.

#include "math/math.h"
#include "motion/target_set.h"
#include "motion/throw_arc.h"

namespace motion {

// config/throw.json.
struct ThrowPlannerSettings {
    float slotRadius = 1.0f, swingRadius = 1.0f;  // the circles, x the arm's length
    float slotMin = -1.047f, slotMax = 0.611f;    // rad, the slots the hand goes to (underarm .. overarm)
    float releaseMin = -0.524f, releaseMax = 2.094f;  // rad, the release points on the throw circle
    float pullMax = 0.9f;    // m, the whole force
    float pullReach = 0.95f; // x the arm's length: from here on round the shoulder
    float pullSpeed = 2.0f;  // m/s, the hand no faster after the mouse
    // The swing: its time to the release (s) from no force to all of it; the
    // share of it the hand rides the chest; by which share the hips, the chest
    // and (a share of the rest) the elbow have turned; how far past square the
    // chest goes with all the force, how far the hand goes on past the release
    // (rad).
    float timeNone = 0.5f, timeFull = 0.36f, ride = 0.7f;
    float hipsBy = 0.66f, chestBy = 0.9f, elbowBy = 1.2f;
    float chestPast = 0.349f, follow = 1.047f, followMax = 2.443f;
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
    float armLength = 0.0f;  // m
    math::Vec3 hand{0.0f};
    math::Transform chest, pelvis;  // in the plan now
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
    // Click 3: the hand forward through the throw circle's point at `angle`.
    void swing(float angle, const ThrowBody& body, const ThrowPlannerSettings& s);
    // The throw circle's lean, about the line from the shoulder to the hand.
    void setTilt(float tilt) { tilt_ = tilt; }
    // Back to standing.
    void cancel(const ThrowBody& body, const ThrowPlannerSettings& s);
    // The character root moved (`toNew`: old character space to new).
    void rebase(const math::Transform& toNew);
    // One tick: writes the hand's target into `t`.
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
    // Where the hand is drawn back to now.
    math::Vec3 pulled(const ThrowBody& body, const ThrowPlannerSettings& s) const;
    // A path from where the hand's target is to `goal`, round `arc`.
    void start(math::Vec3 goal, const ThrowArc& arc, const ThrowPlannerSettings& s);

    enum class Stage { Off, Slot, Pull, Held, Swing, Return };
    Stage stage_ = Stage::Off;
    float time_ = 0.0f, duration_ = 0.0f;            // s, this stage
    float slot_ = 0.0f, tilt_ = 0.0f, pull_ = 0.0f;  // rad, rad, m
    // The swing: the time to the release, the chest's and the hips' turn where
    // it began, the hand in the chest's frame (riding it); off it: since when,
    // where round the circle (angle, rate, distance, depth), the release and the
    // end (rad), the rate there.
    float toRelease_ = 0.0f, chestFrom_ = 0.0f, hipsFrom_ = 0.0f, past_ = 0.0f;
    math::Vec3 riding_{0.0f};
    bool off_ = false;
    float offAt_ = 0.0f, angle_ = 0.0f, rate_ = 0.0f, radius_ = 0.0f, depth_ = 0.0f, lastAngle_ = 0.0f;
    float release_ = 0.0f, end_ = 0.0f, accel_ = 0.0f;
    math::Vec3 swung(const ThrowBody& body, const ThrowPlannerSettings& s, float dt);
    math::Vec3 at_{0.0f}, from_{0.0f}, goal_{0.0f};  // the hand's target now, where the stage began, where it ends
    math::Vec3 slotPoint_{0.0f};
};

} // namespace motion
