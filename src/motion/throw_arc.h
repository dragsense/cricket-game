#pragma once
// Throw arcs (M4). Both are half circles around the throwing shoulder, as
// angles on a circle: point(a) = centre + radius (cos a side + sin a up).
// - Slot arc: square to the facing direction, like a clock face seen from
//   the front; -90 deg straight down (underarm), 0 out to the side (sidearm),
//   +90 overhead (overarm). The user picks the slot point on it.
// - Swing arc: through the slot point (angle 0) and the facing direction
//   (+90), turned by `tilt` about the shoulder-to-slot line. The hand starts
//   at -90 (behind), swings forward and releases at a chosen angle, moving
//   along the circle's tangent.
// See docs/BIOMECHANICS.md section 10. Pure math, no allocations.

#include "math/math.h"

#include <cmath>

namespace motion {

struct ThrowArc {
    math::Vec3 centre{0.0f};      // throwing shoulder
    math::Vec3 side{1, 0, 0};     // unit, angle 0
    math::Vec3 up{0, 1, 0};       // unit, angle +90 deg
    math::Vec3 normal{0, 0, 1};   // unit, the circle's plane normal
    float radius = 0.6f;

    // Point at `angle` (radians); `reach` scales the radius (< 1 bends the elbow).
    math::Vec3 point(float angle, float reach = 1.0f) const;
    // Unit direction of travel at `angle` toward larger angles.
    math::Vec3 tangent(float angle) const;
    // Angle of the circle point nearest `p` (p projected onto the plane).
    float angleOf(math::Vec3 p) const;
    // Angle where the ray hits the circle's plane, snapped to the circle.
    // False if the ray is parallel to the plane or points away from it.
    bool rayAngle(math::Vec3 origin, math::Vec3 direction, float& angle) const;
};

// Slot arc for a fielder facing `rootRotation` (character +Z forward, +X left).
ThrowArc makeThrowArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius);
// Swing arc through the slot arc's point at `slotAngle`, tilted by `tilt` radians.
ThrowArc makeSwingArc(const ThrowArc& slotArc, float slotAngle, float tilt);

// Bowling circle (M5): a full circle around the bowling shoulder through the
// top and the facing direction; the swing arc of the slot point leaned
// `tilt` radians from vertical toward the bowling-arm side. Angle 0 is the
// top, +90 deg in front, -90 behind, +-180 below; the hand travels toward
// larger angles (BIOMECHANICS 11.2).
ThrowArc makeBowlingArc(math::Vec3 shoulder, math::Quat rootRotation, bool leftArm, float radius, float tilt);

// config/throw.json (BIOMECHANICS 10).
struct ThrowSettings {
    float slotRadiusFactor = 1.0f;  // height circle around the chest, x arm length
    float swingRadiusFactor = 1.0f; // throw circle around the shoulder, x arm length
    float maxBack = math::radians(120.0f);  // force point behind for full force
    float turnShare = 0.5f;      // share of the throw angle the trunk turns (the arm the rest)
    float pelvisShare = 0.4f;    // share of the trunk turn taken by the pelvis
    float readyTime = 0.25f;     // s, hand to the slot point
    float windTime = 0.35f;      // s, chest turns back to the force point
    float swingTime = 0.15f;     // s, force point -> release, speeding up
    float push = 0.15f;          // m, the hand pushes out along the throw by the release
    // In the swing the hand moves out from its place on the trunk onto the
    // throw circle, on it by this share of the way to the release.
    float blendShare = 0.6f;
    float followAngle = math::radians(60.0f);  // chest turn past the release
    float followTime = 0.35f;    // s, slowing down
    float untwistRate = 6.0f;    // 1/s, the chest turns back to square after the throw
    // Stride: the foot opposite the throwing hand steps toward the throw
    // when the force (wind-up share of maxBack) is at least strideForce, from
    // minStride to maxStride (m) at full force; the swing starts when it lands.
    float strideForce = 0.3f;
    float minStride = 0.3f;
    float maxStride = 0.9f;
    // Test: the swing stops at the release point, the ball still held,
    // until Human::letGo.
    bool pauseAtRelease = false;
    float minSpeed = 10.0f;      // m/s at release with no wind-up
    float maxSpeed = 31.0f;      // m/s at release from maxBack
};

// config/bowling.json (BIOMECHANICS 11).
struct BowlSettings {
    // Elbow bend kept through the whole circle (Law 21 allows at most 15 deg
    // of straightening, so the arm stays nearly straight; 11.1).
    float elbowBend = math::radians(10.0f);
    float startAngle = math::radians(135.0f);  // hand in front, low
    float setupTime = 0.3f;     // s, hand eases from where it is to the start
    float downBackTime = 0.5f;  // s, start -> horizontal behind (-90 deg), speeding up
    float overTime = 0.18f;     // s, horizontal behind -> over the top -> release
    float followTime = 0.4f;    // s, release -> stop, slowing down
    // Follow-through: share of the way the hand moves across the body, to the
    // mirror of its circle point (the other side), by the end.
    float followAcross = 0.6f;
    float speed = 22.0f;  // m/s at release, standing delivery (11.4)
    // Bowling action (M6): the hand goes to the action point in coilTime
    // (s); then action point -> release in swingTime (s), speeding up.
    float coilTime = 0.35f;
    // After overshooting the action point the hand is pulled back onto it
    // in pullTime (s).
    float pullTime = 0.15f;
    // The front hand starts frontDelay (s) after the bowling hand.
    float frontDelay = 0.1f;
    // Front hand's end kept apart from the bowling hand's way's end: this
    // share of the front shoulder -> end offset off on each axis.
    math::Vec3 frontGap{0.2f};
    // Yellow circle turned this far inward (radians): elbows fully outward-up.
    float elbowUpTurn = math::radians(45.0f);    // The whole-player turn counts as done within this of its goal (radians).
    float turnDone = math::radians(3.0f);
    float backTime = 0.3f;  // s, action point -> bottom point, speeding up
    // Chest turn (side-on, at the action point; chest bone first, the spine
    // taking what its limit leaves, the hips staying): chestBase always,
    // growing once the action point is less than coilAhead (m) in front of
    // the shoulder, maxCoil an arm length behind; held to the bottom point,
    // unwound by the release. Whole-player turn (the feet step round) with
    // the hand from the action point: bodyBase always, plus the release
    // circle's angle, full at the bottom point, back to straight by the
    // release. Values: hip-shoulder separation ~10-30 deg, shoulder
    // counter-rotation ~10-40 deg (above 30 linked to back injury), side-on
    // around back-foot contact (BIOMECHANICS 11.5).
    float coilAhead = 0.2f;
    float chestBase = math::radians(20.0f);
    float maxCoil = math::radians(35.0f);
    float bodyBase = math::radians(60.0f);
    float swingTime = 0.3f;
    // Release circle's part of the whole-player turn: the angle at the
    // shoulder between the bottom point and the release circle's point
    // furthest across the body (leanSign -1: the other way).
    float leanSign = 1.0f;
    // The front foot steps strideLength (m) straight ahead once the hand is
    // strideAt of the way from the action point down to the bottom point.
    float strideLength = 0.6f;
    float strideAt = 0.9f;
    float frontClear = 0.25f;    // m, front hand kept at least this far from the chest's vertical axis
    float releaseLean = 0.2f;   // m the release circle moves toward the batter by the release (the body leans after it)
    float minSpeed = 15.0f, maxSpeed = 26.0f;  // m/s at release: action point in front .. behind
    // Follow-through: the arm carries on followAngle round the circle over
    // swingFollowTime (s), slowing, and the hand is carried followReach (m)
    // forward, which the body folds after.
    float followAngle = math::radians(150.0f);
    float swingFollowTime = 0.45f;
    float followReach = 0.35f;
    // The bowling action runs at this share of real speed (1 = real; the I
    // key switches to slowMotion.factor in config/bowling.json).
    float slowMotion = 1.0f;
};

// Circle radius for an arm of `armLength` bent by `elbowBend` (radians),
// taking the upper arm and forearm as equal.
inline float bowlRadius(float armLength, float elbowBend) { return armLength * std::cos(0.5f * elbowBend); }

} // namespace motion
