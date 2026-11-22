#pragma once
// How an arm gets its hand to a point within the shoulder's range (geometry
// only): where the elbow goes round the shoulder-hand line, and how far the
// hand stays off when the range does not let it get there. Character space.
// No allocations.

#include "ik/joint_limits.h"
#include "math/math.h"

namespace motion {

struct ArmShape {
    float upper = 0.0f, fore = 0.0f;  // m
    float reach = 0.0f, least = 0.0f; // m, the hand's furthest and nearest from the shoulder (the elbow's least and most bend)
    ik::JointLimit shoulder;          // the upper arm's range, in its parent's frame
};

// The arm as it hangs relaxed off the chest (unit, character space): its
// direction and where its elbow bends.
struct ArmHang {
    math::Vec3 down{0.0f, -1.0f, 0.0f}, bend{0.0f, 0.0f, -1.0f};
};

// The elbow's side with the arm turned from hanging to `dir` by the shortest
// swing, adding no twist.
math::Vec3 relaxedPole(const ArmHang& hang, math::Vec3 dir);

// The elbow's side to reach `target` from `shoulder` (the upper arm's parent
// turned `frame`): the relaxed one if the range lets it, else turned from it
// as little as does (the nearest to the target if none, unless `exact` is not
// asked for: then `miss` is only known not to be in the range). `miss`: how
// far (m) the hand stays off the target.
math::Vec3 fitPole(const ArmShape& arm, const ArmHang& hang, math::Quat frame, math::Vec3 shoulder, math::Vec3 target,
                   float& miss, bool exact = true);

// The least turn (rad, about `axis` through `pivot`, >= 0) of the body the arm
// hangs on that lets the hand get to `target` (no further off than its reach
// alone leaves it now); 0: it does already. If none does, the one that leaves
// it nearest. `hint`: the turn a moment ago (the search starts there).
float turnToReach(const ArmShape& arm, const ArmHang& hang, math::Quat frame, math::Vec3 shoulder, math::Vec3 target,
                  math::Vec3 pivot, math::Vec3 axis, float hint);

} // namespace motion
