#include "motion/arm_fit.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr float kClose = 1e-3f;  // m: there
constexpr int kBisections = 6;

// The hand's way off `target` with the elbow on `pole`'s side, the upper arm
// held in its range.
float missOn(const ArmShape& arm, math::Quat frame, math::Vec3 shoulder, math::Vec3 target, math::Vec3 dir, math::Vec3 pole) {
    const float d = std::clamp(math::distance(shoulder, target), arm.least, arm.reach);
    const math::Vec3 hand = shoulder + dir * d;
    const float x = (arm.upper * arm.upper - arm.fore * arm.fore + d * d) / (2.0f * std::max(d, 1e-4f));
    const float r = std::sqrt(std::max(arm.upper * arm.upper - x * x, 0.0f));
    const math::Vec3 u = math::safeNormalize(dir * x + pole * r, dir);
    // The upper arm's axes: -Y along it, +Z where the forearm bends to (the
    // elbow's hinge is X).
    const math::Vec3 f = hand - (shoulder + u * arm.upper);
    const math::Vec3 z = math::safeNormalize(f - u * math::dot(f, u), -pole);
    const math::Quat turned = math::lookRotation(z, -u);
    const math::Quat local = math::normalize(math::inverse(frame) * turned);
    const math::Quat held = ik::clampRotation(local, arm.shoulder);
    if (math::angleBetween(held, local) < 1e-4f) return math::distance(hand, target);
    const math::Vec3 at = shoulder + (frame * held) * (math::inverse(turned) * (hand - shoulder));
    return math::distance(at, target);
}

} // namespace

math::Vec3 relaxedPole(const ArmHang& hang, math::Vec3 dir) {
    const math::Quat swing = math::dot(hang.down, dir) < -0.9999f
                                 ? math::fromAxisAngle(math::safeNormalize(math::cross(hang.down, -hang.bend), math::kRight), math::kPi)
                                 : math::fromTo(hang.down, dir);
    return swing * hang.bend;
}

math::Vec3 fitPole(const ArmShape& arm, const ArmHang& hang, math::Quat frame, math::Vec3 shoulder, math::Vec3 target,
                   float& miss, bool exact) {
    const math::Vec3 dir = math::safeNormalize(target - shoulder, hang.down);
    const math::Vec3 relaxed = relaxedPole(hang, dir);
    const math::Vec3 pole = math::safeNormalize(relaxed - dir * math::dot(relaxed, dir), relaxed);
    const float d = math::distance(shoulder, target);
    const float most = std::max({0.0f, d - arm.reach, arm.least - d}) + kClose;  // in the range
    const auto at = [&](float angle) { return math::fromAxisAngle(dir, angle) * pole; };
    miss = missOn(arm, frame, shoulder, target, dir, pole);
    if (miss <= most) return relaxed;
    // Round both ways, outward from the relaxed side.
    constexpr int kSteps = 12;
    constexpr float kStep = math::kPi / kSteps;
    float best = miss, bestAngle = 0.0f;
    for (int k = 1; k <= kSteps; ++k)
        for (const float side : {1.0f, -1.0f}) {
            const float angle = side * kStep * static_cast<float>(k);
            const float m = missOn(arm, frame, shoulder, target, dir, at(angle));
            if (m < best) {
                best = m;
                bestAngle = angle;
            }
            if (m > most) continue;
            if (!exact) {
                miss = m;
                return at(angle);
            }
            float out = angle - side * kStep, in = angle;  // out of the range, in it
            for (int b = 0; b < kBisections; ++b) {
                const float middle = 0.5f * (out + in);
                if (missOn(arm, frame, shoulder, target, dir, at(middle)) > most) out = middle;
                else in = middle;
            }
            miss = missOn(arm, frame, shoulder, target, dir, at(in));
            return at(in);
        }
    // None: the nearest one, between the steps too.
    float width = kStep;
    for (int b = 0; b < kBisections && exact; ++b) {
        width *= 0.5f;
        for (const float angle : {bestAngle - width, bestAngle + width}) {
            const float m = missOn(arm, frame, shoulder, target, dir, at(angle));
            if (m < best) {
                best = m;
                bestAngle = angle;
            }
        }
    }
    miss = best;
    return at(bestAngle);
}

float turnToReach(const ArmShape& arm, const ArmHang& hang, math::Quat frame, math::Vec3 shoulder, math::Vec3 target,
                  math::Vec3 pivot, math::Vec3 axis, float hint) {
    const auto missAt = [&](float turn) {  // the body turned: the target turned back
        float m = 0.0f;
        fitPole(arm, hang, frame, shoulder, pivot + math::fromAxisAngle(axis, -turn) * (target - pivot), m, false);
        return m;
    };
    const float most = std::max(0.0f, math::distance(shoulder, target) - arm.reach) + kClose;
    const float now = missAt(0.0f);
    if (now <= most) return 0.0f;
    constexpr float kStep = math::kPi / 45.0f, kMost = math::kPi * 0.75f;
    float best = now, bestTurn = 0.0f;
    float in = std::clamp(hint, 0.0f, kMost), out = 0.0f;
    const float start = in > 0.0f ? missAt(in) : now;
    if (start <= most) {  // back toward none, to where it stops being enough
        out = std::max(0.0f, in - kStep);
        while (out > 0.0f && missAt(out) <= most) {
            in = out;
            out = std::max(0.0f, out - kStep);
        }
    } else {  // on, to where it is enough
        if (start < best) {
            best = start;
            bestTurn = in;
        }
        bool found = false;
        while (!found && in < kMost) {
            out = in;
            in += kStep;
            const float m = missAt(in);
            found = m <= most;
            if (!found && m < best - kClose) {
                best = m;
                bestTurn = in;
            }
        }
        if (!found) return bestTurn;
    }
    for (int b = 0; b < kBisections; ++b) {
        const float middle = 0.5f * (out + in);
        if (missAt(middle) > most) out = middle;
        else in = middle;
    }
    return in;
}

} // namespace motion
