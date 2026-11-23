#include "motion/throw_planner.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr bool kLeft = false;  // the throwing hand: the right one
constexpr int kHand = kLeft ? 0 : 1;
constexpr int kBack = kHand;                      // the back foot: the throwing side's
constexpr float kBackWay = kLeft ? 1.0f : -1.0f;  // the body turns back this way about the vertical
constexpr float kLeastSwing = 0.1f;  // rad, off the chest the hand goes at least this far to the release
constexpr float kAhead = 0.15f;  // m, a hand this far ahead of the chest is clear of the body

// A part's turn about the vertical (rad, + left).
float yawOf(math::Quat q) {
    const math::Vec3 ahead = q * math::kForward;
    return std::atan2(ahead.x, ahead.z);
}

// Round `arc` from `from` to `to` (never through the body): the angle, the
// distance from the centre and the depth, each by the same share.
math::Vec3 sweep(const ThrowArc& arc, math::Vec3 from, math::Vec3 to, float u) {
    const math::Vec3 a = from - arc.centre, b = to - arc.centre;
    const float depthA = math::dot(a, arc.normal), depthB = math::dot(b, arc.normal);
    ThrowArc now = arc;
    now.radius = math::lerp(math::length(a - depthA * arc.normal), math::length(b - depthB * arc.normal), u);
    return now.point(math::lerp(arc.angleOf(from), arc.angleOf(to), u)) + math::lerp(depthA, depthB, u) * arc.normal;
}

} // namespace

ThrowArc ThrowPlanner::slotArc(const ThrowBody& body, const ThrowPlannerSettings& s) const {
    return makeSlotArc(body.restChest, kLeft, s.slotRadius * body.armLength);
}

ThrowArc ThrowPlanner::swingArc(const ThrowBody& body, const ThrowPlannerSettings& s) const {
    return makeSwingArc(slotArc(body, s), slot_, body.restShoulder, pulled(body, s, pull_), tilt_, s.swingRadius * body.armLength);
}

void ThrowPlanner::start(math::Vec3 goal, const ThrowArc& arc, const ThrowPlannerSettings& s) {
    from_ = at_;
    goal_ = goal;
    time_ = 0.0f;
    const float round = std::abs(arc.angleOf(goal) - arc.angleOf(at_)) * arc.radius;
    duration_ = std::max(math::fittsTime(std::max(round, math::distance(at_, goal)), s.fittsWidth, s.fittsA, s.fittsB), 1e-3f);
}

void ThrowPlanner::toSlot(float angle, const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (stage_ == Stage::Uncoil) return;  // the hand comes forward first
    if (!on()) at_ = body.hand;
    slot_ = slotOf(angle, s);
    pull_ = drawn_ = 0.0f;
    released_ = false;
    const ThrowArc arc = slotArc(body, s);
    slotPoint_ = arc.point(slot_);
    start(slotPoint_, arc, s);
    stage_ = Stage::Slot;
}

void ThrowPlanner::pull(float by, const ThrowPlannerSettings& s) {
    if (pulling()) pull_ = std::clamp(pull_ + by, 0.0f, s.pullMax);
}

math::Vec3 ThrowPlanner::pulled(const ThrowBody& body, const ThrowPlannerSettings& s, float pull) const {
    if (pull > s.coilFrom) {
        // Coiled: from there it rides the body round.
        const math::Vec3 from = pulled(body, s, s.coilFrom) - body.restChest;
        return body.restChest + math::fromAxisAngle(math::kUp, kBackWay * s.coilHand * coilOf(pull, s)) * from;
    }
    const math::Vec3 straightArm = pulledStraight(body, s, pull);
    // Raised: the elbow bends and the forearm lays back about the upper arm
    // (the shoulder's external rotation, more the higher the arm).
    const math::Vec3 u = math::safeNormalize(slotPoint_ - body.restShoulder, math::kUp);
    const float e = std::acos(std::clamp(-u.y, -1.0f, 1.0f));  // the arm from hanging
    const float w = std::clamp((e - s.bentFrom) / std::max(s.bentAt - s.bentFrom, 1e-3f), 0.0f, 1.0f);
    if (w <= 0.0f) return straightArm;
    const float half = 0.5f * math::kPi * 0.5f;  // 45 deg: the table's step
    const float er = e < half ? math::lerp(s.layback[0], s.layback[1], e / half)
                              : math::lerp(s.layback[1], s.layback[2], std::min((e - half) / half, 1.0f));
    const float phi = er - 0.5f * math::kPi;  // past the forearm upright
    const math::Vec3 side = makeSlotArc(body.restChest, kLeft, 1.0f).side;
    math::Vec3 t = math::kUp - u * math::dot(u, math::kUp);
    t = math::length(t) > 0.2f ? math::normalize(t) : -side;
    const math::Vec3 back = -math::kForward;
    const math::Vec3 b = math::safeNormalize(back - u * math::dot(u, back) - t * math::dot(t, back), back);
    const math::Vec3 fEnd = std::cos(phi) * t + std::sin(phi) * b;
    const float k = std::clamp(pull / std::max(s.coilFrom, 1e-3f), 0.0f, 1.0f);
    const float ang = std::acos(std::clamp(math::dot(u, fEnd), -1.0f, 1.0f));
    const math::Vec3 axis = math::safeNormalize(math::cross(u, fEnd), b);
    const math::Vec3 elbow = body.restShoulder + u * body.upperArm;
    const math::Vec3 laid = elbow + math::fromAxisAngle(axis, k * ang) * u * (body.armLength - body.upperArm);
    return math::lerp(straightArm, laid, w);
}

math::Vec3 ThrowPlanner::pulledStraight(const ThrowBody& body, const ThrowPlannerSettings& s, float pull) const {
    const math::Vec3 back = -math::kForward, d = slotPoint_ - body.restShoulder;
    const float radius = std::max(s.pullReach * body.armLength, math::length(d));
    // Straight back to the arm's length: |d + x back| = radius.
    const float b = math::dot(d, back);
    const float straight = -b + std::sqrt(std::max(b * b - math::dot(d, d) + radius * radius, 0.0f));
    if (pull <= straight) return slotPoint_ + back * pull;
    // Then round the shoulder, backward.
    const math::Vec3 out = math::safeNormalize(d + back * straight, back);
    const math::Vec3 axis = math::safeNormalize(math::cross(out, back), math::kUp);
    return body.restShoulder + math::fromAxisAngle(axis, (pull - straight) / std::max(radius, 1e-3f)) * out * radius;
}

void ThrowPlanner::hold() {
    if (pulling()) stage_ = Stage::Held;
}

void ThrowPlanner::swing(float angle, const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (stage_ != Stage::Held) return;
    release_ = releaseOf(angle, s);
    released_ = true;
    if (body.standing) startSwing(body, s);
}

void ThrowPlanner::startSwing(const ThrowBody& body, const ThrowPlannerSettings& s) {
    released_ = false;
    toRelease_ = std::max(math::lerp(s.timeNone, s.timeFull, force(s)), 0.05f);
    chestFrom_ = yawOf(body.chest.rotation);
    hipsFrom_ = yawOf(body.pelvis.rotation);
    loadFrom_ = math::lerp(0.5f, s.backLoad, coilOf(drawn_, s));
    past_ = (kLeft ? -1.0f : 1.0f) * s.chestPast * force(s);
    riding_ = math::inverse(body.chest.rotation) * (at_ - body.chest.position);
    off_ = false;
    lastAngle_ = swingArc(body, s).angleOf(at_);
    rate_ = 0.0f;
    time_ = 0.0f;
    duration_ = 1e9f;  // known once the hand is off the chest
    stage_ = Stage::Swing;
}

math::Vec3 ThrowPlanner::swung(const ThrowBody& body, const ThrowPlannerSettings& s, float dt) {
    ThrowArc arc = swingArc(body, s);
    if (time_ < s.ride * toRelease_) {
        const math::Vec3 p = body.chest.position + body.chest.rotation * riding_;
        const float a = arc.angleOf(p);
        if (dt > 0.0f) rate_ = std::remainder(a - lastAngle_, math::kTwoPi) / dt;
        lastAngle_ = a;
        return p;
    }
    if (!off_) {
        // Off the chest, from where it is as it goes: evenly faster to be at the
        // release on time, out onto the circle meanwhile.
        off_ = true;
        offAt_ = time_;
        const math::Vec3 d = at_ - arc.centre;
        depth_ = math::dot(d, arc.normal);
        radius_ = math::length(d - depth_ * arc.normal);
        angle_ = lastAngle_;
        const float left = std::max(toRelease_ - offAt_, 1e-2f);
        release_ = std::max(release_, angle_ + kLeastSwing);
        rate_ = std::clamp(rate_, 0.0f, (release_ - angle_) / left);
        accel_ = 2.0f * (release_ - angle_ - rate_ * left) / (left * left);
        end_ = std::max(release_, std::min(release_ + s.follow, s.followMax));
        const float fast = rate_ + accel_ * left;
        duration_ = toRelease_ + (fast > 1e-3f ? 2.0f * (end_ - release_) / fast : 0.0f);
    }
    const float left = std::max(toRelease_ - offAt_, 1e-2f);
    const float t = time_ - offAt_, before = std::min(t, left), after = std::max(0.0f, t - left);
    const float fast = rate_ + accel_ * left, past = std::max(duration_ - toRelease_, 1e-3f);
    const float angle = angle_ + rate_ * before + 0.5f * accel_ * before * before + fast * after * (1.0f - 0.5f * after / past);
    const float out = math::minimumJerk(std::clamp(t / (left * s.elbowBy), 0.0f, 1.0f));
    arc.radius = math::lerp(radius_, arc.radius, out);
    return arc.point(angle) + (1.0f - out) * depth_ * arc.normal;
}

void ThrowPlanner::cancel(const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (!on() || stage_ == Stage::Uncoil || stage_ == Stage::Return) return;
    released_ = false;
    if (drawn_ > s.coilFrom && (stage_ != Stage::Swing || !off_)) {
        // Round the body as it came.
        stage_ = Stage::Uncoil;
        pull_ = s.coilFrom;
        return;
    }
    drawn_ = 0.0f;
    start(body.restHand, slotArc(body, s), s);
    stage_ = Stage::Return;
}

void ThrowPlanner::rebase(const math::Transform& toNew) {
    for (math::Vec3* p : {&at_, &from_, &goal_, &slotPoint_}) *p = toNew.transformPoint(*p);
}

void ThrowPlanner::tick(const ThrowBody& body, const ThrowPlannerSettings& s, TargetSet& t) {
    if (!on()) return;
    time_ += t.dt;
    if (stage_ == Stage::Return && time_ >= duration_) {
        stop();
        return;
    }
    if (stage_ == Stage::Slot && time_ >= duration_) stage_ = Stage::Pull;
    if (stage_ == Stage::Swing && time_ >= duration_) cancel(body, s);
    if (stage_ == Stage::Uncoil && drawn_ <= s.coilFrom) {
        start(body.restHand, slotArc(body, s), s);
        stage_ = Stage::Return;
        drawn_ = 0.0f;
    }
    if (stage_ == Stage::Pull || stage_ == Stage::Held || stage_ == Stage::Uncoil) {
        // After the mouse, no faster than pullSpeed.
        const float most = s.pullSpeed * t.dt;
        drawn_ += std::clamp(pull_ - drawn_, -most, most);
        const math::Vec3 way = pulled(body, s, drawn_) - at_;
        const float len = math::length(way);
        at_ += len > most ? way * (most / len) : way;
    } else if (stage_ == Stage::Swing) {
        at_ = swung(body, s, t.dt);
    } else {
        const float u = math::minimumJerk(std::clamp(time_ / duration_, 0.0f, 1.0f));
        const bool ahead = stage_ == Stage::Return && from_.z - body.restChest.z > kAhead;
        at_ = u >= 1.0f ? goal_
              : ahead   ? math::lerp(from_, goal_, u)  // in front of the body: straight down
                        : sweep(slotArc(body, s), from_, goal_, u);
    }
    Target& hand = t.hand[kHand];
    hand.mode = TargetMode::Driven;
    hand.position = at_;
    hand.weightRotation = 0.0f;  // the wrist straight on the forearm
    // Its path (as a mover's): where it goes and in what time, so the body
    // plans ahead for it (the decider's reach: the trunk, the hips, a step).
    if (stage_ == Stage::Pull || stage_ == Stage::Held || stage_ == Stage::Uncoil) {
        hand.pathEnd = pulled(body, s, pull_);
        hand.pathLeft = std::abs(pull_ - drawn_) / std::max(s.pullSpeed, 1e-3f);
    } else if (stage_ != Stage::Swing) {
        hand.pathEnd = goal_;
        hand.pathLeft = std::max(0.0f, duration_ - time_);
    }
    if (stage_ == Stage::Held && released_ && body.standing) startSwing(body, s);
    const float coil = stage_ == Stage::Pull || stage_ == Stage::Held || stage_ == Stage::Uncoil ? coilOf(drawn_, s) : 0.0f;
    if (coil > 0.0f) {
        // The weight onto the back foot (the COM over the drive leg); the
        // body turns by itself where the arm cannot (the decider).
        t.foot[kBack].load = math::lerp(0.5f, s.backLoad, coil);
    }
    if (stage_ == Stage::Swing) {
        // The hips first, then the chest, from where the pull left them; the
        // weight back to even with the hips.
        const auto share = [&](float by) {
            return math::minimumJerk(std::clamp(time_ / std::max(by * toRelease_, 1e-3f), 0.0f, 1.0f));
        };
        const auto turned = [&](float from, float to, float by) {
            return math::fromAxisAngle(math::kUp, math::lerp(from, to, share(by)));
        };
        t.foot[kBack].load = math::lerp(loadFrom_, 0.5f, share(s.hipsBy));
        t.hips.mode = TargetMode::Driven;
        t.hips.weightPosition = 0.0f;
        t.hips.rotation = turned(hipsFrom_, 0.0f, s.hipsBy);
        t.chest.mode = TargetMode::Driven;
        t.chest.weightPosition = 0.0f;
        t.chest.rotation = turned(chestFrom_, past_, s.chestBy);
    }
}

} // namespace motion
