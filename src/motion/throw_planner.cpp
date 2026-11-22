#include "motion/throw_planner.h"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {

constexpr bool kLeft = false;  // the throwing hand: the right one
constexpr int kHand = kLeft ? 0 : 1;
constexpr float kAhead = 0.15f;  // m, a hand this far ahead of the chest is clear of the body

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
    return makeSwingArc(slotArc(body, s), slot_, body.restShoulder, pulled(body, s), tilt_, s.swingRadius * body.armLength);
}

void ThrowPlanner::start(math::Vec3 goal, const ThrowArc& arc, const ThrowPlannerSettings& s) {
    from_ = at_;
    goal_ = goal;
    time_ = 0.0f;
    const float round = std::abs(arc.angleOf(goal) - arc.angleOf(at_)) * arc.radius;
    duration_ = std::max(math::fittsTime(std::max(round, math::distance(at_, goal)), s.fittsWidth, s.fittsA, s.fittsB), 1e-3f);
}

void ThrowPlanner::toSlot(float angle, const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (!on()) at_ = body.hand;
    slot_ = slotOf(angle, s);
    pull_ = 0.0f;
    const ThrowArc arc = slotArc(body, s);
    slotPoint_ = arc.point(slot_);
    start(slotPoint_, arc, s);
    stage_ = Stage::Slot;
}

void ThrowPlanner::pull(float by, const ThrowPlannerSettings& s) {
    if (pulling()) pull_ = std::clamp(pull_ + by, 0.0f, s.pullMax);
}

math::Vec3 ThrowPlanner::pulled(const ThrowBody& body, const ThrowPlannerSettings& s) const {
    const math::Vec3 back = -math::kForward, d = slotPoint_ - body.restShoulder;
    const float radius = std::max(s.pullReach * body.armLength, math::length(d));
    // Straight back to the arm's length: |d + x back| = radius.
    const float b = math::dot(d, back);
    const float straight = -b + std::sqrt(std::max(b * b - math::dot(d, d) + radius * radius, 0.0f));
    if (pull_ <= straight) return slotPoint_ + back * pull_;
    // Then round the shoulder, backward.
    const math::Vec3 out = math::safeNormalize(d + back * straight, back);
    const math::Vec3 axis = math::safeNormalize(math::cross(out, back), math::kUp);
    return body.restShoulder + math::fromAxisAngle(axis, (pull_ - straight) / std::max(radius, 1e-3f)) * out * radius;
}

void ThrowPlanner::hold() {
    if (pulling()) stage_ = Stage::Held;
}

void ThrowPlanner::swing(float angle, const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (stage_ != Stage::Held) return;
    const ThrowArc arc = swingArc(body, s);
    from_ = at_;
    begin_ = arc.angleOf(at_);
    release_ = std::max(releaseOf(angle, s), begin_);
    end_ = std::min(release_ + s.follow, std::max(release_, s.followMax));
    accel_ = math::lerp(s.accelMin, s.accelMax, force(s));
    const float way = (release_ - begin_) * arc.radius, past = (end_ - release_) * arc.radius;
    speed_ = std::sqrt(2.0f * accel_ * way);
    time_ = 0.0f;
    duration_ = speed_ > 1e-3f ? speed_ / accel_ + 2.0f * past / speed_ : 0.0f;
    goal_ = arc.point(end_);
    stage_ = Stage::Swing;
}

void ThrowPlanner::cancel(const ThrowBody& body, const ThrowPlannerSettings& s) {
    if (!on() || stage_ == Stage::Return) return;
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
    if (stage_ == Stage::Pull || stage_ == Stage::Held) {
        // After the mouse, no faster than pullSpeed.
        const math::Vec3 way = pulled(body, s) - at_;
        const float len = math::length(way), most = s.pullSpeed * t.dt;
        at_ += len > most ? way * (most / len) : way;
    } else if (stage_ == Stage::Swing) {
        // Round the throw circle: evenly faster to the release, then evenly
        // slower; out onto it (from the bent arm) by the release.
        ThrowArc arc = swingArc(body, s);
        const float toRelease = speed_ / accel_, past = duration_ - toRelease;
        const float after = std::max(0.0f, time_ - toRelease), before = std::min(time_, toRelease);
        const float way = 0.5f * accel_ * before * before + speed_ * after * (1.0f - 0.5f * after / std::max(past, 1e-3f));
        const float u = std::min(1.0f, before * before / std::max(toRelease * toRelease, 1e-6f));  // of the way to the release
        const float angle = begin_ + way / arc.radius;
        const math::Vec3 d = from_ - arc.centre;
        const float depth = math::dot(d, arc.normal);
        arc.radius = math::lerp(math::length(d - depth * arc.normal), arc.radius, u);
        at_ = arc.point(angle) + (1.0f - u) * depth * arc.normal;
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
}

} // namespace motion
