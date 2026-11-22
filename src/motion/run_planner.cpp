#include "motion/run_planner.h"

#include "motion/foot_stepper.h"

#include <algorithm>
#include <cmath>

namespace motion {

void RunPlanner::times(float speed, const RunPlannerSettings& s, float& stance, float& flight) {
    stance = (s.stanceWay + s.stanceWayPerSpeed * speed) / std::max(speed, 0.1f);
    flight = std::max(0.0f, 0.5f * (s.swingTime - stance));
}

void RunPlanner::start(const RunBody& body, const math::Vec3 feet[2], math::Vec3 velocity) {
    stop();
    on_ = true;
    velocity.y = 0.0f;
    way_ = math::safeNormalize(velocity, math::kForward);
    speed_ = math::length(velocity);
    foot_ = math::dot(feet[0] - feet[1], way_) >= 0.0f ? 0 : 1;
    hips_ = body.pelvis;
    from_ = body.pelvis.y;
    groundY_ = feet[foot_].y - body.ankleUp;
}

void RunPlanner::rebase(const math::Transform& toNew) {
    if (!on_) return;
    for (Swing& w : swing_) {
        for (math::Vec3* p : {&w.from, &w.aim, &w.at}) *p = toNew.transformPoint(*p);
        w.normal = toNew.rotation * w.normal;
    }
    const float moved = toNew.transformPoint(math::Vec3(0.0f)).y;
    hips_ = toNew.transformPoint(hips_);
    way_ = toNew.rotation * way_;
    from_ += moved;
    groundY_ += moved;
}

math::Vec3 RunPlanner::landingSpot(int foot, float left, math::Vec3 velocity, const RunBody& body,
                                   const RunPlannerSettings& s) const {
    // Ahead of where the hips will be by the stance's share, a little in from
    // under its own hip.
    float stance, flight;
    times(speed_, s, stance, flight);
    const math::Vec3 left3(way_.z, 0.0f, -way_.x);  // to the left of the way
    const float across = math::dot(body.hip[foot] - body.pelvis, left3) * s.width;
    math::Vec3 spot = hips_ + velocity * left + way_ * (s.stanceAhead * speed_ * stance) + left3 * across;
    spot.y = groundY_ + body.ankleUp;
    return spot;
}

float RunPlanner::height(const RunBody& body, const RunPlannerSettings& s) {
    float stance, flight;
    times(speed_, s, stance, flight);
    // As high as the landing leg reaches its spot ahead.
    const float reach = std::min(body.legLength[0], body.legLength[1]) * s.legReach;
    const float ahead = std::min(s.stanceAhead * speed_ * stance, 0.9f * reach);
    const float hipDown = body.pelvis.y - 0.5f * (body.hip[0].y + body.hip[1].y);
    const float top = groundY_ + body.ankleUp + std::sqrt(reach * reach - ahead * ahead) + hipDown;
    const float g = body.gravity, fall = 0.5f * g * flight;  // m/s at touchdown and (up) at take-off
    float y;
    if (age_ < stance) {
        // The push: a half sine, over the stance the weight of it and a flight.
        const float most = 0.5f * math::kPi * g * (stance + flight) / stance, a = math::kPi / stance;
        load_ = g > 0.0f ? most / g * std::sin(a * age_) : 1.0f;
        y = -fall * age_ + most / a * (age_ - std::sin(a * age_) / a) - 0.5f * g * age_ * age_;
    } else {
        const float x = age_ - stance;
        load_ = 0.0f;
        y = fall * x - 0.5f * g * x * x;
    }
    const float k = blend_ * blend_ * (3.0f - 2.0f * blend_);
    load_ = math::lerp(1.0f, load_, k);
    return math::lerp(from_, top + y, k);
}

void RunPlanner::moveFoot(int i, const RunPlannerSettings& s, TargetSet& t) {
    Swing& w = swing_[i];
    Target& f = t.foot[i];
    f.contactNormal = w.normal;
    w.time += t.dt;
    if (w.time >= w.duration) {  // on its spot
        f.mode = TargetMode::Pinned;
        f.position = spot_[i] = w.aim;
        f.velocity = math::Vec3(0.0f);
        f.swing = -1.0f;
        landed_[i] = true;
        w.on = false;
        return;
    }
    const math::Vec3 before = w.at;
    const float u = std::clamp((w.time - w.heel) / std::max(w.duration - w.heel, 1e-3f), 0.0f, 1.0f);
    // The leg swings through at once (the hips are already far ahead of the
    // foot): an even push, then an even slowing.
    const float along = u * u * (3.0f - 2.0f * u);
    w.at = math::lerp(w.from, w.aim, along) + w.normal * (s.liftPerSpeed * speed_ * swingArc(u, s.peakAt));
    // The foot rolls over its ball; the swing carries the heel's rise away.
    const float r = std::clamp(w.time / std::max(w.heel, 1e-3f), 0.0f, 1.0f);
    const float pitch = s.toeOffPitch * r * r * (3.0f - 2.0f * r);
    w.at += (1.0f - along) * s.ballReach * (w.normal * std::sin(pitch) + way_ * (1.0f - std::cos(pitch)));
    f.mode = TargetMode::Driven;
    f.position = w.at;
    f.velocity = t.dt > 0.0f ? (w.at - before) / t.dt : math::Vec3(0.0f);
    f.swing = u;
    f.weightPosition = f.weightRotation = 1.0f;
}

void RunPlanner::pose(const RunBody& body, const RunPlannerSettings& s, TargetSet& t) const {
    const float k = poseWeight_ * poseWeight_ * (3.0f - 2.0f * poseWeight_);
    if (k <= 0.0f) return;
    const math::Vec3 left3(way_.z, 0.0f, -way_.x);
    // Each arm swings with the other side's leg: its foot ahead of the hips.
    const float half = std::max(0.5f * (s.stanceWay + s.stanceWayPerSpeed * speed_), 0.1f);
    for (int h = 0; h < 2; ++h) {
        Target& hand = t.hand[h];
        if (hand.mode != TargetMode::Free || body.upperArm <= 0.0f) continue;
        const float ahead = math::dot(t.foot[1 - h].position - body.pelvis, way_) / half;
        const float a = s.armMid + s.armSwing * std::clamp(ahead, -1.2f, 1.2f);  // the upper arm from hanging
        const float b = a + s.elbowBend;                                           // the forearm
        auto dir = [&](float x) { return way_ * std::sin(x) - math::kUp * std::cos(x); };
        hand.mode = TargetMode::Driven;
        hand.position = body.shoulder[h] + dir(a) * body.upperArm + dir(b) * body.forearm -
                        left3 * (h == 0 ? s.handIn : -s.handIn);
        hand.weightPosition = k;
        hand.weightRotation = 0.0f;
        hand.pathLeft = -1.0f;
    }
    if (t.chest.mode == TargetMode::Free) {
        t.chest.mode = TargetMode::Driven;
        t.chest.rotation = math::fromAxisAngle(left3, s.leanPerSpeed * speed_);
        t.chest.weightPosition = 0.0f;
        t.chest.weightRotation = k;
    }
}

void RunPlanner::tick(math::Vec3 velocity, const RunBody& body, const RunPlannerSettings& s, TargetSet& t) {
    landed_[0] = landed_[1] = false;
    if (!on_) return;
    velocity.y = 0.0f;
    const float goal = math::length(velocity);
    age_ += t.dt;
    blend_ = std::min(1.0f, blend_ + t.dt / std::max(s.blendTime, 1e-3f));
    hips_ += velocity * t.dt;

    for (int i = 0; i < 2; ++i) {
        if (!swing_[i].on) continue;
        moveFoot(i, s, t);
        if (!landed_[i]) continue;
        if (goal < s.exitSpeed) ending_ = true;
        if (ending_) continue;
        foot_ = i;
        age_ = 0.0f;
        speed_ = goal;
        way_ = math::safeNormalize(velocity, way_);
        groundY_ = spot_[i].y - body.ankleUp;
    }
    if (ending_ && !swing_[0].on && !swing_[1].on) {
        on_ = false;
        load_ = 1.0f;
        return;
    }

    float stance, flight;
    times(speed_, s, stance, flight);
    const int f = foot_, o = 1 - f;
    // A foot lifts (its heel first) and lands a stance and a flight after the
    // other one does.
    auto lift = [&](int i, float heel, float total) {
        Swing& w = swing_[i];
        w = Swing{};
        w.on = true;
        w.duration = std::max(total, 2.0f * t.dt);
        w.heel = std::min(heel, 0.9f * w.duration);
        w.from = w.aim = w.at = t.foot[i].position;
        w.normal = t.foot[i].contactNormal;
    };
    if (!ending_) {
        if (!swing_[o].on && age_ < stance) lift(o, 0.0f, stance + flight - age_);
        if (!swing_[f].on && !landed_[f] && age_ >= stance * (1.0f - s.heelShare))
            lift(f, std::max(0.0f, stance - age_), swing_[o].left() + stance + flight);
    }
    for (int i = 0; i < 2; ++i)
        if (swing_[i].on) swing_[i].aim = landingSpot(i, swing_[i].left(), velocity, body, s);

    poseWeight_ = std::clamp(poseWeight_ + (ending_ ? -1.0f : 1.0f) * t.dt / std::max(s.poseBlend, 1e-3f), 0.0f, 1.0f);
    pose(body, s, t);
    t.hips.mode = TargetMode::Driven;
    t.hips.velocity = velocity;
    t.hips.weightRotation = 0.0f;
    if (ending_) {
        // The height is the standing legs' again; level all of it still (the
        // decider's own place across is where its last step was).
        load_ = 1.0f;
        t.hips.position = hips_;
        t.hips.directions[0] = way_;
        t.hips.directions[1] = math::Vec3(way_.z, 0.0f, -way_.x);
        t.hips.directionCount = 2;
    } else {
        hips_.y = height(body, s);
        t.hips.position = hips_;
        t.hips.directionCount = 0;
    }
}

} // namespace motion
