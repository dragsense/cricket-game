#include "agents/balancer.h"

#include "core/config.h"
#include "core/log.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace agents {

namespace {

float cross2(math::Vec2 o, math::Vec2 a, math::Vec2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

math::Vec2 flat(math::Vec3 v) { return math::Vec2(v.x, v.z); }

// The part's sole: the four corners of its box's face most toward the ground
// (world), written to `out`; 0 if the part is not a box.
int soleCorners(const physics::PhysicsWorld& world, const HumanBody::Part& p, math::Vec3* out) {
    if (p.shapeType != physics::ShapeType::Box) return 0;
    const math::Transform t = world.transform(p.body);
    const math::Vec3 axes[3] = {t.rotation * math::Vec3(1, 0, 0), t.rotation * math::Vec3(0, 1, 0),
                                t.rotation * math::Vec3(0, 0, 1)};
    int down = 0;
    for (int k = 1; k < 3; ++k)
        if (std::fabs(axes[k].y) > std::fabs(axes[down].y)) down = k;
    const int u = (down + 1) % 3, v = (down + 2) % 3;
    const math::Vec3 base = t.position + axes[down] * (axes[down].y < 0.0f ? 1.0f : -1.0f) * p.halfExtents[down];
    int n = 0;
    for (float su : {-1.0f, 1.0f})
        for (float sv : {-1.0f, 1.0f})
            out[n++] = base + su * p.halfExtents[u] * axes[u] + sv * p.halfExtents[v] * axes[v];
    return n;
}

math::Vec3 jointOf(const physics::PhysicsWorld& world, const HumanBody::Part& p) {
    return (world.transform(p.body) * math::inverse(p.shape)).position;
}

constexpr float kTwoPi = 6.2831853f;

} // namespace

BalanceSettings readBalanceSettings(const core::Config& c) {
    BalanceSettings s;
    auto f = [&](const char* key, float& v) { v = c.getFloat(std::string("balance.") + key, v); };
    f("hoverShare", s.hoverShare);
    f("hoverMaxShare", s.hoverMaxShare);
    f("hoverFrequency", s.hoverFrequency);
    f("hoverDamping", s.hoverDamping);
    f("uprightFrequency", s.uprightFrequency);
    f("uprightDamping", s.uprightDamping);
    f("uprightMaxTorque", s.uprightMaxTorque);
    f("pullFrequency", s.pullFrequency);
    f("pullDamping", s.pullDamping);
    f("pullMaxForce", s.pullMaxForce);
    f("footFriction", s.footFriction);
    f("footRestitution", s.footRestitution);
    f("lockMinStrength", s.lockMinStrength);
    f("shiftTime", s.shiftTime);
    f("stepMargin", s.stepMargin);
    f("stepMaxLength", s.stepMaxLength);
    f("stepMinWidth", s.stepMinWidth);
    f("stepClearance", s.stepClearance);
    f("recoverRate", s.recoverRate);
    return s;
}

void Balancer::Hull::build(const math::Vec3* points, int n) {
    // Andrew's monotone chain on (x, z); counter-clockwise, no repeated or
    // collinear corners. At most kHullPoints - 1 points (no allocation).
    count = 0;
    n = std::min(n, kHullPoints - 1);
    if (n <= 0) return;
    math::Vec2 p[kHullPoints];
    for (int k = 0; k < n; ++k) p[k] = flat(points[k]);
    std::sort(p, p + n, [](math::Vec2 a, math::Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    n = static_cast<int>(std::unique(p, p + n) - p);
    if (n == 1) {
        corner[0] = p[0];
        count = 1;
        return;
    }
    math::Vec2 chain[2 * kHullPoints];
    int m = 0;
    for (int k = 0; k < n; ++k) {  // lower chain
        while (m >= 2 && cross2(chain[m - 2], chain[m - 1], p[k]) <= 0.0f) --m;
        chain[m++] = p[k];
    }
    for (int k = n - 2, lower = m + 1; k >= 0; --k) {  // upper chain
        while (m >= lower && cross2(chain[m - 2], chain[m - 1], p[k]) <= 0.0f) --m;
        chain[m++] = p[k];
    }
    count = std::min(m - 1, kHullPoints);  // the last point repeats the first
    std::copy(chain, chain + count, corner);
}

bool Balancer::Hull::contains(math::Vec2 q) const {
    if (count < 3) return false;
    for (int k = 0; k < count; ++k)  // counter-clockwise: inside is left of every edge
        if (cross2(corner[k], corner[(k + 1) % count], q) < 0.0f) return false;
    return true;
}

bool Balancer::init(physics::PhysicsWorld& world, const HumanBody& body, const int foot[2], const int toe[2],
                    const BalanceSettings& s) {
    settings_ = s;
    const std::vector<HumanBody::Part>& parts = body.parts();
    if (parts.empty()) return false;
    for (int i = 0; i < 2; ++i) {
        footPart_[i] = body.partOf(foot[i]);
        toePart_[i] = body.partOf(toe[i]);
        if (footPart_[i] < 0 || parts[static_cast<size_t>(footPart_[i])].joint != foot[i]) {
            LOG_ERROR("balancer: foot joint {} has no part of its own", foot[i]);
            return false;
        }
        if (toePart_[i] >= 0 && parts[static_cast<size_t>(toePart_[i])].joint != toe[i]) toePart_[i] = -1;
        for (int part : {footPart_[i], toePart_[i]})
            if (part >= 0) world.setMaterial(parts[static_cast<size_t>(part)].body, s.footFriction, s.footRestitution);
        // The lock: a hidden kinematic anchor on the foot, joined with every
        // axis fixed, off until the foot plants (then the anchor is moved onto
        // the foot, so the joint holds it where it is).
        const physics::BodyId f = parts[static_cast<size_t>(footPart_[i])].body;
        physics::BodyDesc d;
        d.shape = physics::ShapeType::Sphere;
        d.radius = 0.01f;
        d.mass = 1.0f;
        d.kinematic = true;
        d.collides = false;
        d.pose = world.transform(f);
        anchor_[i] = world.createBody(d);
        if (anchor_[i] == physics::kInvalidBody) return false;
        physics::SixDofDesc j;
        j.pivot = d.pose.position;
        lock_[i] = world.createSixDof(anchor_[i], f, j);  // min = max = 0: every axis fixed
        if (lock_[i] == physics::kInvalidJoint) return false;
        world.setJointEnabled(lock_[i], false);
        locked_[i] = false;
    }
    free_.assign(parts.size(), 0);
    const math::Vec3 hips = world.transform(parts[0].body).position;
    standHeight_ = hips.y - world.groundHeightAt(hips);
    strength_ = 1.0f;
    return true;
}

void Balancer::release(physics::PhysicsWorld& world) {
    for (int i = 0; i < 2; ++i) {
        if (lock_[i] != physics::kInvalidJoint) world.setJointEnabled(lock_[i], false);
        locked_[i] = false;
        lifting_[i] = false;
        shiftTimer_[i] = 0.0f;
    }
    std::fill(free_.begin(), free_.end(), uint8_t{0});
    stepFoot_ = -1;
    hover_ = pull_ = upright_ = math::Vec3(0.0f);
}

void Balancer::knock(float amount) { strength_ = std::clamp(strength_ - amount, 0.0f, 1.0f); }

void Balancer::plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2], float dt) {
    strength_ = std::min(1.0f, strength_ + settings_.recoverRate * dt);
    stanceCount_ = 0;
    stepFoot_ = -1;
    const std::vector<HumanBody::Part>& parts = body.parts();

    // 1. The support: the soles of every touching foot part (a touching part
    // can press anywhere on its sole); the toes only at the ball's edge (they
    // carry little). A lifting foot carries nothing.
    math::Vec3 ankle[2];
    for (int i = 0; i < 2; ++i) {
        down_[i] = down[i];
        if (down[i]) {
            lifting_[i] = false;
            shiftTimer_[i] = 0.0f;
        }
        footPoints_[i] = 0;
        int contacts = 0;
        for (int part : {footPart_[i], toePart_[i]}) {
            if (part < 0 || lifting_[i]) continue;
            const HumanBody::Part& p = parts[static_cast<size_t>(part)];
            math::Vec3 touch[kPointsPerFoot];
            const int n = world.groundContacts(p.body, touch, kPointsPerFoot);
            if (n == 0) continue;
            contacts += n;
            math::Vec3 corners[4];
            const int sole = footPoints_[i] + 4 <= kPointsPerFoot ? soleCorners(world, p, corners) : 0;
            if (sole > 0 && part == toePart_[i]) {
                const math::Vec3 ball = jointOf(world, p);
                std::sort(corners, corners + 4, [&](math::Vec3 a, math::Vec3 b) {
                    return math::distance(flat(a), flat(ball)) < math::distance(flat(b), flat(ball));
                });
                for (int k = 0; k < 2; ++k) footPoint_[i][footPoints_[i]++] = corners[k];
                continue;
            }
            if (sole > 0) {
                for (int k = 0; k < sole; ++k) footPoint_[i][footPoints_[i]++] = corners[k];
                continue;
            }
            for (int k = 0; k < n && footPoints_[i] < kPointsPerFoot; ++k) footPoint_[i][footPoints_[i]++] = touch[k];
        }
        stance_[i] = contacts > 0;
        free_[static_cast<size_t>(footPart_[i])] = stance_[i] && down[i] && strength_ >= settings_.lockMinStrength;
        footHull_[i].build(footPoint_[i], footPoints_[i]);
        ankle[i] = jointOf(world, parts[static_cast<size_t>(footPart_[i])]);
        if (stance_[i]) ++stanceCount_;
    }

    const math::Vec3 com = body.centreOfMass(world);
    const math::Vec3 vel = body.centreOfMassVelocity(world);
    const float ground = world.groundHeightAt(com);
    const float g = std::max(-world.world().gravity.y, 0.0f);
    const float omega0 = std::sqrt(g / std::max(com.y - ground, 0.1f));
    capture_ = math::Vec3(com.x + vel.x / omega0, ground, com.z + vel.z / omega0);

    // Lifts: a foot meant to lift waits for the weight over the other (its
    // centre of mass inside that foot), at most shiftTime.
    for (int i = 0; i < 2; ++i) {
        if (down[i] || lifting_[i]) continue;
        const int o = 1 - i;
        shiftTimer_[i] += dt;
        if (!stance_[i] || !stance_[o] || footHull_[o].contains(flat(com)) || shiftTimer_[i] >= settings_.shiftTime)
            lifting_[i] = true;
    }
    for (int i = 0; i < 2; ++i) liftReady_[i] = lifting_[i] || !stance_[i];
    lost_ = stanceCount_ == 0;
    if (stanceCount_ == 0) return;

    // The aim: over the stance ankles (where a standing body rests: the
    // centre of mass a few cm ahead of the ankle, the soles reaching on to the
    // toes); over the other ankle while one foot is meant to lift.
    const int unloading = !down[0] && down[1] && stance_[1] ? 0 : !down[1] && down[0] && stance_[0] ? 1 : -1;
    math::Vec3 all[2 * kPointsPerFoot], sum(0.0f);
    int n = 0, aimFeet = 0;
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        for (int k = 0; k < footPoints_[i]; ++k) all[n++] = footPoint_[i][k];
        if (i == unloading) continue;
        sum += ankle[i];
        ++aimFeet;
    }
    const math::Vec3 centre = aimFeet > 0 ? sum / static_cast<float>(aimFeet) : com;
    aim_ = math::Vec3(centre.x, ground, centre.z);
    Hull support;
    support.build(all, n);
    lost_ = !support.contains(flat(capture_));

    // 6. A step: on two feet with the capture point outside the support, the
    // foot whose step is shorter goes past it, to its own side of the other.
    if (!lost_ || stanceCount_ != 2 || !down[0] || !down[1]) return;
    float best = 1e30f;
    for (int i = 0; i < 2; ++i) {
        const int o = 1 - i;
        math::Vec2 dir = flat(capture_) - flat(aim_);
        const float len = math::length(dir);
        dir = len > 1e-4f ? dir / len : math::Vec2(0.0f);
        math::Vec2 t = flat(capture_) + settings_.stepMargin * dir;
        const math::Vec2 stance = flat(ankle[o]);
        math::Vec2 side = flat(ankle[i]) - stance;
        const float sideLen = math::length(side);
        side = sideLen > 1e-4f ? side / sideLen : math::Vec2(0.0f);
        const float out = math::dot(t - stance, side);
        if (out < settings_.stepMinWidth) t += (settings_.stepMinWidth - out) * side;  // no crossing
        const math::Vec2 reach = t - stance;
        const float r = math::length(reach);
        if (r > settings_.stepMaxLength) t = stance + reach * (settings_.stepMaxLength / r);
        const float travel = math::distance(t, flat(ankle[i]));
        if (travel < best) {
            best = travel;
            stepFoot_ = i;
            stepTarget_ = math::Vec3(t.x, ankle[i].y, t.y);
        }
    }
}

void Balancer::apply(physics::PhysicsWorld& world, const HumanBody& body) {
    hover_ = pull_ = upright_ = math::Vec3(0.0f);
    const std::vector<HumanBody::Part>& parts = body.parts();

    // 5. The feet's locks: a carrying foot meant down is held while the
    // strength allows; a lifting one, or a weak body, lets go.
    for (int i = 0; i < 2; ++i) {
        const bool hold = stance_[i] && down_[i] && strength_ >= settings_.lockMinStrength;
        if (hold && !locked_[i]) {
            world.resetBody(anchor_[i], world.transform(parts[static_cast<size_t>(footPart_[i])].body));
            world.setJointEnabled(lock_[i], true);
        } else if (!hold && locked_[i]) {
            world.setJointEnabled(lock_[i], false);
        }
        locked_[i] = hold;
        anchorAt_[i] = world.transform(anchor_[i]).position;
    }
    if (stanceCount_ == 0 || strength_ <= 0.0f) return;  // in the air or knocked out: nothing holds it

    const float mass = body.totalMass();
    const float g = std::max(-world.world().gravity.y, 0.0f);
    const math::Vec3 com = body.centreOfMass(world);
    const math::Vec3 vel = body.centreOfMassVelocity(world);
    const HumanBody::Part& pelvis = parts[0];
    const math::Transform hipsNow = world.transform(pelvis.body);

    // 2. Hover: a share of the weight, plus a spring on the hips' height.
    {
        const float w = kTwoPi * settings_.hoverFrequency;
        const float height = hipsNow.position.y - world.groundHeightAt(hipsNow.position);
        const float up = mass * (settings_.hoverShare * g + w * w * (standHeight_ - height) -
                                 2.0f * settings_.hoverDamping * w * world.linearVelocity(pelvis.body).y);
        const float most = settings_.hoverMaxShare * mass * g;
        hover_ = math::Vec3(0.0f, std::clamp(up, 0.0f, most) * strength_, 0.0f);
    }
    // 4. Pull: the centre of mass over the aim, across only.
    {
        const float w = kTwoPi * settings_.pullFrequency;
        math::Vec3 f = mass * (w * w * (aim_ - com) - 2.0f * settings_.pullDamping * w * vel);
        f.y = 0.0f;
        const float size = math::length(f), most = settings_.pullMaxForce * strength_;
        pull_ = size > most && size > 0.0f ? f * (most / size) : f * strength_;
    }
    world.addForce(pelvis.body, hover_ + pull_);

    // 3. Upright: the pelvis's up toward the plan's pelvis up, by the body's
    // inertia about its centre of mass; the turn about up is left free.
    {
        float inertia = 0.0f;
        for (const HumanBody::Part& p : parts) {
            const math::Vec3 d = world.transform(p.body).position - com;
            inertia += p.mass * math::dot(d, d);
        }
        const math::Vec3 boneUp = (hipsNow.rotation * math::inverse(pelvis.shape.rotation)) * math::kUp;
        const math::Vec3 want = math::normalize(pelvisTarget_) * math::kUp;
        const math::Vec3 axis = math::cross(boneUp, want);
        const float s = math::length(axis);
        const float angle = std::atan2(s, math::dot(boneUp, want));
        const math::Vec3 err = s > 1e-6f ? axis * (angle / s) : math::Vec3(0.0f);
        math::Vec3 spin = world.angularVelocity(pelvis.body);
        spin -= math::kUp * math::dot(spin, math::kUp);  // the turn about up is left free
        const float w = kTwoPi * settings_.uprightFrequency;
        math::Vec3 t = inertia * (w * w * err - 2.0f * settings_.uprightDamping * w * spin);
        const float size = math::length(t), most = settings_.uprightMaxTorque * strength_;
        upright_ = size > most && size > 0.0f ? t * (most / size) : t * strength_;
        world.addTorque(pelvis.body, upright_);
    }
}

} // namespace agents
