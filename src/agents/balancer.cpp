#include "agents/balancer.h"

#include "core/config.h"
#include "core/log.h"

#include <algorithm>
#include <cmath>

namespace agents {

namespace {

// The joint-frame axis (0 x, 1 y, 2 z) of a SixDOF part most along `dir` at
// rest (the bone frame then has the character's axes); -1 for a hinge.
int axisAlong(const HumanBody::Part& p, math::Vec3 dir) {
    if (p.hinge) return -1;
    const float a[3] = {std::fabs(math::dot(p.frameX, dir)), std::fabs(math::dot(p.frameY, dir)),
                        std::fabs(math::dot(p.frameZ, dir))};
    return a[0] >= a[1] && a[0] >= a[2] ? 0 : a[1] >= a[2] ? 1 : 2;
}

// Jolt treats a spring of zero stiffness as a rigid lock: a passive joint
// axis gets this (N m/rad, nothing next to the body's weight) with its damping.
constexpr float kNoStiffness = 1e-3f;

math::Vec3 frameAxis(const HumanBody::Part& p, int axis) {
    return axis == 0 ? p.frameX : axis == 1 ? p.frameY : p.frameZ;
}

// One joint axis as a spring (Jolt's stiffness-and-damping motor, solved
// with the contacts) that gives `torque` (world, on the part, from its
// parent) along that axis now: its aim set `torque / stiffness` from where
// the joint is. Stiffness kNoStiffness and no torque: only the joint's own
// (passive) damping holds it. Never harder than the joint's muscle.
void axisSpring(physics::PhysicsWorld& world, const HumanBody::Part& p, int axis, math::Vec3 torque, float stiffness,
                float damping) {
    const math::Quat bone = world.transform(p.body).rotation * math::inverse(p.shape.rotation);
    const math::Vec3 along = bone * (p.hinge ? p.frameX : frameAxis(p, axis));
    const float offset = stiffness > kNoStiffness ? math::dot(torque, along) / stiffness : 0.0f;
    if (p.hinge) {
        world.setHingeSpring(p.link, offset, stiffness, damping, p.strength);
        return;
    }
    if (axis >= 0) world.setSixDofAxisSpring(p.link, axis, offset, stiffness, damping, p.strength);
}

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

} // namespace

BalanceSettings readBalanceSettings(const core::Config& joints) {
    BalanceSettings s;
    const float perDeg = math::degrees(1.0f);  // N m/deg -> N m/rad
    s.ankleStiffness = joints.getFloat("balance.ankleStiffness", s.ankleStiffness / perDeg) * perDeg;
    s.ankleDamping = joints.getFloat("balance.ankleDamping", s.ankleDamping / perDeg) * perDeg;
    s.passiveDamping = joints.getFloat("balance.passiveDamping", s.passiveDamping / perDeg) * perDeg;
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

math::Vec2 Balancer::Hull::closest(math::Vec2 q) const {
    if (count == 0) return q;
    if (count == 1) return corner[0];
    if (contains(q)) return q;
    math::Vec2 best = corner[0];
    float bestDist = 1e30f;
    for (int k = 0; k < count; ++k) {
        const math::Vec2 a = corner[k], b = corner[(k + 1) % count];
        const math::Vec2 ab = b - a;
        const float len2 = math::dot(ab, ab);
        const float t = len2 > 0.0f ? std::clamp(math::dot(q - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
        const math::Vec2 c = a + t * ab;
        const float d = math::dot(q - c, q - c);
        if (d < bestDist) {
            bestDist = d;
            best = c;
        }
    }
    return best;
}

bool Balancer::init(const HumanBody& body, const int foot[2], const int toe[2], const BalanceSettings& s) {
    settings_ = s;
    for (int i = 0; i < 2; ++i) {
        footPart_[i] = body.partOf(foot[i]);
        toePart_[i] = body.partOf(toe[i]);
        if (footPart_[i] < 0 || body.parts()[static_cast<size_t>(footPart_[i])].joint != foot[i] ||
            body.parts()[static_cast<size_t>(footPart_[i])].parent < 0) {
            LOG_ERROR("balancer: foot joint {} has no part of its own", foot[i]);
            return false;
        }
        // The shin and the thigh: the two parts up from the foot (their
        // joints: knee, hip).
        shinPart_[i] = body.parts()[static_cast<size_t>(footPart_[i])].parent;
        thighPart_[i] = body.parts()[static_cast<size_t>(shinPart_[i])].parent;
        if (thighPart_[i] < 0 || body.parts()[static_cast<size_t>(thighPart_[i])].parent < 0) {
            LOG_ERROR("balancer: foot joint {} has no thigh above its shin", foot[i]);
            return false;
        }
        // Ankle: plantar/dorsiflexion (side to side at rest) and inversion
        // (along the foot); hip: ab/adduction (along the facing).
        const HumanBody::Part& f = body.parts()[static_cast<size_t>(footPart_[i])];
        ankleAheadAxis_[i] = axisAlong(f, math::kRight);
        ankleSideAxis_[i] = axisAlong(f, math::kForward);
        hipSideAxis_[i] = axisAlong(body.parts()[static_cast<size_t>(thighPart_[i])], math::kForward);
    }
    owned_.assign(body.parts().size(), 0);
    driven_.assign(body.parts().size(), 0);
    return true;
}

void Balancer::plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2]) {
    std::fill(owned_.begin(), owned_.end(), uint8_t{0});
    std::fill(driven_.begin(), driven_.end(), uint8_t{0});
    stanceCount_ = 0;
    force_ = math::Vec3(0.0f);
    share_[0] = share_[1] = 0.0f;

    // 1. The support: every foot that touches the ground (Jolt's contacts),
    // also one meant to lift that has not started to (it still carries). A
    // foot lifting carries nothing even while its toes still drag.
    for (int i = 0; i < 2; ++i)
        if (down[i]) lifting_[i] = false;  // a lift once started goes on until the foot is meant down again
    math::Vec3 sum(0.0f), ankle[2];
    float groundY[2] = {0.0f, 0.0f};
    int points = 0;
    for (int i = 0; i < 2; ++i) {
        // A touching part can press anywhere on its sole (pressing a raised
        // side brings it back down), so the foot's support is the soles of
        // its touching parts, not only the corners touching this step (a
        // tilted foot would then press its own edge and stay tilted).
        footPoints_[i] = 0;
        int contacts = 0;
        for (int part : {footPart_[i], toePart_[i]}) {
            if (part < 0 || lifting_[i]) continue;
            const HumanBody::Part& p = body.parts()[static_cast<size_t>(part)];
            math::Vec3 touch[kPointsPerFoot];
            const int n = world.groundContacts(p.body, touch, kPointsPerFoot);
            if (n == 0) continue;
            for (int k = 0; k < n; ++k) groundY[i] += touch[k].y;
            contacts += n;
            const int sole = footPoints_[i] + 4 <= kPointsPerFoot ? soleCorners(world, p, footPoint_[i] + footPoints_[i]) : 0;
            if (sole > 0) {
                footPoints_[i] += sole;
                continue;
            }
            for (int k = 0; k < n && footPoints_[i] < kPointsPerFoot; ++k) footPoint_[i][footPoints_[i]++] = touch[k];
        }
        for (int k = 0; k < footPoints_[i]; ++k) sum += footPoint_[i][k];
        points += footPoints_[i];
        stance_[i] = contacts > 0;
        if (stance_[i]) groundY[i] /= static_cast<float>(contacts);
        footHull_[i].build(footPoint_[i], footPoints_[i]);
        const HumanBody::Part& foot = body.parts()[static_cast<size_t>(footPart_[i])];
        ankle[i] = (world.transform(foot.body) * math::inverse(foot.shape)).position;  // the foot's joint
        if (stance_[i]) ++stanceCount_;
    }
    for (int i = 0; i < 2; ++i) liftReady_[i] = lifting_[i] || !stance_[i];
    if (stanceCount_ == 0) return;  // nothing to stand on: no balance torque
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        // The stance leg carries the body from the foot (its support torque,
        // not the hanging-limb hold).
        for (int part : {footPart_[i], shinPart_[i], thighPart_[i]}) owned_[static_cast<size_t>(part)] = 1;
        // The axes the pressure moves the body with: the ankle's, and on two
        // feet the hip's ab/adduction (the parallelogram over the feet).
        driven_[static_cast<size_t>(footPart_[i])] = 1;
        if (stanceCount_ == 2) driven_[static_cast<size_t>(thighPart_[i])] = 1;
    }
    centre_ = sum / static_cast<float>(points);

    // The aim: between the stance ankles; while one foot is meant to lift but
    // still carries, over the other's ankle alone (the weight shifts first).
    const int unloading = stanceCount_ == 2 && !down[0] && down[1] ? 0 : stanceCount_ == 2 && !down[1] && down[0] ? 1 : -1;
    const math::Vec3 aimAnkle = unloading >= 0     ? ankle[1 - unloading]
                                : stanceCount_ == 2 ? 0.5f * (ankle[0] + ankle[1])
                                                    : ankle[stance_[0] ? 0 : 1];
    aim_ = math::Vec3(aimAnkle.x, centre_.y, aimAnkle.z);

    // 2. The capture point and the pressure point.
    const math::Vec3 com = body.centreOfMass(world);
    const math::Vec3 vel = body.centreOfMassVelocity(world);
    const float mass = body.totalMass();
    const float g = std::max(-world.world().gravity.y, 0.0f);
    const float h = std::max(com.y - centre_.y, 0.1f);
    const float omega0 = std::sqrt(g / h);
    const math::Vec3 rate(vel.x, 0.0f, vel.z);
    capture_ = math::Vec3(com.x, centre_.y, com.z) + (omega0 > 0.0f ? rate / omega0 : math::Vec3(0.0f));
    // Peterka's feedback as the pressure point it makes (the ankle torque
    // KP theta + KD theta' = m g (p - aim), theta = (x - aim) / h), kept
    // inside the support: the hull of every stance contact.
    const float weightLever = std::max(mass * g * h, 1e-3f);
    const math::Vec3 off(com.x - aim_.x, 0.0f, com.z - aim_.z);
    const math::Vec3 still = aim_ + settings_.ankleStiffness * off / weightLever;  // for the position alone
    const math::Vec3 want = still + settings_.ankleDamping * rate / weightLever;
    // The unloading foot may lift once the other alone can hold the body,
    // where it is and as it moves: the pressure the feedback asks for, for
    // its position and for its position and motion, lies inside that foot.
    if (unloading >= 0 && footHull_[1 - unloading].contains(flat(still)) &&
        footHull_[1 - unloading].contains(flat(want)))
        lifting_[unloading] = liftReady_[unloading] = true;
    math::Vec3 all[2 * kPointsPerFoot];
    int n = 0;
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < footPoints_[i]; ++k) all[n++] = footPoint_[i][k];
    Hull support;
    support.build(all, n);
    const math::Vec2 p = support.closest(flat(want));
    pressure_ = math::Vec3(p.x, centre_.y, p.y);

    // 3. The ground force: the inverted pendulum's across, the weight up.
    const float pull = mass * omega0 * omega0;
    force_ = math::Vec3(pull * (com.x - pressure_.x), mass * g, pull * (com.z - pressure_.z));

    // 4. Each foot's share and its own pressure point: two feet by where p
    // lies between their ankles, the rest of p's offset on each foot.
    if (stanceCount_ == 2) {
        const math::Vec2 a0 = flat(ankle[0]), a1 = flat(ankle[1]);
        const math::Vec2 d = a1 - a0;
        const float len2 = math::dot(d, d);
        const float t = len2 > 1e-6f ? std::clamp(math::dot(p - a0, d) / len2, 0.0f, 1.0f) : 0.5f;
        share_[0] = 1.0f - t;
        share_[1] = t;
        const math::Vec2 rest = p - (a0 + t * d);
        for (int i = 0; i < 2; ++i) {
            const math::Vec2 q = footHull_[i].closest(flat(ankle[i]) + rest);
            footPressure_[i] = math::Vec3(q.x, groundY[i], q.y);
        }
    } else {
        const int i = stance_[0] ? 0 : 1;
        share_[i] = 1.0f;
        const math::Vec2 q = footHull_[i].closest(p);
        footPressure_[i] = math::Vec3(q.x, groundY[i], q.y);
    }
}

void Balancer::apply(physics::PhysicsWorld& world, const HumanBody& body) const {
    if (stanceCount_ == 0) return;
    const std::vector<HumanBody::Part>& parts = body.parts();
    const math::Vec3 g = world.world().gravity;
    // Peterka's KP and passive damping B are for the stance ankles together.
    const float stiffness = settings_.ankleStiffness / static_cast<float>(stanceCount_);
    const float damping = settings_.passiveDamping / static_cast<float>(stanceCount_);
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        if (stanceCount_ == 2)
            axisSpring(world, parts[static_cast<size_t>(thighPart_[i])], hipSideAxis_[i], math::Vec3(0.0f), kNoStiffness,
                       damping);

        // 5. J^T through the leg, from the foot up: each joint holds the
        // ground's force at this foot's pressure point and the weight of the
        // leg below it (statics), as equal and opposite torques.
        const math::Vec3 force = share_[i] * force_;
        float below = 0.0f;
        math::Vec3 belowMoment(0.0f);
        auto carry = [&](int part) {
            if (part < 0) return;
            const HumanBody::Part& p = parts[static_cast<size_t>(part)];
            below += p.mass;
            belowMoment += p.mass * world.transform(p.body).position;
        };
        auto torqueAt = [&](const HumanBody::Part& p) {
            const math::Vec3 joint = (world.transform(p.body) * math::inverse(p.shape)).position;
            math::Vec3 t = -math::cross(footPressure_[i] - joint, force) - math::cross(belowMoment - below * joint, g);
            const float size = math::length(t);
            if (size > p.strength) t *= p.strength / size;  // no stronger than the muscle
            return t;
        };
        auto torque = [&](int part) {
            const HumanBody::Part& p = parts[static_cast<size_t>(part)];
            const math::Vec3 t = torqueAt(p);
            world.addTorque(p.body, t);
            world.addTorque(parts[static_cast<size_t>(p.parent)].body, -t);
        };
        carry(toePart_[i]);
        carry(footPart_[i]);
        // The ankle: through its springs, solved with the foot's contacts (an
        // added torque on the light foot made it chatter and spin); their
        // slope is Peterka's KP, their aim gives the torque above now.
        const HumanBody::Part& foot = parts[static_cast<size_t>(footPart_[i])];
        const math::Vec3 ankleTorque = torqueAt(foot);
        axisSpring(world, foot, ankleAheadAxis_[i], ankleTorque, stiffness, damping);
        if (!foot.hinge) axisSpring(world, foot, ankleSideAxis_[i], ankleTorque, stiffness, damping);
        carry(shinPart_[i]);
        torque(shinPart_[i]);   // knee
        carry(thighPart_[i]);
        torque(thighPart_[i]);  // hip
    }
}

} // namespace agents
