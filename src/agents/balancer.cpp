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

math::Vec3 frameAxis(const HumanBody::Part& p, int axis) {
    return axis == 0 ? p.frameX : axis == 1 ? p.frameY : p.frameZ;
}

// The balance as a spring on one joint axis (Jolt's stiffness-and-damping
// motor, solved with the contacts): `turn` (world rotation vector) is how
// the part's joint must turn, body b against body a, to bring the sway to
// zero; the spring pulls that way with the given stiffness and damping, no
// harder than the joint's muscle.
void balanceSpring(physics::PhysicsWorld& world, const HumanBody::Part& p, int axis, math::Vec3 turn,
                   float stiffness, float damping) {
    const math::Quat bone = world.transform(p.body).rotation * math::inverse(p.shape.rotation);
    if (p.hinge) {
        world.setHingeSpring(p.link, math::dot(turn, bone * p.frameX), stiffness, damping, p.strength);
        return;
    }
    world.setSixDofAxisSpring(p.link, axis, math::dot(turn, bone * frameAxis(p, axis)), stiffness, damping,
                              p.strength);
}

} // namespace

BalanceSettings readBalanceSettings(const core::Config& joints) {
    BalanceSettings s;
    const float perDeg = math::degrees(1.0f);  // N m/deg -> N m/rad
    s.ankleStiffness = joints.getFloat("balance.ankleStiffness", s.ankleStiffness / perDeg) * perDeg;
    s.ankleDamping = joints.getFloat("balance.ankleDamping", s.ankleDamping / perDeg) * perDeg;
    s.sideStiffness = joints.getFloat("balance.sideStiffness", s.sideStiffness / perDeg) * perDeg;
    s.sideDamping = joints.getFloat("balance.sideDamping", s.sideDamping / perDeg) * perDeg;
    return s;
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
        // The thigh: two parts up from the foot (shin, thigh); its joint is the hip.
        const int shin = body.parts()[static_cast<size_t>(footPart_[i])].parent;
        thighPart_[i] = body.parts()[static_cast<size_t>(shin)].parent;
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
    return true;
}

void Balancer::plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2]) {
    std::fill(owned_.begin(), owned_.end(), uint8_t{0});
    stanceCount_ = 0;
    torque_ = sideTorque_ = math::Vec3(0.0f);

    // The support: the ground contacts of the feet meant to be down; the
    // pivot: their ankles.
    math::Vec3 sum(0.0f), ankles(0.0f), facing(0.0f);
    int points = 0;
    math::Vec3 buffer[kPointsPerPart];
    for (int i = 0; i < 2; ++i) {
        stance_[i] = false;
        if (!down[i]) continue;
        for (int part : {footPart_[i], toePart_[i]}) {
            if (part < 0) continue;
            const int n = world.groundContacts(body.parts()[static_cast<size_t>(part)].body, buffer, kPointsPerPart);
            for (int k = 0; k < n; ++k) sum += buffer[k];
            points += n;
            stance_[i] = stance_[i] || n > 0;
        }
        if (stance_[i]) {
            ++stanceCount_;
            const HumanBody::Part& foot = body.parts()[static_cast<size_t>(footPart_[i])];
            const math::Transform bone = world.transform(foot.body) * math::inverse(foot.shape);
            ankles += bone.position;  // the foot's joint
            facing += bone.rotation * math::kForward;
        }
    }
    if (stanceCount_ == 0) return;  // nothing to stand on: no balance torque
    // The loads the balancer carries (no hanging-limb hold there): the stance
    // ankles, and with both feet down the hips.
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        owned_[static_cast<size_t>(footPart_[i])] = 1;
        if (stanceCount_ == 2) owned_[static_cast<size_t>(thighPart_[i])] = 1;
    }
    centre_ = sum / static_cast<float>(points);
    pivot_ = ankles / static_cast<float>(stanceCount_);

    // Sway: the body's angle about the stance ankles (Peterka's inverted
    // pendulum; angle vector, right-hand rule) and its rate; the capture
    // point for reading.
    const math::Vec3 com = body.centreOfMass(world);
    const math::Vec3 vel = body.centreOfMassVelocity(world);
    const float h = std::max(com.y - pivot_.y, 0.1f);
    const math::Vec3 off(com.x - pivot_.x, 0.0f, com.z - pivot_.z);
    const math::Vec3 rate(vel.x, 0.0f, vel.z);
    const float omega0 = std::sqrt(std::max(-world.world().gravity.y, 0.0f) / h);
    capture_ = math::Vec3(com.x, centre_.y, com.z) + (omega0 > 0.0f ? rate / omega0 : math::Vec3(0.0f));
    const math::Vec3 sway = math::cross(math::kUp, off) / h;
    const math::Vec3 swayRate = math::cross(math::kUp, rate) / h;
    // Ahead and back: the ankles (about the feet's side-to-side axis). Side to
    // side: the hips load and unload the legs (Winter 1995), the legs and
    // pelvis turning together as a parallelogram over the two feet; on one
    // foot, its ankle (a narrow foot tips if it cannot carry it).
    const math::Vec3 ahead = math::safeNormalize(math::Vec3(facing.x, 0.0f, facing.z), math::kForward);
    const math::Vec3 side = math::cross(math::kUp, ahead);
    swayAhead_ = side * math::dot(sway, side);
    swaySide_ = ahead * math::dot(sway, ahead);
    torque_ = -side * math::dot(settings_.ankleStiffness * sway + settings_.ankleDamping * swayRate, side);
    sideTorque_ = -ahead * math::dot(settings_.sideStiffness * sway + settings_.sideDamping * swayRate, ahead);
}

void Balancer::apply(physics::PhysicsWorld& world, const HumanBody& body) const {
    if (stanceCount_ == 0) return;
    // Springs in parallel: each stance joint carries its share of the gains.
    const float share = 1.0f / static_cast<float>(stanceCount_);
    const bool hips = stanceCount_ == 2;
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        // The ankle, the foot planted: the body above must turn back by the
        // sway, so the foot turns by it against the shin.
        const HumanBody::Part& foot = body.parts()[static_cast<size_t>(footPart_[i])];
        balanceSpring(world, foot, ankleAheadAxis_[i], swayAhead_, settings_.ankleStiffness * share,
                      settings_.ankleDamping * share);
        if (foot.hinge) continue;
        if (!hips) {
            // One foot: side to side is its ankle's inversion.
            balanceSpring(world, foot, ankleSideAxis_[i], swaySide_, settings_.sideStiffness, settings_.sideDamping);
            continue;
        }
        // Two feet: the legs tilt over them (inversion free), the hips'
        // ab/adduction turning the legs back against the level pelvis.
        world.setSixDofAxisFree(foot.link, ankleSideAxis_[i]);
        const HumanBody::Part& thigh = body.parts()[static_cast<size_t>(thighPart_[i])];
        balanceSpring(world, thigh, hipSideAxis_[i], -swaySide_, settings_.sideStiffness * share,
                      settings_.sideDamping * share);
    }
}

} // namespace agents
