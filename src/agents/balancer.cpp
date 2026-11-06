#include "agents/balancer.h"

#include "core/config.h"
#include "core/log.h"

#include <algorithm>
#include <cmath>

namespace agents {

BalanceSettings readBalanceSettings(const core::Config& joints) {
    BalanceSettings s;
    const float perDeg = math::degrees(1.0f);  // N m/deg -> N m/rad
    s.ankleStiffness = joints.getFloat("balance.ankleStiffness", s.ankleStiffness / perDeg) * perDeg;
    s.ankleDamping = joints.getFloat("balance.ankleDamping", s.ankleDamping / perDeg) * perDeg;
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
        // The ankle's plantar/dorsiflexion axis (side to side at rest, when
        // the bone frame has the character's axes): the balancer's.
        const HumanBody::Part& p = body.parts()[static_cast<size_t>(footPart_[i])];
        swayAxis_[i] = -1;
        if (!p.hinge) {
            const float side[3] = {std::fabs(p.frameX.x), std::fabs(p.frameY.x), std::fabs(p.frameZ.x)};
            swayAxis_[i] = side[0] >= side[1] && side[0] >= side[2] ? 0 : side[1] >= side[2] ? 1 : 2;
        }
    }
    owned_.assign(body.parts().size(), 0);
    return true;
}

void Balancer::plan(const physics::PhysicsWorld& world, const HumanBody& body, const bool down[2]) {
    std::fill(owned_.begin(), owned_.end(), uint8_t{0});
    stanceCount_ = 0;
    torque_ = math::Vec3(0.0f);

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
            owned_[static_cast<size_t>(footPart_[i])] = 1;
            const HumanBody::Part& foot = body.parts()[static_cast<size_t>(footPart_[i])];
            const math::Transform bone = world.transform(foot.body) * math::inverse(foot.shape);
            ankles += bone.position;  // the foot's joint
            facing += bone.rotation * math::kForward;
        }
    }
    if (stanceCount_ == 0) return;  // nothing to stand on: no balance torque
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
    // The ankles carry the sway ahead and back only (about the feet's
    // side-to-side axis); side to side, the hips load and unload the legs
    // (Winter 1995), and a narrow foot would tip on its edge.
    const math::Vec3 side = math::safeNormalize(math::cross(math::kUp, math::Vec3(facing.x, 0.0f, facing.z)), math::kRight);
    torque_ = -side * math::dot(settings_.ankleStiffness * sway + settings_.ankleDamping * swayRate, side);
}

void Balancer::apply(physics::PhysicsWorld& world, const HumanBody& body) const {
    if (stanceCount_ == 0) return;
    const math::Vec3 each = torque_ / static_cast<float>(stanceCount_);
    for (int i = 0; i < 2; ++i) {
        if (!stance_[i]) continue;
        const HumanBody::Part& foot = body.parts()[static_cast<size_t>(footPart_[i])];
        // Plantar/dorsiflexion is the balancer's; the other axes stay as the
        // muscle set them (drive).
        if (swayAxis_[i] < 0)
            world.setJointMotor(foot.link, physics::MotorDesc{}, false);
        else
            world.setSixDofFreeAxes(foot.link, swayAxis_[i] == 0, swayAxis_[i] == 1, swayAxis_[i] == 2);
        math::Vec3 t = each;
        const float size = math::length(t);
        if (size > foot.strength) t *= foot.strength / size;  // no stronger than the ankle
        world.addTorque(body.parts()[static_cast<size_t>(foot.parent)].body, t);  // the shin and all above
        world.addTorque(foot.body, -t);                                           // the foot, on the ground
    }
}

} // namespace agents
