#include "physics/sim/sphere_solver.h"

#include <algorithm>
#include <cmath>

namespace physics {

namespace {

constexpr float kTouchSkin = 0.002f;  // m; how close counts as touching

} // namespace

bool SphereSolver::addForce(const ForceGenerator* force) {
    if (!force || forceCount_ >= kMaxForces) return false;
    forces_[static_cast<size_t>(forceCount_++)] = force;
    return true;
}

math::Vec3 SphereSolver::acceleration(const BodyState& s) const {
    math::Vec3 f(0.0f);
    for (int i = 0; i < forceCount_; ++i) f += forces_[static_cast<size_t>(i)]->force(s, body_, world_);
    return f / body_.mass;
}

void SphereSolver::step(BodyState& s, float dt) const {
    // Substeps from the speed at the start of the tick: each moves the body at
    // most maxTravelPerSubstep radii, so it cannot pass through a thin collider.
    const float maxTravel = settings_.maxTravelPerSubstep * body_.radius;
    const float travel = (math::length(s.velocity) + math::length(acceleration(s)) * dt) * dt;
    const int n = std::clamp(static_cast<int>(std::ceil(travel / maxTravel)), 1, settings_.maxSubsteps);
    const float h = dt / static_cast<float>(n);
    for (int i = 0; i < n; ++i) {
        s.velocity += acceleration(s) * h;  // semi-implicit Euler
        s.position += s.velocity * h;
        const math::Vec3& w = s.angularVelocity;
        s.orientation = math::normalize(s.orientation + 0.5f * h * (math::Quat(0.0f, w.x, w.y, w.z) * s.orientation));

        Contact c;
        for (int b = 0; b < colliders_.boxCount(); ++b)
            if (collide(colliders_.box(b), s.position, body_.radius, c)) resolveContact(s, c, h);
        for (int b = 0; b < colliders_.sphereCount(); ++b)
            if (collide(colliders_.sphere(b), s.position, body_.radius, c)) resolveContact(s, c, h);
    }
}

void SphereSolver::resolveContact(BodyState& s, const Contact& c, float h) const {
    s.position += c.normal * c.depth;
    const float vn = math::dot(s.velocity, c.normal);
    if (vn >= 0.0f) return;  // already separating
    const float m = body_.mass, r = body_.radius, k = body_.inertiaFactor;

    // Normal: reverse the normal speed scaled by restitution; a bounce slower
    // than restSpeed is absorbed, so the body settles on the surface.
    const float bounce = -c.restitution * vn;
    const bool resting = bounce < settings_.restSpeed;
    const float vnAfter = resting ? 0.0f : bounce;
    s.velocity += (vnAfter - vn) * c.normal;
    const float jn = m * (vnAfter - vn);

    // Friction (grip-slip): the contact point's slip is opposed by at most
    // mu * normal impulse; once it stops slipping the body rolls (spin = v / r).
    const math::Vec3 arm = -c.normal * r;  // centre to contact point
    const math::Vec3 vt = s.velocity - math::dot(s.velocity, c.normal) * c.normal;
    const math::Vec3 slip = vt + math::cross(s.angularVelocity, arm);
    const float slipSpeed = math::length(slip);
    if (slipSpeed > math::kEpsilon) {
        const math::Vec3 dir = slip / slipSpeed;
        const float jStop = slipSpeed * m * k / (k + 1.0f);  // impulse that makes it roll
        const float jt = std::min(c.friction * jn, jStop);
        s.velocity -= dir * (jt / m);
        s.angularVelocity += math::cross(arm, -dir * jt) / (k * m * r * r);
    }

    // Rolling resistance while it stays on the surface: slows the rolling
    // speed by C_rr * normal gravity, keeping the spin matched; below
    // restSpeed it stops.
    if (resting) {
        const math::Vec3 t = s.velocity - math::dot(s.velocity, c.normal) * c.normal;
        const float speed = math::length(t);
        const float slow = c.rollingResistance * std::fabs(math::dot(world_.gravity, c.normal)) * h;
        if (speed - slow < settings_.restSpeed) {
            s.velocity -= t;
            s.angularVelocity = math::Vec3(0.0f);
        } else {
            const math::Vec3 dv = -t * (slow / speed);
            s.velocity += dv;
            s.angularVelocity += math::cross(c.normal, dv) / r;
        }
    }
}

bool SphereSolver::touching(const BodyState& s) const {
    Contact c;
    const float r = body_.radius + kTouchSkin;
    for (int i = 0; i < colliders_.boxCount(); ++i)
        if (collide(colliders_.box(i), s.position, r, c)) return true;
    for (int i = 0; i < colliders_.sphereCount(); ++i)
        if (collide(colliders_.sphere(i), s.position, r, c)) return true;
    return false;
}

} // namespace physics
