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
        resolveContacts(s);
    }
}

void SphereSolver::resolveContacts(BodyState& s) const {
    auto respond = [&](const Contact& c) {
        s.position += c.normal * c.depth;
        const float vn = math::dot(s.velocity, c.normal);
        if (vn >= 0.0f) return;  // already separating
        // Bounce: reverse the normal speed scaled by restitution; a bounce
        // slower than restSpeed is absorbed so the body settles on the surface.
        const float out = -c.restitution * vn;
        s.velocity += (out < settings_.restSpeed ? -vn : out - vn) * c.normal;
    };
    Contact c;
    for (int i = 0; i < colliders_.boxCount(); ++i)
        if (collide(colliders_.box(i), s.position, body_.radius, c)) respond(c);
    for (int i = 0; i < colliders_.sphereCount(); ++i)
        if (collide(colliders_.sphere(i), s.position, body_.radius, c)) respond(c);
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
