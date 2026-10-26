#pragma once
// Static collision shapes (boxes, spheres) and sphere-vs-shape contact tests,
// shared by every sphere body we simulate (ball, later others). The ground is
// one large box. Fixed capacity, so the sim never allocates. Each shape has
// its own restitution (normal bounce); friction comes later (Step 3.3).

#include "math/math.h"

#include <array>

namespace core {
class Config;
}

namespace physics {

struct BoxCollider {
    math::Transform pose;  // centre and rotation (scale unused)
    math::Vec3 halfExtents{0.5f};
    float restitution = 0.5f;
};

struct SphereCollider {
    math::Vec3 centre{0.0f};
    float radius = 0.5f;
    float restitution = 0.5f;
};

// Where a sphere touches a shape: push its centre out along `normal` by
// `depth` to separate them.
struct Contact {
    math::Vec3 normal{0.0f, 1.0f, 0.0f};  // from the shape toward the sphere
    float depth = 0.0f;
    float restitution = 0.0f;
};

bool collide(const BoxCollider& box, math::Vec3 centre, float radius, Contact& out);
bool collide(const SphereCollider& sphere, math::Vec3 centre, float radius, Contact& out);

class ColliderSet {
public:
    static constexpr int kMaxBoxes = 32;
    static constexpr int kMaxSpheres = 32;

    // Reads `c`'s "colliders" ("boxes", "spheres"). Load time.
    void load(const core::Config& c);
    void clear() { boxCount_ = sphereCount_ = 0; }
    bool add(const BoxCollider& box);
    bool add(const SphereCollider& sphere);

    int boxCount() const { return boxCount_; }
    int sphereCount() const { return sphereCount_; }
    const BoxCollider& box(int i) const { return boxes_[static_cast<size_t>(i)]; }
    const SphereCollider& sphere(int i) const { return spheres_[static_cast<size_t>(i)]; }

private:
    std::array<BoxCollider, kMaxBoxes> boxes_{};
    std::array<SphereCollider, kMaxSpheres> spheres_{};
    int boxCount_ = 0;
    int sphereCount_ = 0;
};

} // namespace physics
