#include "physics/sim/colliders.h"

#include "core/config.h"
#include "core/log.h"

#include <cmath>

namespace physics {

namespace {

math::Vec3 vec3(const nlohmann::json& j, const char* key, math::Vec3 fallback) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return fallback;
    return {(*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>()};
}

} // namespace

bool collide(const BoxCollider& box, math::Vec3 centre, float radius, Contact& out) {
    const math::Quat toLocal = math::inverse(box.pose.rotation);
    const math::Vec3 local = toLocal * (centre - box.pose.position);
    const math::Vec3& h = box.halfExtents;
    const math::Vec3 closest = math::clamp(local, -h, h);
    math::Vec3 normal;
    if (closest == local) {
        // Centre inside the box: leave through the nearest face.
        const math::Vec3 gap = h - math::Vec3(std::fabs(local.x), std::fabs(local.y), std::fabs(local.z));
        int axis = gap.x < gap.y ? (gap.x < gap.z ? 0 : 2) : (gap.y < gap.z ? 1 : 2);
        normal = math::Vec3(0.0f);
        normal[axis] = local[axis] < 0.0f ? -1.0f : 1.0f;
        out.depth = gap[axis] + radius;
    } else {
        const math::Vec3 d = local - closest;
        const float dist = math::length(d);
        if (dist >= radius) return false;
        normal = d / dist;
        out.depth = radius - dist;
    }
    out.normal = box.pose.rotation * normal;
    out.restitution = box.restitution;
    return true;
}

bool collide(const SphereCollider& sphere, math::Vec3 centre, float radius, Contact& out) {
    const math::Vec3 d = centre - sphere.centre;
    const float reach = sphere.radius + radius;
    const float dist2 = math::length2(d);
    if (dist2 >= reach * reach) return false;
    const float dist = std::sqrt(dist2);
    out.normal = dist > math::kEpsilon ? d / dist : math::kUp;
    out.depth = reach - dist;
    out.restitution = sphere.restitution;
    return true;
}

void ColliderSet::load(const core::Config& c) {
    clear();
    const nlohmann::json& root = c.root();
    auto colliders = root.find("colliders");
    if (colliders == root.end()) return;
    if (auto boxes = colliders->find("boxes"); boxes != colliders->end() && boxes->is_array()) {
        for (const nlohmann::json& b : *boxes) {
            BoxCollider box;
            box.pose.position = vec3(b, "centre", box.pose.position);
            const math::Vec3 deg = vec3(b, "rotationDeg", math::Vec3(0.0f));
            box.pose.rotation = math::Quat(math::radians(deg));
            box.halfExtents = vec3(b, "halfExtents", box.halfExtents);
            box.restitution = b.value("restitution", box.restitution);
            if (!add(box)) break;
        }
    }
    if (auto spheres = colliders->find("spheres"); spheres != colliders->end() && spheres->is_array()) {
        for (const nlohmann::json& s : *spheres) {
            SphereCollider sphere;
            sphere.centre = vec3(s, "centre", sphere.centre);
            sphere.radius = s.value("radius", sphere.radius);
            sphere.restitution = s.value("restitution", sphere.restitution);
            if (!add(sphere)) break;
        }
    }
}

bool ColliderSet::add(const BoxCollider& box) {
    if (boxCount_ >= kMaxBoxes) {
        LOG_WARN("colliders: more than {} boxes, extra ignored", kMaxBoxes);
        return false;
    }
    boxes_[static_cast<size_t>(boxCount_++)] = box;
    return true;
}

bool ColliderSet::add(const SphereCollider& sphere) {
    if (sphereCount_ >= kMaxSpheres) {
        LOG_WARN("colliders: more than {} spheres, extra ignored", kMaxSpheres);
        return false;
    }
    spheres_[static_cast<size_t>(sphereCount_++)] = sphere;
    return true;
}

} // namespace physics
