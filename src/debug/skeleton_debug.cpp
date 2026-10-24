#include "debug/skeleton_debug.h"

#include <algorithm>

namespace debug {

namespace {

constexpr float kJointRadius = 0.015f;
constexpr float kAxisSize = 0.06f;
constexpr float kComRadius = 0.03f;
constexpr render::Color kBoneColor = render::colors::kYellow;
constexpr render::Color kJointColor = render::colors::kWhite;
constexpr render::Color kComColor = render::colors::kMagenta;

math::Vec3 jointWorld(const std::vector<math::Mat4>& models, const math::Transform& root, int joint) {
    return root.transformPoint(math::Vec3(models[static_cast<size_t>(joint)][3]));
}

math::Vec3 tipWorld(const rig::SkeletonDef& def, const rig::Rig& rig, const std::vector<math::Mat4>& models,
                    const math::Transform& root, int joint) {
    math::Vec3 tip = def.bones[static_cast<size_t>(rig.boneDefIndex(joint))].tip * rig.height();
    return root.transformPoint(math::Vec3(models[static_cast<size_t>(joint)] * math::Vec4(tip, 1.0f)));
}

bool isLeaf(const rig::Rig& rig, int joint) {
    for (int c = joint + 1; c < rig.numJoints(); ++c)
        if (rig.parent(c) == joint) return false;
    return true;
}

} // namespace

void drawSkeleton(render::DebugDraw& dd, const rig::SkeletonDef& def, const rig::Rig& rig,
                  const std::vector<math::Mat4>& models, const math::Transform& root, const SkeletonDrawFlags& flags) {
    for (int j = 0; j < rig.numJoints(); ++j) {
        math::Vec3 p = jointWorld(models, root, j);
        if (flags.bones) {
            if (rig.parent(j) >= 0) dd.line(jointWorld(models, root, rig.parent(j)), p, kBoneColor);
            if (isLeaf(rig, j)) dd.line(p, tipWorld(def, rig, models, root, j), kBoneColor);
        }
        // Small bones (fingers) get proportionally small spheres and axes.
        float bone = rig.parent(j) >= 0 ? math::distance(p, jointWorld(models, root, rig.parent(j))) : kAxisSize;
        if (flags.joints) dd.sphere(p, std::min(kJointRadius, 0.3f * bone), kJointColor, 8);
        if (flags.axes) dd.axes(root * math::toTransform(models[static_cast<size_t>(j)]), std::min(kAxisSize, 0.8f * bone));
    }
    if (flags.centreOfMass) {
        math::Vec3 com = centreOfMass(def, rig, models, root);
        math::Vec3 ground(com.x, root.position.y + 0.005f, com.z);
        dd.sphere(com, kComRadius, kComColor, 12);
        dd.line(com, ground, kComColor);
        dd.circle(ground, math::kUp, kComRadius, kComColor);
    }
}

math::Vec3 centreOfMass(const rig::SkeletonDef& def, const rig::Rig& rig, const std::vector<math::Mat4>& models,
                        const math::Transform& root) {
    math::Vec3 sum(0.0f);
    float total = 0.0f;
    for (int j = 0; j < rig.numJoints(); ++j) {
        float mass = def.bones[static_cast<size_t>(rig.boneDefIndex(j))].massPercent;
        if (mass <= 0.0f) continue;
        math::Vec3 a = jointWorld(models, root, j);
        math::Vec3 b = tipWorld(def, rig, models, root, j);
        for (int c = j + 1; c < rig.numJoints(); ++c) {
            if (rig.parent(c) == j) {
                b = jointWorld(models, root, c);
                break;
            }
        }
        sum += mass * 0.5f * (a + b);
        total += mass;
    }
    return total > 0.0f ? sum / total : root.position;
}

} // namespace debug
