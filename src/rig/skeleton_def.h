#pragma once
// Skeleton description loaded from config/fielder_skeleton.json. Every length
// is a ratio of standing height, so one height value scales the whole body.
// Pure data: the ozz skeleton is built from it by rig::Rig, and the body
// visual (game layer) reads the "visual" part. Load-time only.

#include "math/math.h"

#include <string>
#include <vector>

namespace core {
class Config;
}

namespace rig {

inline constexpr int kNoParent = -1;

enum class VisualShape { None, Capsule, Box };

// Display shape rigidly attached to a bone, in the bone's local frame.
// Positions and sizes are ratios of height.
struct BoneVisual {
    VisualShape shape = VisualShape::None;
    std::string material;
    math::Vec3 from{0.0f};  // capsule
    math::Vec3 to{0.0f};    // capsule
    float radius = 0.0f;    // capsule
    math::Vec3 center{0.0f};  // box
    math::Vec3 size{0.0f};    // box
};

struct BoneDef {
    std::string name;
    int parent = kNoParent;  // index into SkeletonDef::bones; parents come first
    math::Vec3 offset{0.0f};  // joint position relative to the parent joint (ratio of H)
    math::Vec3 tip{0.0f};     // end of a leaf bone (ratio of H); zero for inner bones
    float massPercent = 0.0f;
    BoneVisual visual;
};

struct SkeletonDef {
    float height = 1.8f;  // default standing height, metres
    float minHeight = 1.5f;
    float maxHeight = 2.1f;
    math::Vec3 spawnPosition{0.0f};
    float spawnYaw = 0.0f;  // radians
    std::vector<BoneDef> bones;

    int find(const std::string& name) const;
    // Thigh + shank of one leg, as a ratio of height.
    float legLengthRatio() const;
};

// Parses the skeleton config (mirrored bones expanded to _l/_r). Logs and
// returns false if the description is invalid (unknown parent, no bones...).
bool loadSkeletonDef(const core::Config& config, SkeletonDef& out);

} // namespace rig
