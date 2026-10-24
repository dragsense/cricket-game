#pragma once
// Placeholder fielder body: one capsule or box per bone, rigidly attached and
// drawn through the render interface. Kept apart from the skeleton so a
// skinned glTF body can replace it later without touching rig/agents code.
// Meshes are built once in height-ratio units and scaled by the rig height,
// so changing height needs no new meshes.

#include "game/world.h"
#include "math/math.h"
#include "rig/rig.h"

#include <vector>

namespace core {
class Config;
}

namespace game {

class FielderVisual {
public:
    // Creates one entity per bone with a visual. Load-time only.
    void build(World& world, render::Renderer& renderer, const rig::SkeletonDef& def, const rig::Rig& rig,
               const core::Config& config);

    // Places every part from model-space joint matrices and the character root.
    void update(World& world, const rig::Rig& rig, const std::vector<math::Mat4>& models, const math::Transform& root);

    void setVisible(render::Renderer& renderer, World& world, bool visible);

private:
    struct Part {
        entt::entity entity;
        int joint;
        math::Transform local;  // part frame relative to its joint, in ratio units (scaled by height)
    };
    std::vector<Part> parts_;
};

} // namespace game
