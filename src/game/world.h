#pragma once
// Game objects are EnTT entities. Anything visible carries a Renderable that
// links it to an object in the renderer's scene (a Filament entity behind the
// render interface). Each frame, syncToRenderer() pushes world transforms of
// changed entities into the renderer's transform system.

#include "math/math.h"
#include "render/renderer.h"

#include <entt/entity/registry.hpp>

namespace game {

struct TransformComponent {
    math::Transform local;
    bool dirty = true;  // set when changed; cleared after syncing to the renderer
};

struct Renderable {
    render::ObjectId object = render::kInvalidObject;
};

// Tag: the entity follows the interpolated sim probe.
struct FollowsProbe {};

class World {
public:
    entt::registry& registry() { return registry_; }

    // Creates an entity with a transform and a renderer object.
    entt::entity spawn(render::Renderer& renderer, const render::MeshData& mesh,
                       const render::MaterialDesc& material, const math::Transform& transform,
                       bool castShadows);

    void setTransform(entt::entity e, const math::Transform& t);

    // Destroys the entity and its renderer object.
    void despawn(render::Renderer& renderer, entt::entity e);

    // Pushes dirty transforms to the renderer. No allocations.
    void syncToRenderer(render::Renderer& renderer);

private:
    entt::registry registry_;
};

} // namespace game
