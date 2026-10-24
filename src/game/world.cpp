#include "game/world.h"

namespace game {

entt::entity World::spawn(render::Renderer& renderer, const render::MeshData& mesh,
                          const render::MaterialDesc& material, const math::Transform& transform,
                          bool castShadows) {
    entt::entity e = registry_.create();
    registry_.emplace<TransformComponent>(e, transform, true);
    registry_.emplace<Renderable>(e, renderer.createObject(mesh, material, castShadows));
    return e;
}

void World::setTransform(entt::entity e, const math::Transform& t) {
    auto& tc = registry_.get<TransformComponent>(e);
    tc.local = t;
    tc.dirty = true;
}

void World::syncToRenderer(render::Renderer& renderer) {
    auto view = registry_.view<TransformComponent, const Renderable>();
    for (auto [e, tc, r] : view.each()) {
        if (!tc.dirty) continue;
        renderer.setObjectTransform(r.object, tc.local.toMat4());
        tc.dirty = false;
    }
}

} // namespace game
