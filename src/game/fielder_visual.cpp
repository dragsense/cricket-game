#include "game/fielder_visual.h"

#include "core/config.h"
#include "core/log.h"

namespace game {

namespace {

constexpr int kCapsuleSegments = 16;
constexpr int kCapsuleRings = 6;

render::MaterialDesc readMaterial(const core::Config& c, const std::string& name) {
    render::MaterialDesc m;
    std::string key = "materials." + name;
    m.baseColor = c.getVec3(key + ".color", m.baseColor);
    m.roughness = c.getFloat(key + ".roughness", m.roughness);
    return m;
}

} // namespace

void FielderVisual::build(World& world, render::Renderer& renderer, const rig::SkeletonDef& def, const rig::Rig& rig,
                          const core::Config& config) {
    parts_.clear();
    for (int j = 0; j < rig.numJoints(); ++j) {
        const rig::BoneVisual& v = def.bones[static_cast<size_t>(rig.boneDefIndex(j))].visual;
        render::MeshData mesh;
        math::Transform local;
        if (v.shape == rig::VisualShape::Capsule) {
            // makeCapsule runs along +Y from y = 0; shift so the hemisphere
            // centres sit on `from` and `to`.
            math::Vec3 axis = v.to - v.from;
            float len = math::length(axis);
            math::Transform shift;
            shift.position = math::Vec3(0.0f, -v.radius, 0.0f);
            render::appendMesh(mesh, render::makeCapsule(v.radius, len + 2.0f * v.radius, kCapsuleSegments, kCapsuleRings),
                               shift);
            local.position = v.from;
            local.rotation = math::fromTo(math::kUp, math::safeNormalize(axis, math::kUp));
        } else if (v.shape == rig::VisualShape::Box) {
            math::Transform centre;
            centre.position = v.center;
            render::appendMesh(mesh, render::makeBox(v.size), centre);
        } else {
            continue;
        }
        entt::entity e = world.spawn(renderer, mesh, readMaterial(config, v.material), math::Transform::identity(), true);
        parts_.push_back({e, j, local});
    }
    LOG_INFO("fielder visual: {} parts", parts_.size());
}

void FielderVisual::update(World& world, const rig::Rig& rig, const std::vector<math::Mat4>& models,
                           const math::Transform& root) {
    const float h = rig.height();
    for (const Part& p : parts_) {
        math::Transform local = p.local;
        local.position *= h;
        local.scale = math::Vec3(h);
        world.setTransform(p.entity, root * math::toTransform(models[static_cast<size_t>(p.joint)]) * local);
    }
}

void FielderVisual::setVisible(render::Renderer& renderer, World& world, bool visible) {
    for (const Part& p : parts_) renderer.setObjectVisible(world.registry().get<Renderable>(p.entity).object, visible);
}

} // namespace game
