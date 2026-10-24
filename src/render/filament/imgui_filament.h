#pragma once
// Draws Dear ImGui output with Filament. Implements the ImGui 1.92 texture
// protocol (ImGuiBackendFlags_RendererHasTextures), so the font atlas can grow
// at runtime. All UI geometry goes into one renderable with one primitive per
// ImGui draw command; per-command material instances carry texture + scissor.

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <utils/Entity.h>

#include <vector>

struct ImDrawData;
struct ImTextureData;

namespace render {

class ImGuiFilament {
public:
    // `view` must be a dedicated UI view with an orthographic camera.
    void init(filament::Engine* engine, filament::Scene* scene, filament::Material* material);
    void shutdown();

    // Uploads geometry/textures for this frame. Call before Renderer::render(uiView).
    void update(ImDrawData* drawData, int framebufferWidth, int framebufferHeight);

private:
    void updateTexture(ImTextureData* tex);
    void ensureBuffers(size_t vertexCount, size_t indexCount);
    void rebuildRenderable(size_t primitiveCount);

    filament::Engine* engine_ = nullptr;
    filament::Scene* scene_ = nullptr;
    filament::Material* material_ = nullptr;
    filament::VertexBuffer* vb_ = nullptr;
    filament::IndexBuffer* ib_ = nullptr;
    size_t vbCapacity_ = 0, ibCapacity_ = 0;
    utils::Entity entity_;
    size_t primitiveCount_ = 0;
    bool renderableBuilt_ = false;
    std::vector<filament::MaterialInstance*> instances_;  // grows only
    std::vector<filament::Texture*> textures_;            // owned, for shutdown
};

} // namespace render
