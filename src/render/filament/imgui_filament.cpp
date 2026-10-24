#include "render/filament/imgui_filament.h"

#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <utils/EntityManager.h>

#include <imgui.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>

using namespace filament;

namespace render {

namespace {

// Note: Filament consumes buffers asynchronously on its driver thread, so all
// uploads below hand over malloc'd copies that the driver frees when done.

Texture* toTexture(ImTextureID id) { return reinterpret_cast<Texture*>(static_cast<uintptr_t>(id)); }

} // namespace

void ImGuiFilament::init(Engine* engine, Scene* scene, Material* material) {
    engine_ = engine;
    scene_ = scene;
    material_ = material;
    entity_ = utils::EntityManager::get().create();

    ImGuiIO& io = ImGui::GetIO();
    io.BackendRendererName = "cricket_filament";
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset | ImGuiBackendFlags_RendererHasTextures;
}

void ImGuiFilament::shutdown() {
    if (!engine_) return;
    // Let ImGui know its textures are gone.
    for (ImTextureData* tex : ImGui::GetPlatformIO().Textures) {
        if (tex->RefCount == 1) {
            tex->SetTexID(ImTextureID_Invalid);
            tex->SetStatus(ImTextureStatus_Destroyed);
        }
    }
    if (renderableBuilt_) {
        scene_->remove(entity_);
        engine_->getRenderableManager().destroy(entity_);
    }
    utils::EntityManager::get().destroy(entity_);
    for (MaterialInstance* mi : instances_) engine_->destroy(mi);
    for (Texture* t : textures_) engine_->destroy(t);
    if (vb_) engine_->destroy(vb_);
    if (ib_) engine_->destroy(ib_);
    instances_.clear();
    textures_.clear();
    engine_ = nullptr;
}

void ImGuiFilament::updateTexture(ImTextureData* tex) {
    if (tex->Status == ImTextureStatus_WantCreate) {
        IM_ASSERT(tex->Format == ImTextureFormat_RGBA32);
        Texture* t = Texture::Builder()
                         .width(static_cast<uint32_t>(tex->Width))
                         .height(static_cast<uint32_t>(tex->Height))
                         .levels(1)
                         .format(Texture::InternalFormat::RGBA8)
                         .sampler(Texture::Sampler::SAMPLER_2D)
                         .build(*engine_);
        textures_.push_back(t);
        tex->SetTexID(static_cast<ImTextureID>(reinterpret_cast<uintptr_t>(t)));
    }
    if (tex->Status == ImTextureStatus_WantCreate || tex->Status == ImTextureStatus_WantUpdates) {
        // Simplest correct approach: re-upload the whole image. Updates are
        // rare (new glyphs), so this costs nothing in practice.
        size_t bytes = static_cast<size_t>(tex->Width) * tex->Height * 4;
        void* pixels = std::malloc(bytes);
        std::memcpy(pixels, tex->GetPixels(), bytes);
        Texture::PixelBufferDescriptor pb(pixels, bytes,
                                          Texture::Format::RGBA, Texture::Type::UBYTE,
                                          [](void* buffer, size_t, void*) { std::free(buffer); });
        toTexture(tex->GetTexID())->setImage(*engine_, 0, std::move(pb));
        tex->SetStatus(ImTextureStatus_OK);
    }
    if (tex->Status == ImTextureStatus_WantDestroy && tex->UnusedFrames > 0) {
        Texture* t = toTexture(tex->GetTexID());
        textures_.erase(std::remove(textures_.begin(), textures_.end(), t), textures_.end());
        engine_->destroy(t);
        tex->SetTexID(ImTextureID_Invalid);
        tex->SetStatus(ImTextureStatus_Destroyed);
    }
}

void ImGuiFilament::ensureBuffers(size_t vertexCount, size_t indexCount) {
    if (vertexCount > vbCapacity_) {
        if (vb_) engine_->destroy(vb_);
        vbCapacity_ = std::max<size_t>(vertexCount * 2, 4096);
        constexpr uint8_t stride = sizeof(ImDrawVert);
        vb_ = VertexBuffer::Builder()
                  .vertexCount(static_cast<uint32_t>(vbCapacity_))
                  .bufferCount(1)
                  .attribute(VertexAttribute::POSITION, 0, VertexBuffer::AttributeType::FLOAT2, offsetof(ImDrawVert, pos), stride)
                  .attribute(VertexAttribute::UV0, 0, VertexBuffer::AttributeType::FLOAT2, offsetof(ImDrawVert, uv), stride)
                  .attribute(VertexAttribute::COLOR, 0, VertexBuffer::AttributeType::UBYTE4, offsetof(ImDrawVert, col), stride)
                  .normalized(VertexAttribute::COLOR)
                  .build(*engine_);
    }
    if (indexCount > ibCapacity_) {
        if (ib_) engine_->destroy(ib_);
        ibCapacity_ = std::max<size_t>(indexCount * 2, 8192);
        ib_ = IndexBuffer::Builder()
                  .indexCount(static_cast<uint32_t>(ibCapacity_))
                  .bufferType(IndexBuffer::IndexType::UINT)
                  .build(*engine_);
    }
}

void ImGuiFilament::rebuildRenderable(size_t primitiveCount) {
    auto& rm = engine_->getRenderableManager();
    if (renderableBuilt_) {
        scene_->remove(entity_);
        rm.destroy(entity_);
    }
    while (instances_.size() < primitiveCount) instances_.push_back(material_->createInstance());

    RenderableManager::Builder b(primitiveCount);
    b.boundingBox({{-1e5f, -1e5f, -1e5f}, {1e5f, 1e5f, 1e5f}})
        .culling(false)
        .castShadows(false)
        .receiveShadows(false);
    for (size_t i = 0; i < primitiveCount; ++i) {
        b.geometry(i, RenderableManager::PrimitiveType::TRIANGLES, vb_, ib_, 0, 3);
        b.material(i, instances_[i]);
        b.blendOrder(i, static_cast<uint16_t>(i));  // draw in ImGui's order
    }
    b.build(*engine_, entity_);
    scene_->addEntity(entity_);
    renderableBuilt_ = true;
    primitiveCount_ = primitiveCount;
}

void ImGuiFilament::update(ImDrawData* dd, int fbWidth, int fbHeight) {
    if (!dd) return;
    if (dd->Textures)
        for (ImTextureData* tex : *dd->Textures)
            if (tex->Status != ImTextureStatus_OK) updateTexture(tex);

    // Count draw commands (user callbacks are not supported).
    size_t prims = 0;
    for (const ImDrawList* list : dd->CmdLists)
        for (const ImDrawCmd& cmd : list->CmdBuffer)
            if (!cmd.UserCallback && cmd.ElemCount > 0) ++prims;

    if (prims == 0 || dd->TotalVtxCount == 0) {
        if (renderableBuilt_) {
            scene_->remove(entity_);
            engine_->getRenderableManager().destroy(entity_);
            renderableBuilt_ = false;
            primitiveCount_ = 0;
        }
        return;
    }

    size_t vbOld = vbCapacity_, ibOld = ibCapacity_;
    ensureBuffers(static_cast<size_t>(dd->TotalVtxCount), static_cast<size_t>(dd->TotalIdxCount));
    if (prims != primitiveCount_ || vbOld != vbCapacity_ || ibOld != ibCapacity_ || !renderableBuilt_)
        rebuildRenderable(prims);

    // Pack every list into one vertex and one index buffer. Indices become
    // absolute (list base + VtxOffset), since Filament has no base vertex.
    size_t vtxBytes = static_cast<size_t>(dd->TotalVtxCount) * sizeof(ImDrawVert);
    size_t idxBytes = static_cast<size_t>(dd->TotalIdxCount) * sizeof(uint32_t);
    auto* vtx = static_cast<ImDrawVert*>(std::malloc(vtxBytes));
    auto* idx = static_cast<uint32_t*>(std::malloc(idxBytes));

    auto& rm = engine_->getRenderableManager();
    auto inst = rm.getInstance(entity_);
    const TextureSampler sampler(TextureSampler::MinFilter::LINEAR, TextureSampler::MagFilter::LINEAR);
    const ImVec2 scale = dd->FramebufferScale;

    size_t vtxBase = 0, idxBase = 0, prim = 0;
    for (const ImDrawList* list : dd->CmdLists) {
        std::memcpy(vtx + vtxBase, list->VtxBuffer.Data, list->VtxBuffer.Size * sizeof(ImDrawVert));
        for (const ImDrawCmd& cmd : list->CmdBuffer) {
            if (cmd.UserCallback || cmd.ElemCount == 0) continue;
            uint32_t* out = idx + idxBase;
            const ImDrawIdx* in = list->IdxBuffer.Data + cmd.IdxOffset;
            uint32_t base = static_cast<uint32_t>(vtxBase + cmd.VtxOffset);
            for (unsigned i = 0; i < cmd.ElemCount; ++i) out[i] = base + in[i];

            // Scissor: ImGui clip rect is top-left origin in display units;
            // Filament wants bottom-left origin in pixels.
            float x0 = std::clamp((cmd.ClipRect.x - dd->DisplayPos.x) * scale.x, 0.0f, static_cast<float>(fbWidth));
            float y0 = std::clamp((cmd.ClipRect.y - dd->DisplayPos.y) * scale.y, 0.0f, static_cast<float>(fbHeight));
            float x1 = std::clamp((cmd.ClipRect.z - dd->DisplayPos.x) * scale.x, 0.0f, static_cast<float>(fbWidth));
            float y1 = std::clamp((cmd.ClipRect.w - dd->DisplayPos.y) * scale.y, 0.0f, static_cast<float>(fbHeight));

            MaterialInstance* mi = instances_[prim];
            mi->setParameter("albedo", toTexture(cmd.GetTexID()), sampler);
            mi->setScissor(static_cast<uint32_t>(x0), static_cast<uint32_t>(fbHeight - y1),
                           static_cast<uint32_t>(std::max(0.0f, x1 - x0)),
                           static_cast<uint32_t>(std::max(0.0f, y1 - y0)));
            rm.setGeometryAt(inst, prim, RenderableManager::PrimitiveType::TRIANGLES, vb_, ib_, idxBase, cmd.ElemCount);
            ++prim;
            idxBase += cmd.ElemCount;
        }
        vtxBase += static_cast<size_t>(list->VtxBuffer.Size);
    }

    auto freeFn = [](void* buffer, size_t, void*) { std::free(buffer); };
    vb_->setBufferAt(*engine_, 0, backend::BufferDescriptor(vtx, vtxBytes, freeFn));
    ib_->setBuffer(*engine_, backend::BufferDescriptor(idx, idxBase * sizeof(uint32_t), freeFn));
}

} // namespace render
