// Filament implementation of render::Renderer.
//
// Two views are rendered each frame into the same swap chain:
//   1. the 3D scene (lit objects, sun + IBL, shadows, debug lines);
//   2. the ImGui overlay (orthographic, no post-processing, blended on top).
// Scene objects are Filament entities; their transforms live in Filament's
// TransformManager so hierarchies (e.g. glTF models) work natively.

#include "render/renderer.h"

#include "core/log.h"
#include "render/filament/imgui_filament.h"

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/SwapChain.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <geometry/SurfaceOrientation.h>
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/TextureProvider.h>
#include <gltfio/materials/uberarchive.h>
#include <utils/EntityManager.h>

#include <imgui.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fl = filament;
namespace fm = filament::math;

namespace render {

namespace {

fm::float3 toF3(math::Vec3 v) { return {v.x, v.y, v.z}; }

fm::mat4f toMat4f(const math::Mat4& m) {
    fm::mat4f out;
    static_assert(sizeof(out) == sizeof(m));
    std::memcpy(&out, math::data(m), sizeof(out));  // both column-major
    return out;
}

// sRGB (0..1) -> linear, the space Filament's material parameters expect.
math::Vec3 srgbToLinear(math::Vec3 c) {
    auto f = [](float v) { return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f); };
    return {f(c.x), f(c.y), f(c.z)};
}

// Hands Filament a malloc'd copy that its driver thread frees when consumed.
template <typename T>
fl::backend::BufferDescriptor copyBuffer(const T* data, size_t count) {
    size_t bytes = count * sizeof(T);
    void* copy = std::malloc(bytes);
    std::memcpy(copy, data, bytes);
    return fl::backend::BufferDescriptor(copy, bytes, [](void* b, size_t, void*) { std::free(b); });
}

std::vector<uint8_t> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

// Procedural sky radiance used for the irradiance spherical harmonics:
// zenith colour blending to horizon, ground colour below.
math::Vec3 skyRadiance(const LightingDesc& l, math::Vec3 dir) {
    if (dir.y >= 0.0f) return math::lerp(l.horizonColor, l.skyColor, std::sqrt(dir.y));
    return math::lerp(l.horizonColor, l.groundColor, std::min(1.0f, -dir.y * 4.0f));
}

// Projects the sky onto 3-band SH, pre-scaled the way Filament expects
// (see IndirectLight::Builder::irradiance): sh_i = (A_l / pi) * K_i^2 * integral(f * poly_i).
void computeSkySH(const LightingDesc& l, fm::float3 out[9]) {
    // Basis polynomials in Filament's order and their K_i^2 * A_l / pi factors.
    const float k[9] = {
        0.282095f * 0.282095f,           // 1
        0.488603f * 0.488603f * 2.0f / 3.0f,  // y
        0.488603f * 0.488603f * 2.0f / 3.0f,  // z
        0.488603f * 0.488603f * 2.0f / 3.0f,  // x
        1.092548f * 1.092548f * 0.25f,   // xy
        1.092548f * 1.092548f * 0.25f,   // yz
        0.315392f * 0.315392f * 0.25f,   // 3z^2 - 1
        1.092548f * 1.092548f * 0.25f,   // xz
        0.546274f * 0.546274f * 0.25f,   // x^2 - y^2
    };
    math::Vec3 acc[9] = {};
    const int n = 64;  // integration grid (theta x phi)
    for (int i = 0; i < n; ++i) {
        float theta = math::kPi * (static_cast<float>(i) + 0.5f) / n;
        float st = std::sin(theta), ct = std::cos(theta);
        float dA = (math::kPi / n) * (math::kTwoPi / (2 * n)) * st;  // solid angle element
        for (int j = 0; j < 2 * n; ++j) {
            float phi = math::kTwoPi * (static_cast<float>(j) + 0.5f) / (2 * n);
            math::Vec3 d(st * std::cos(phi), ct, st * std::sin(phi));  // y up
            math::Vec3 f = skyRadiance(l, d) * dA;
            float x = d.x, y = d.y, z = d.z;
            const float poly[9] = {1, y, z, x, x * y, y * z, 3 * z * z - 1, x * z, x * x - y * y};
            for (int b = 0; b < 9; ++b) acc[b] += f * poly[b];
        }
    }
    for (int b = 0; b < 9; ++b) out[b] = toF3(acc[b] * k[b]);
}

} // namespace

class FilamentRenderer final : public Renderer {
public:
    bool init(const RendererDesc& desc) override;
    void shutdown() override;
    void resize(int width, int height) override;

    ObjectId createObject(const MeshData& mesh, const MaterialDesc& material, bool castShadows) override;
    void setObjectTransform(ObjectId id, const math::Mat4& local) override;
    void setObjectParent(ObjectId child, ObjectId parent) override;
    void setObjectVisible(ObjectId id, bool visible) override;
    ObjectId loadModel(const std::string& path) override;

    void setLighting(const LightingDesc& lighting) override;
    void setShadowsEnabled(bool enabled) override;
    void setCamera(const CameraDesc& camera) override;

    DebugDraw& debugDraw() override { return *debugDraw_; }
    void setDebugDrawEnabled(bool enabled) override;

    bool render(ImDrawData* ui) override;

private:
    struct Object {
        utils::Entity root;                  // transform owner
        std::vector<utils::Entity> entities; // everything added to the scene
        fl::VertexBuffer* vb = nullptr;
        fl::IndexBuffer* ib = nullptr;
        fl::MaterialInstance* mi = nullptr;
        fl::gltfio::FilamentAsset* asset = nullptr;
        bool visible = true;
    };

    fl::Material* loadMaterial(const char* name);
    void createDebugLines(size_t maxLines);
    void uploadDebugLines();
    void applyViewport();

    RendererDesc desc_;
    int width_ = 1, height_ = 1;

    fl::Engine* engine_ = nullptr;
    fl::SwapChain* swapChain_ = nullptr;
    fl::Renderer* renderer_ = nullptr;
    fl::Scene* scene_ = nullptr;
    fl::View* view_ = nullptr;
    fl::Camera* camera_ = nullptr;
    utils::Entity cameraEntity_;

    fl::Scene* uiScene_ = nullptr;
    fl::View* uiView_ = nullptr;
    fl::Camera* uiCamera_ = nullptr;
    utils::Entity uiCameraEntity_;
    ImGuiFilament imgui_;

    fl::Material* litMaterial_ = nullptr;
    fl::Material* lineMaterial_ = nullptr;
    fl::Material* uiMaterial_ = nullptr;

    utils::Entity sun_;
    fl::IndirectLight* ibl_ = nullptr;
    fl::Skybox* skybox_ = nullptr;

    std::vector<Object> objects_;

    // Debug lines: one renderable, one dynamic vertex buffer.
    std::unique_ptr<DebugDraw> debugDraw_;
    utils::Entity lineEntity_;
    fl::VertexBuffer* lineVb_ = nullptr;
    fl::IndexBuffer* lineIb_ = nullptr;
    fl::MaterialInstance* lineMi_ = nullptr;
    bool debugEnabled_ = true;
    bool linesInScene_ = false;

    // glTF loading (created on first use).
    fl::gltfio::MaterialProvider* gltfMaterials_ = nullptr;
    fl::gltfio::AssetLoader* gltfLoader_ = nullptr;
    fl::gltfio::TextureProvider* gltfTextures_ = nullptr;
};

bool FilamentRenderer::init(const RendererDesc& desc) {
    desc_ = desc;
    width_ = std::max(1, desc.width);
    height_ = std::max(1, desc.height);

    fl::Engine::Backend backend =
        desc.backend == "vulkan" ? fl::Engine::Backend::VULKAN : fl::Engine::Backend::OPENGL;
    engine_ = fl::Engine::Builder().backend(backend).build();
    if (!engine_) {
        LOG_ERROR("Filament: failed to create engine ({} backend)", desc.backend);
        return false;
    }
    swapChain_ = engine_->createSwapChain(desc.nativeWindow);
    renderer_ = engine_->createRenderer();

    // Present at the display rate.
    fl::Renderer::FrameRateOptions fro;
    fro.interval = 1;
    renderer_->setFrameRateOptions(fro);
    fl::Renderer::ClearOptions clear;
    clear.clearColor = {0.0, 0.0, 0.0, 1.0};
    clear.clear = true;
    renderer_->setClearOptions(clear);

    litMaterial_ = loadMaterial("lit");
    lineMaterial_ = loadMaterial("debug_line");
    uiMaterial_ = loadMaterial("ui");
    if (!litMaterial_ || !lineMaterial_ || !uiMaterial_) return false;

    auto& em = utils::EntityManager::get();

    // --- Main 3D view ---
    scene_ = engine_->createScene();
    view_ = engine_->createView();
    cameraEntity_ = em.create();
    camera_ = engine_->createCamera(cameraEntity_);
    view_->setScene(scene_);
    view_->setCamera(camera_);
    view_->setShadowType(fl::View::ShadowType::PCF);
    fl::View::MultiSampleAntiAliasingOptions msaa;
    msaa.enabled = true;
    msaa.sampleCount = 4;
    view_->setMultiSampleAntiAliasingOptions(msaa);
    fl::View::AmbientOcclusionOptions ao;
    ao.enabled = true;
    view_->setAmbientOcclusionOptions(ao);

    // --- UI view: drawn after the scene, blended, no post-processing ---
    uiScene_ = engine_->createScene();
    uiView_ = engine_->createView();
    uiCameraEntity_ = em.create();
    uiCamera_ = engine_->createCamera(uiCameraEntity_);
    uiView_->setScene(uiScene_);
    uiView_->setCamera(uiCamera_);
    uiView_->setPostProcessingEnabled(false);
    uiView_->setBlendMode(fl::View::BlendMode::TRANSLUCENT);
    uiView_->setShadowingEnabled(false);
    imgui_.init(engine_, uiScene_, uiMaterial_);

    // --- Sun ---
    sun_ = em.create();
    fl::LightManager::ShadowOptions shadows;
    shadows.mapSize = static_cast<uint32_t>(desc.shadowMapSize);
    shadows.shadowCascades = static_cast<uint8_t>(std::clamp(desc.shadowCascades, 1, 4));
    shadows.shadowFar = desc.shadowFar;
    shadows.stable = true;  // no shimmering while the camera moves
    fl::LightManager::ShadowCascades::computePracticalSplits(
        shadows.cascadeSplitPositions, shadows.shadowCascades, 0.1f, desc.shadowFar, 0.5f);
    fl::LightManager::Builder(fl::LightManager::Type::SUN)
        .castShadows(true)
        .shadowOptions(shadows)
        .sunAngularRadius(0.545f)
        .build(*engine_, sun_);
    scene_->addEntity(sun_);

    createDebugLines(desc.debugLineCapacity);
    applyViewport();
    LOG_INFO("Filament ready: {} backend, {}x{}", desc.backend, width_, height_);
    return true;
}

fl::Material* FilamentRenderer::loadMaterial(const char* name) {
    std::string path = desc_.assetDir + "/materials/" + name + ".filamat";
    std::vector<uint8_t> data = readFile(path);
    if (data.empty()) {
        LOG_ERROR("Filament: cannot read material {}", path);
        return nullptr;
    }
    return fl::Material::Builder().package(data.data(), data.size()).build(*engine_);
}

void FilamentRenderer::createDebugLines(size_t maxLines) {
    debugDraw_ = std::make_unique<DebugDraw>(maxLines);
    uint32_t maxVerts = static_cast<uint32_t>(debugDraw_->capacityVertices());

    lineVb_ = fl::VertexBuffer::Builder()
                  .vertexCount(maxVerts)
                  .bufferCount(1)
                  .attribute(fl::VertexAttribute::POSITION, 0, fl::VertexBuffer::AttributeType::FLOAT3,
                             offsetof(DebugVertex, position), sizeof(DebugVertex))
                  .attribute(fl::VertexAttribute::COLOR, 0, fl::VertexBuffer::AttributeType::UBYTE4,
                             offsetof(DebugVertex, color), sizeof(DebugVertex))
                  .normalized(fl::VertexAttribute::COLOR)
                  .build(*engine_);
    // Lines are drawn as consecutive pairs, so the index buffer is just 0..N-1, built once.
    lineIb_ = fl::IndexBuffer::Builder()
                  .indexCount(maxVerts)
                  .bufferType(fl::IndexBuffer::IndexType::UINT)
                  .build(*engine_);
    std::vector<uint32_t> indices(maxVerts);
    for (uint32_t i = 0; i < maxVerts; ++i) indices[i] = i;
    lineIb_->setBuffer(*engine_, copyBuffer(indices.data(), indices.size()));

    lineMi_ = lineMaterial_->createInstance();
    lineEntity_ = utils::EntityManager::get().create();
    fl::RenderableManager::Builder(1)
        .boundingBox({{-1e4f, -1e4f, -1e4f}, {1e4f, 1e4f, 1e4f}})
        .culling(false)
        .castShadows(false)
        .receiveShadows(false)
        .material(0, lineMi_)
        .geometry(0, fl::RenderableManager::PrimitiveType::LINES, lineVb_, lineIb_, 0, 2)
        .build(*engine_, lineEntity_);
}

void FilamentRenderer::uploadDebugLines() {
    size_t count = debugEnabled_ ? debugDraw_->vertexCount() : 0;
    bool show = count >= 2;
    if (show != linesInScene_) {
        if (show) scene_->addEntity(lineEntity_);
        else scene_->remove(lineEntity_);
        linesInScene_ = show;
    }
    if (show) {
        lineVb_->setBufferAt(*engine_, 0, copyBuffer(debugDraw_->vertices(), count));
        auto& rm = engine_->getRenderableManager();
        rm.setGeometryAt(rm.getInstance(lineEntity_), 0, fl::RenderableManager::PrimitiveType::LINES, lineVb_,
                         lineIb_, 0, count);
    }
    debugDraw_->clear();
}

void FilamentRenderer::applyViewport() {
    fl::Viewport vp(0, 0, static_cast<uint32_t>(width_), static_cast<uint32_t>(height_));
    view_->setViewport(vp);
    uiView_->setViewport(vp);
}

void FilamentRenderer::resize(int width, int height) {
    width_ = std::max(1, width);
    height_ = std::max(1, height);
    applyViewport();
}

ObjectId FilamentRenderer::createObject(const MeshData& mesh, const MaterialDesc& mat, bool castShadows) {
    Object obj;
    size_t n = mesh.positions.size();

    // Tangent frames (quaternions) from normals, as Filament's lit shading needs.
    std::vector<fm::short4> tangents(n);
    fl::geometry::SurfaceOrientation* so = fl::geometry::SurfaceOrientation::Builder()
                                               .vertexCount(n)
                                               .normals(reinterpret_cast<const fm::float3*>(mesh.normals.data()))
                                               .build();
    so->getQuats(tangents.data(), n);
    delete so;

    obj.vb = fl::VertexBuffer::Builder()
                 .vertexCount(static_cast<uint32_t>(n))
                 .bufferCount(2)
                 .attribute(fl::VertexAttribute::POSITION, 0, fl::VertexBuffer::AttributeType::FLOAT3)
                 .attribute(fl::VertexAttribute::TANGENTS, 1, fl::VertexBuffer::AttributeType::SHORT4)
                 .normalized(fl::VertexAttribute::TANGENTS)
                 .build(*engine_);
    obj.vb->setBufferAt(*engine_, 0, copyBuffer(mesh.positions.data(), n));
    obj.vb->setBufferAt(*engine_, 1, copyBuffer(tangents.data(), n));

    obj.ib = fl::IndexBuffer::Builder()
                 .indexCount(static_cast<uint32_t>(mesh.indices.size()))
                 .bufferType(fl::IndexBuffer::IndexType::UINT)
                 .build(*engine_);
    obj.ib->setBuffer(*engine_, copyBuffer(mesh.indices.data(), mesh.indices.size()));

    obj.mi = litMaterial_->createInstance();
    obj.mi->setParameter("baseColor", toF3(srgbToLinear(mat.baseColor)));
    obj.mi->setParameter("roughness", mat.roughness);
    obj.mi->setParameter("metallic", mat.metallic);
    obj.mi->setParameter("stripeWidth", mat.stripeWidth);
    obj.mi->setParameter("stripeStrength", mat.stripeStrength);

    math::Vec3 lo(1e30f), hi(-1e30f);
    for (const auto& p : mesh.positions) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }

    obj.root = utils::EntityManager::get().create();
    fl::RenderableManager::Builder(1)
        .boundingBox({toF3(lo), toF3(hi)})
        .material(0, obj.mi)
        .geometry(0, fl::RenderableManager::PrimitiveType::TRIANGLES, obj.vb, obj.ib)
        .castShadows(castShadows)
        .receiveShadows(true)
        .build(*engine_, obj.root);
    engine_->getTransformManager().create(obj.root);
    obj.entities.push_back(obj.root);
    scene_->addEntity(obj.root);

    objects_.push_back(std::move(obj));
    return static_cast<ObjectId>(objects_.size() - 1);
}

void FilamentRenderer::setObjectTransform(ObjectId id, const math::Mat4& local) {
    if (id >= objects_.size()) return;
    auto& tm = engine_->getTransformManager();
    tm.setTransform(tm.getInstance(objects_[id].root), toMat4f(local));
}

void FilamentRenderer::setObjectParent(ObjectId child, ObjectId parent) {
    if (child >= objects_.size()) return;
    auto& tm = engine_->getTransformManager();
    auto parentInst = parent < objects_.size() ? tm.getInstance(objects_[parent].root)
                                               : fl::TransformManager::Instance{};
    tm.setParent(tm.getInstance(objects_[child].root), parentInst);
}

void FilamentRenderer::setObjectVisible(ObjectId id, bool visible) {
    if (id >= objects_.size() || objects_[id].visible == visible) return;
    Object& o = objects_[id];
    o.visible = visible;
    if (visible) scene_->addEntities(o.entities.data(), o.entities.size());
    else scene_->removeEntities(o.entities.data(), o.entities.size());
}

ObjectId FilamentRenderer::loadModel(const std::string& path) {
    using namespace fl::gltfio;
    std::vector<uint8_t> bytes = readFile(path);
    if (bytes.empty()) {
        LOG_ERROR("loadModel: cannot read {}", path);
        return kInvalidObject;
    }
    if (!gltfLoader_) {
        gltfMaterials_ = createUbershaderProvider(engine_, UBERARCHIVE_DEFAULT_DATA, UBERARCHIVE_DEFAULT_SIZE);
        AssetConfiguration cfg{};
        cfg.engine = engine_;
        cfg.materials = gltfMaterials_;
        gltfLoader_ = AssetLoader::create(cfg);
        gltfTextures_ = createStbProvider(engine_);
    }
    FilamentAsset* asset = gltfLoader_->createAsset(bytes.data(), static_cast<uint32_t>(bytes.size()));
    if (!asset) {
        LOG_ERROR("loadModel: failed to parse {}", path);
        return kInvalidObject;
    }

    ResourceConfiguration rc{};
    rc.engine = engine_;
    rc.normalizeSkinningWeights = true;
    ResourceLoader resources(rc);
    resources.addTextureProvider("image/png", gltfTextures_);
    resources.addTextureProvider("image/jpeg", gltfTextures_);
    // External buffers/images (.gltf + .bin) are resolved next to the model file.
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    const char* const* uris = asset->getResourceUris();
    for (size_t i = 0; i < asset->getResourceUriCount(); ++i) {
        std::vector<uint8_t> data = readFile((dir / uris[i]).string());
        if (data.empty()) {
            LOG_WARN("loadModel: missing resource {}", uris[i]);
            continue;
        }
        resources.addResourceData(uris[i], copyBuffer(data.data(), data.size()));
    }
    if (!resources.loadResources(asset)) {
        LOG_ERROR("loadModel: failed to load resources for {}", path);
        gltfLoader_->destroyAsset(asset);
        return kInvalidObject;
    }
    asset->releaseSourceData();

    Object obj;
    obj.asset = asset;
    obj.root = asset->getRoot();
    obj.entities.assign(asset->getEntities(), asset->getEntities() + asset->getEntityCount());
    scene_->addEntities(obj.entities.data(), obj.entities.size());
    objects_.push_back(std::move(obj));
    return static_cast<ObjectId>(objects_.size() - 1);
}

void FilamentRenderer::setLighting(const LightingDesc& l) {
    auto& lm = engine_->getLightManager();
    auto li = lm.getInstance(sun_);
    lm.setDirection(li, toF3(math::safeNormalize(l.sunDirection, {0, -1, 0})));
    lm.setColor(li, toF3(l.sunColor));
    lm.setIntensity(li, l.sunIlluminance);

    fm::float3 sh[9];
    computeSkySH(l, sh);
    if (ibl_) engine_->destroy(ibl_);
    ibl_ = fl::IndirectLight::Builder().irradiance(3, sh).intensity(l.iblIntensity).build(*engine_);
    scene_->setIndirectLight(ibl_);

    if (skybox_) engine_->destroy(skybox_);
    math::Vec3 sky = math::lerp(l.horizonColor, l.skyColor, 0.6f);
    skybox_ = fl::Skybox::Builder().color({sky.x, sky.y, sky.z, 1.0f}).intensity(l.iblIntensity).build(*engine_);
    scene_->setSkybox(skybox_);
}

void FilamentRenderer::setShadowsEnabled(bool enabled) { view_->setShadowingEnabled(enabled); }

void FilamentRenderer::setDebugDrawEnabled(bool enabled) { debugEnabled_ = enabled; }

void FilamentRenderer::setCamera(const CameraDesc& c) {
    double aspect = static_cast<double>(width_) / static_cast<double>(height_);
    camera_->setProjection(c.fovYDegrees, aspect, c.nearPlane, c.farPlane, fl::Camera::Fov::VERTICAL);
    camera_->lookAt(fm::double3(c.eye.x, c.eye.y, c.eye.z), fm::double3(c.target.x, c.target.y, c.target.z),
                    fm::double3(c.up.x, c.up.y, c.up.z));
}

bool FilamentRenderer::render(ImDrawData* ui) {
    uploadDebugLines();

    // UI camera maps ImGui display coordinates (top-left origin) to the viewport.
    if (ui && ui->DisplaySize.x > 0 && ui->DisplaySize.y > 0) {
        uiCamera_->setProjection(fl::Camera::Projection::ORTHO, 0.0, ui->DisplaySize.x, ui->DisplaySize.y, 0.0,
                                 0.0, 1.0);
    }
    imgui_.update(ui, width_, height_);

    if (!renderer_->beginFrame(swapChain_)) return false;
    renderer_->render(view_);
    renderer_->render(uiView_);
    renderer_->endFrame();
    return true;
}

void FilamentRenderer::shutdown() {
    if (!engine_) return;
    auto& em = utils::EntityManager::get();
    imgui_.shutdown();

    for (Object& o : objects_) {
        if (o.asset) {
            scene_->removeEntities(o.entities.data(), o.entities.size());
            gltfLoader_->destroyAsset(o.asset);
            continue;
        }
        scene_->remove(o.root);
        engine_->getRenderableManager().destroy(o.root);
        engine_->getTransformManager().destroy(o.root);
        em.destroy(o.root);
        engine_->destroy(o.vb);
        engine_->destroy(o.ib);
        engine_->destroy(o.mi);
    }
    objects_.clear();
    if (gltfLoader_) {
        gltfMaterials_->destroyMaterials();
        delete gltfMaterials_;
        delete gltfTextures_;
        fl::gltfio::AssetLoader::destroy(&gltfLoader_);
    }

    engine_->getRenderableManager().destroy(lineEntity_);
    em.destroy(lineEntity_);
    engine_->destroy(lineVb_);
    engine_->destroy(lineIb_);
    engine_->destroy(lineMi_);

    engine_->getLightManager().destroy(sun_);
    em.destroy(sun_);
    if (ibl_) engine_->destroy(ibl_);
    if (skybox_) engine_->destroy(skybox_);

    engine_->destroy(litMaterial_);
    engine_->destroy(lineMaterial_);
    engine_->destroy(uiMaterial_);

    engine_->destroy(view_);
    engine_->destroy(uiView_);
    engine_->destroy(scene_);
    engine_->destroy(uiScene_);
    engine_->destroyCameraComponent(cameraEntity_);
    engine_->destroyCameraComponent(uiCameraEntity_);
    em.destroy(cameraEntity_);
    em.destroy(uiCameraEntity_);
    engine_->destroy(renderer_);
    engine_->destroy(swapChain_);
    fl::Engine::destroy(&engine_);
}

std::unique_ptr<Renderer> createFilamentRenderer() { return std::make_unique<FilamentRenderer>(); }

} // namespace render
