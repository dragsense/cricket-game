#pragma once
// Renderer interface. Game code talks only to this; the Filament backend lives
// in src/render/filament and could be swapped without touching callers.
// The simulation never calls the renderer: the app copies interpolated sim
// state into it once per rendered frame.

#include "math/math.h"
#include "render/debug_draw.h"
#include "render/mesh_gen.h"

#include <cstdint>
#include <memory>
#include <string>

struct ImDrawData;

namespace render {

using ObjectId = uint32_t;
inline constexpr ObjectId kInvalidObject = 0xFFFFFFFFu;

struct RendererDesc {
    void* nativeWindow = nullptr;  // HWND on Windows
    int width = 0, height = 0;     // drawable size in pixels
    std::string backend = "opengl"; // "opengl" | "vulkan"
    bool vsync = true;
    std::string assetDir = "assets";
    int shadowMapSize = 2048;
    int shadowCascades = 4;
    float shadowFar = 120.0f;       // metres covered by sun shadows
    size_t debugLineCapacity = 65536;
};

struct MaterialDesc {
    math::Vec3 baseColor{0.8f};   // sRGB, 0..1
    float roughness = 0.6f;
    float metallic = 0.0f;
    // Mowing stripes (field only): alternating bands along world X.
    float stripeWidth = 0.0f;     // metres; 0 disables
    float stripeStrength = 0.0f;  // 0..1 brightness variation
};

struct CameraDesc {
    math::Vec3 eye{0.0f};
    math::Vec3 target{0, 0, -1};
    math::Vec3 up{0, 1, 0};
    float fovYDegrees = 50.0f;
    float nearPlane = 0.1f;
    float farPlane = 2000.0f;
};

struct LightingDesc {
    math::Vec3 sunDirection{-0.4f, -1.0f, -0.3f};  // direction the light travels
    math::Vec3 sunColor{1.0f, 0.96f, 0.9f};
    float sunIlluminance = 100000.0f;              // lux
    math::Vec3 skyColor{0.45f, 0.65f, 0.95f};      // linear, zenith
    math::Vec3 horizonColor{0.75f, 0.85f, 0.95f};  // linear
    math::Vec3 groundColor{0.25f, 0.3f, 0.2f};     // linear, bounce from grass
    float iblIntensity = 30000.0f;
};

class Renderer {
public:
    virtual ~Renderer() = default;

    virtual bool init(const RendererDesc& desc) = 0;
    virtual void shutdown() = 0;
    virtual void resize(int width, int height) = 0;

    // Scene objects. Creation allocates; do it at load time, not per frame.
    virtual ObjectId createObject(const MeshData& mesh, const MaterialDesc& material, bool castShadows) = 0;
    // Transform relative to the parent (or world if none). Uses the backend's
    // transform hierarchy, so children follow parents automatically.
    virtual void setObjectTransform(ObjectId id, const math::Mat4& local) = 0;
    virtual void setObjectParent(ObjectId child, ObjectId parent) = 0;  // kInvalidObject = root
    virtual void setObjectVisible(ObjectId id, bool visible) = 0;

    // Loads a glTF/GLB model (via gltfio) and returns its root object, or
    // kInvalidObject on failure. Load-time only.
    virtual ObjectId loadModel(const std::string& path) = 0;

    virtual void setLighting(const LightingDesc& lighting) = 0;
    virtual void setShadowsEnabled(bool enabled) = 0;
    virtual void setCamera(const CameraDesc& camera) = 0;

    // Lines submitted here are drawn this frame, then cleared.
    virtual DebugDraw& debugDraw() = 0;
    virtual void setDebugDrawEnabled(bool enabled) = 0;

    // Draws the scene, debug lines and (optionally) ImGui output, then presents.
    // Returns false if the frame was skipped (e.g. GPU busy).
    virtual bool render(ImDrawData* ui) = 0;
};

std::unique_ptr<Renderer> createFilamentRenderer();

} // namespace render
