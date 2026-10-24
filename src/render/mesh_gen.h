#pragma once
// Procedural primitive meshes (everything is created from code). Output is
// API-neutral: positions + normals + triangle indices. Load-time only.

#include "math/math.h"

#include <cstdint>
#include <vector>

namespace render {

struct MeshData {
    std::vector<math::Vec3> positions;
    std::vector<math::Vec3> normals;
    std::vector<uint32_t> indices;  // triangles, counter-clockwise front faces
};

// Flat rectangle in the XZ plane centred on the origin, facing +Y.
MeshData makePlane(float sizeX, float sizeZ);
// Flat disc in the XZ plane, facing +Y.
MeshData makeDisc(float radius, int segments);
// Axis-aligned box centred on the origin.
MeshData makeBox(math::Vec3 size);
// Capped cylinder along +Y, base at y = 0.
MeshData makeCylinder(float radius, float height, int segments);
// Capsule along +Y, base at y = 0, total height includes both hemispheres.
MeshData makeCapsule(float radius, float height, int segments, int rings);
MeshData makeSphere(float radius, int segments, int rings);

} // namespace render
