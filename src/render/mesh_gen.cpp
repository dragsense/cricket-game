#include "render/mesh_gen.h"

namespace render {

using math::Vec3;

namespace {

uint32_t addVertex(MeshData& m, Vec3 p, Vec3 n) {
    m.positions.push_back(p);
    m.normals.push_back(n);
    return static_cast<uint32_t>(m.positions.size() - 1);
}

void addQuad(MeshData& m, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    // a-b-c-d counter-clockwise when viewed from the front.
    m.indices.insert(m.indices.end(), {a, b, c, a, c, d});
}

// Surface of revolution around +Y from a profile of (radius, y, normal) rings.
struct ProfilePoint {
    float radius, y;
    math::Vec2 normal;  // (outward, up) in the profile plane
};

void revolve(MeshData& m, const std::vector<ProfilePoint>& profile, int segments) {
    uint32_t base = static_cast<uint32_t>(m.positions.size());
    for (const auto& p : profile) {
        for (int s = 0; s <= segments; ++s) {
            float a = math::kTwoPi * static_cast<float>(s) / static_cast<float>(segments);
            float c = std::cos(a), sn = std::sin(a);
            addVertex(m, {p.radius * c, p.y, -p.radius * sn}, {p.normal.x * c, p.normal.y, -p.normal.x * sn});
        }
    }
    uint32_t stride = static_cast<uint32_t>(segments + 1);
    for (uint32_t r = 0; r + 1 < profile.size(); ++r) {
        for (uint32_t s = 0; s < static_cast<uint32_t>(segments); ++s) {
            uint32_t i0 = base + r * stride + s;
            uint32_t i1 = i0 + stride;
            // Rings go upwards; angle goes counter-clockwise seen from above.
            addQuad(m, i0, i0 + 1, i1 + 1, i1);
        }
    }
}

} // namespace

MeshData makePlane(float sizeX, float sizeZ) {
    MeshData m;
    float hx = 0.5f * sizeX, hz = 0.5f * sizeZ;
    Vec3 n{0, 1, 0};
    uint32_t a = addVertex(m, {-hx, 0, hz}, n);
    uint32_t b = addVertex(m, {hx, 0, hz}, n);
    uint32_t c = addVertex(m, {hx, 0, -hz}, n);
    uint32_t d = addVertex(m, {-hx, 0, -hz}, n);
    addQuad(m, a, b, c, d);
    return m;
}

MeshData makeDisc(float radius, int segments) {
    MeshData m;
    Vec3 n{0, 1, 0};
    uint32_t centre = addVertex(m, {0, 0, 0}, n);
    for (int s = 0; s <= segments; ++s) {
        float a = math::kTwoPi * static_cast<float>(s) / static_cast<float>(segments);
        addVertex(m, {radius * std::cos(a), 0, -radius * std::sin(a)}, n);
    }
    for (uint32_t s = 0; s < static_cast<uint32_t>(segments); ++s)
        m.indices.insert(m.indices.end(), {centre, centre + 1 + s, centre + 2 + s});
    return m;
}

MeshData makeBox(Vec3 size) {
    MeshData m;
    Vec3 h = size * 0.5f;
    // For each face: normal, and two tangent axes u, v with u x v = normal.
    const Vec3 faces[6][3] = {
        {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},  {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
        {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},  {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
        {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},   {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}},
    };
    for (const auto& f : faces) {
        Vec3 n = f[0], u = f[1], v = f[2];
        Vec3 c = n * h;
        Vec3 du = u * h, dv = v * h;
        uint32_t a = addVertex(m, c - du - dv, n);
        uint32_t b = addVertex(m, c + du - dv, n);
        uint32_t cc = addVertex(m, c + du + dv, n);
        uint32_t d = addVertex(m, c - du + dv, n);
        addQuad(m, a, b, cc, d);
    }
    return m;
}

MeshData makeCylinder(float radius, float height, int segments) {
    MeshData m;
    revolve(m, {{radius, 0, {1, 0}}, {radius, height, {1, 0}}}, segments);
    // Caps.
    MeshData top = makeDisc(radius, segments);
    uint32_t base = static_cast<uint32_t>(m.positions.size());
    for (size_t i = 0; i < top.positions.size(); ++i) addVertex(m, top.positions[i] + Vec3(0, height, 0), {0, 1, 0});
    for (uint32_t i : top.indices) m.indices.push_back(base + i);
    base = static_cast<uint32_t>(m.positions.size());
    for (size_t i = 0; i < top.positions.size(); ++i) addVertex(m, top.positions[i], {0, -1, 0});
    for (size_t i = 0; i < top.indices.size(); i += 3)  // flip winding for the bottom
        m.indices.insert(m.indices.end(), {base + top.indices[i], base + top.indices[i + 2], base + top.indices[i + 1]});
    return m;
}

MeshData makeCapsule(float radius, float height, int segments, int rings) {
    MeshData m;
    float cyl = std::max(0.0f, height - 2.0f * radius);
    std::vector<ProfilePoint> profile;
    // Bottom hemisphere from the pole up to the equator.
    for (int r = 0; r <= rings; ++r) {
        float a = -math::kHalfPi + math::kHalfPi * static_cast<float>(r) / static_cast<float>(rings);
        float c = std::cos(a), s = std::sin(a);
        profile.push_back({radius * c, radius + radius * s, {c, s}});
    }
    // Top hemisphere from the equator to the pole.
    for (int r = 0; r <= rings; ++r) {
        float a = math::kHalfPi * static_cast<float>(r) / static_cast<float>(rings);
        float c = std::cos(a), s = std::sin(a);
        profile.push_back({radius * c, radius + cyl + radius * s, {c, s}});
    }
    revolve(m, profile, segments);
    return m;
}

MeshData makeRing(float innerRadius, float outerRadius, int segments) {
    MeshData m;
    Vec3 n{0, 1, 0};
    for (int s = 0; s <= segments; ++s) {
        float a = math::kTwoPi * static_cast<float>(s) / static_cast<float>(segments);
        Vec3 dir(std::cos(a), 0, -std::sin(a));
        addVertex(m, dir * innerRadius, n);
        addVertex(m, dir * outerRadius, n);
    }
    for (uint32_t s = 0; s < static_cast<uint32_t>(segments); ++s) {
        uint32_t i = s * 2;  // inner, outer, next inner, next outer
        addQuad(m, i, i + 1, i + 3, i + 2);
    }
    return m;
}

MeshData makeTorus(float majorRadius, float minorRadius, int segments, int sides) {
    MeshData m;
    for (int s = 0; s <= segments; ++s) {
        float a = math::kTwoPi * static_cast<float>(s) / static_cast<float>(segments);
        Vec3 radial(std::cos(a), 0, -std::sin(a));
        for (int k = 0; k <= sides; ++k) {
            float b = math::kTwoPi * static_cast<float>(k) / static_cast<float>(sides);
            Vec3 n = radial * std::cos(b) + Vec3(0, std::sin(b), 0);
            addVertex(m, radial * majorRadius + n * minorRadius, n);
        }
    }
    uint32_t stride = static_cast<uint32_t>(sides + 1);
    for (uint32_t s = 0; s < static_cast<uint32_t>(segments); ++s)
        for (uint32_t k = 0; k < static_cast<uint32_t>(sides); ++k) {
            uint32_t i0 = s * stride + k, i1 = i0 + stride;
            addQuad(m, i0, i1, i1 + 1, i0 + 1);
        }
    return m;
}

void appendMesh(MeshData& dst, const MeshData& src, const math::Transform& t) {
    uint32_t base = static_cast<uint32_t>(dst.positions.size());
    for (size_t i = 0; i < src.positions.size(); ++i) {
        dst.positions.push_back(t.transformPoint(src.positions[i]));
        dst.normals.push_back(math::safeNormalize(t.transformDirection(src.normals[i]), math::kUp));
    }
    for (uint32_t i : src.indices) dst.indices.push_back(base + i);
}

MeshData makeSphere(float radius, int segments, int rings) {
    MeshData m;
    std::vector<ProfilePoint> profile;
    for (int r = 0; r <= rings; ++r) {
        float a = -math::kHalfPi + math::kPi * static_cast<float>(r) / static_cast<float>(rings);
        float c = std::cos(a), s = std::sin(a);
        profile.push_back({radius * c, radius * s, {c, s}});
    }
    revolve(m, profile, segments);
    return m;
}

} // namespace render
