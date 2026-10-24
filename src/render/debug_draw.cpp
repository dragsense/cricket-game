#include "render/debug_draw.h"

namespace render {

using math::Vec3;

DebugDraw::DebugDraw(size_t maxLines) : vertices_(maxLines * 2) {}

void DebugDraw::line(Vec3 a, Vec3 b, Color color) {
    if (count_ + 2 > vertices_.size()) {
        ++dropped_;
        return;
    }
    vertices_[count_++] = {a, color};
    vertices_[count_++] = {b, color};
}

void DebugDraw::circle(Vec3 centre, Vec3 normal, float radius, Color color, int segments) {
    Vec3 n = math::safeNormalize(normal, math::kUp);
    // Any two axes perpendicular to n.
    Vec3 u = math::safeNormalize(math::cross(std::fabs(n.y) < 0.99f ? math::kUp : math::kRight, n));
    Vec3 v = math::cross(n, u);
    Vec3 prev = centre + u * radius;
    for (int i = 1; i <= segments; ++i) {
        float a = math::kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        Vec3 p = centre + (u * std::cos(a) + v * std::sin(a)) * radius;
        line(prev, p, color);
        prev = p;
    }
}

void DebugDraw::sphere(Vec3 centre, float radius, Color color, int segments) {
    circle(centre, math::kRight, radius, color, segments);
    circle(centre, math::kUp, radius, color, segments);
    circle(centre, math::kForward, radius, color, segments);
}

void DebugDraw::axes(const math::Transform& t, float size) {
    line(t.position, t.position + t.rotation * Vec3(size, 0, 0), colors::kRed);
    line(t.position, t.position + t.rotation * Vec3(0, size, 0), colors::kGreen);
    line(t.position, t.position + t.rotation * Vec3(0, 0, size), colors::kBlue);
}

void DebugDraw::box(Vec3 c, Vec3 h, Color color) {
    Vec3 p[8];
    for (int i = 0; i < 8; ++i)
        p[i] = c + Vec3((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z);
    const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                              {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges) line(p[e[0]], p[e[1]], color);
}

void DebugDraw::grid(Vec3 centre, float halfSize, float spacing, Color color) {
    int n = static_cast<int>(halfSize / spacing);
    for (int i = -n; i <= n; ++i) {
        float o = static_cast<float>(i) * spacing;
        line(centre + Vec3(o, 0, -halfSize), centre + Vec3(o, 0, halfSize), color);
        line(centre + Vec3(-halfSize, 0, o), centre + Vec3(halfSize, 0, o), color);
    }
}

} // namespace render
