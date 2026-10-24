#pragma once
// Immediate-mode debug lines (bones, IK targets, velocities...). Callers add
// lines every frame; the renderer uploads and clears them. Storage is
// allocated once at the configured capacity; extra lines are dropped.

#include "math/math.h"

#include <cstdint>
#include <vector>

namespace render {

// Packed 0xAABBGGRR, so it uploads directly as 4 normalized bytes (R,G,B,A).
using Color = uint32_t;
constexpr Color rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return static_cast<Color>(r) | (static_cast<Color>(g) << 8) | (static_cast<Color>(b) << 16) |
           (static_cast<Color>(a) << 24);
}
namespace colors {
inline constexpr Color kRed = rgba(230, 60, 60);
inline constexpr Color kGreen = rgba(60, 210, 80);
inline constexpr Color kBlue = rgba(70, 120, 240);
inline constexpr Color kYellow = rgba(240, 220, 60);
inline constexpr Color kWhite = rgba(255, 255, 255);
inline constexpr Color kGrey = rgba(140, 140, 140);
inline constexpr Color kCyan = rgba(60, 220, 230);
inline constexpr Color kMagenta = rgba(220, 70, 220);
} // namespace colors

struct DebugVertex {
    math::Vec3 position;
    Color color;
};
static_assert(sizeof(DebugVertex) == 16);

class DebugDraw {
public:
    explicit DebugDraw(size_t maxLines);

    void line(math::Vec3 a, math::Vec3 b, Color color);
    void circle(math::Vec3 centre, math::Vec3 normal, float radius, Color color, int segments = 24);
    // Three orthogonal circles.
    void sphere(math::Vec3 centre, float radius, Color color, int segments = 16);
    // X red, Y green, Z blue.
    void axes(const math::Transform& t, float size);
    void box(math::Vec3 centre, math::Vec3 halfExtents, Color color);
    // Square grid on the XZ plane centred at `centre`.
    void grid(math::Vec3 centre, float halfSize, float spacing, Color color);

    void clear() { count_ = 0; }
    const DebugVertex* vertices() const { return vertices_.data(); }
    size_t vertexCount() const { return count_; }
    size_t capacityVertices() const { return vertices_.size(); }
    size_t droppedLines() const { return dropped_; }

private:
    std::vector<DebugVertex> vertices_;  // sized once in the constructor
    size_t count_ = 0;
    size_t dropped_ = 0;
};

} // namespace render
