#pragma once
// Translation / rotation / scale transform. Bones will use this.
//
// (parent * child) maps child-local space into parent space, matching the
// matrices: (a * b).toMat4() == a.toMat4() * b.toMat4() for uniform scale.
// Non-uniform scale combined with rotation produces shear, which a TRS cannot
// represent, so keep bone scales uniform.
//
// Included at the end of math/math.h; include that instead.

namespace math {

struct Transform {
    Vec3 position{0.0f};
    Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 scale{1.0f};

    static Transform identity() { return {}; }

    Vec3 transformPoint(Vec3 p) const { return position + rotation * (scale * p); }
    Vec3 transformVector(Vec3 v) const { return rotation * (scale * v); }
    Vec3 transformDirection(Vec3 d) const { return rotation * d; }

    Transform operator*(const Transform& child) const {
        Transform r;
        r.position = transformPoint(child.position);
        r.rotation = rotation * child.rotation;
        r.scale = scale * child.scale;
        return r;
    }

    Mat4 toMat4() const { return trs(position, rotation, scale); }
};

inline Transform inverse(const Transform& t) {
    Transform r;
    r.rotation = glm::conjugate(glm::normalize(t.rotation));
    r.scale = 1.0f / t.scale;
    r.position = r.scale * (r.rotation * -t.position);
    return r;
}

// Rigid matrix (rotation + translation, uniform scale) to a Transform.
inline Transform toTransform(const Mat4& m) {
    Transform r;
    r.position = Vec3(m[3]);
    float s = glm::length(Vec3(m[0]));
    r.scale = Vec3(s);
    r.rotation = glm::normalize(glm::quat_cast(Mat3(m) * (1.0f / s)));
    return r;
}

inline Transform lerp(const Transform& a, const Transform& b, float t) {
    return {lerp(a.position, b.position, t), glm::slerp(a.rotation, b.rotation, t), lerp(a.scale, b.scale, t)};
}

} // namespace math
