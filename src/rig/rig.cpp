#include "rig/rig.h"

#include "core/log.h"

#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/animation/runtime/local_to_model_job.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/maths/simd_math.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/span.h>

namespace rig {

struct Rig::Impl {
    ozz::unique_ptr<ozz::animation::Skeleton> skeleton;
    // Scratch buffers, sized once in build(); reused by localToModel().
    mutable ozz::vector<ozz::math::SoaTransform> soaLocals;
    mutable ozz::vector<ozz::math::Float4x4> models;
};

namespace {

using RawJoint = ozz::animation::offline::RawSkeleton::Joint;

void addChildren(const SkeletonDef& def, int parent, float height, RawJoint::Children& out) {
    for (size_t i = 0; i < def.bones.size(); ++i) {
        const BoneDef& b = def.bones[i];
        if (b.parent != parent) continue;
        RawJoint j;
        j.name = b.name.c_str();
        math::Vec3 t = b.offset * height;
        j.transform.translation = ozz::math::Float3(t.x, t.y, t.z);
        j.transform.rotation = ozz::math::Quaternion::identity();
        j.transform.scale = ozz::math::Float3::one();
        addChildren(def, static_cast<int>(i), height, j.children);
        out.push_back(std::move(j));
    }
}

} // namespace

Rig::Rig() : impl_(std::make_unique<Impl>()) {}
Rig::~Rig() = default;
Rig::Rig(Rig&&) noexcept = default;
Rig& Rig::operator=(Rig&&) noexcept = default;

bool Rig::valid() const { return impl_ && impl_->skeleton != nullptr; }

bool Rig::build(const SkeletonDef& def, float height) {
    ozz::animation::offline::RawSkeleton raw;
    addChildren(def, kNoParent, height, raw.roots);
    if (!raw.Validate() || raw.num_joints() != static_cast<int>(def.bones.size())) {
        LOG_ERROR("rig: invalid skeleton description ({} of {} bones reachable)", raw.num_joints(), def.bones.size());
        return false;
    }
    auto skeleton = ozz::animation::offline::SkeletonBuilder()(raw);
    if (!skeleton) {
        LOG_ERROR("rig: ozz SkeletonBuilder failed");
        return false;
    }

    const int n = skeleton->num_joints();
    height_ = height;
    parents_.assign(static_cast<size_t>(n), -1);
    names_.assign(static_cast<size_t>(n), {});
    defIndex_.assign(static_cast<size_t>(n), -1);
    rest_.locals.assign(static_cast<size_t>(n), math::Transform::identity());
    for (int i = 0; i < n; ++i) {
        size_t u = static_cast<size_t>(i);
        parents_[u] = skeleton->joint_parents()[u];
        names_[u] = skeleton->joint_names()[u];
        defIndex_[u] = def.find(names_[u]);
        rest_.locals[u].position = def.bones[static_cast<size_t>(defIndex_[u])].offset * height;
    }
    legLength_ = def.legLengthRatio() * height;
    // Each joint's descendants follow it in one run (depth-first order).
    subtreeEnd_.assign(static_cast<size_t>(n), n);
    for (int i = 0; i < n; ++i) {
        int end = i + 1;
        while (end < n) {
            int p = parents_[static_cast<size_t>(end)];
            while (p > i) p = parents_[static_cast<size_t>(p)];
            if (p != i) break;
            ++end;
        }
        subtreeEnd_[static_cast<size_t>(i)] = end;
    }

    impl_->soaLocals.resize(static_cast<size_t>(skeleton->num_soa_joints()));
    impl_->models.resize(static_cast<size_t>(n));
    impl_->skeleton = std::move(skeleton);
    return true;
}

int Rig::findJoint(const std::string& name) const {
    for (size_t i = 0; i < names_.size(); ++i)
        if (names_[i] == name) return static_cast<int>(i);
    return -1;
}

bool Rig::modelToLocal(const std::vector<math::Mat4>& models, Pose& pose) const {
    const size_t n = parents_.size();
    if (!valid() || models.size() != n || pose.locals.size() != n) return false;
    for (size_t j = 0; j < n; ++j) {
        const math::Transform model = math::toTransform(models[j]);
        const int parent = parents_[j];
        pose.locals[j] = parent >= 0 ? math::inverse(math::toTransform(models[static_cast<size_t>(parent)])) * model : model;
    }
    return true;
}

namespace {

// One SoA lane group (four joints from g * 4; padding lanes get identity).
void toSoa(const rig::Pose& pose, size_t n, size_t g, ozz::math::SoaTransform& s) {
    namespace om = ozz::math;
    math::Transform t[4];
    for (size_t l = 0; l < 4; ++l)
        if (g * 4 + l < n) t[l] = pose.locals[g * 4 + l];
    s.translation.x = om::simd_float4::Load(t[0].position.x, t[1].position.x, t[2].position.x, t[3].position.x);
    s.translation.y = om::simd_float4::Load(t[0].position.y, t[1].position.y, t[2].position.y, t[3].position.y);
    s.translation.z = om::simd_float4::Load(t[0].position.z, t[1].position.z, t[2].position.z, t[3].position.z);
    s.rotation.x = om::simd_float4::Load(t[0].rotation.x, t[1].rotation.x, t[2].rotation.x, t[3].rotation.x);
    s.rotation.y = om::simd_float4::Load(t[0].rotation.y, t[1].rotation.y, t[2].rotation.y, t[3].rotation.y);
    s.rotation.z = om::simd_float4::Load(t[0].rotation.z, t[1].rotation.z, t[2].rotation.z, t[3].rotation.z);
    s.rotation.w = om::simd_float4::Load(t[0].rotation.w, t[1].rotation.w, t[2].rotation.w, t[3].rotation.w);
    s.scale.x = om::simd_float4::Load(t[0].scale.x, t[1].scale.x, t[2].scale.x, t[3].scale.x);
    s.scale.y = om::simd_float4::Load(t[0].scale.y, t[1].scale.y, t[2].scale.y, t[3].scale.y);
    s.scale.z = om::simd_float4::Load(t[0].scale.z, t[1].scale.z, t[2].scale.z, t[3].scale.z);
}

} // namespace

bool Rig::localToModel(const Pose& pose, std::vector<math::Mat4>& out) const { return localToModel(pose, out, -1); }

bool Rig::localToModel(const Pose& pose, std::vector<math::Mat4>& out, int from) const {
    const size_t n = parents_.size();
    if (!valid() || pose.locals.size() != n || out.size() != n || from >= static_cast<int>(n)) return false;
    namespace om = ozz::math;
    const bool whole = from < 0;
    const size_t first = whole ? 0 : static_cast<size_t>(from);
    const size_t end = whole ? n : static_cast<size_t>(subtreeEnd_[first]);

    // AoS -> SoA for the lane groups the range touches.
    for (size_t g = first / 4; g * 4 < end; ++g) toSoa(pose, n, g, impl_->soaLocals[g]);

    ozz::animation::LocalToModelJob job;
    job.skeleton = impl_->skeleton.get();
    job.input = ozz::make_span(impl_->soaLocals);
    job.output = ozz::make_span(impl_->models);
    if (!whole) {
        // The range starts from its parent as `out` holds it (ozz reads the
        // parent's model matrix from its output).
        const int parent = parents_[first];
        if (parent >= 0)
            for (int c = 0; c < 4; ++c)
                impl_->models[static_cast<size_t>(parent)].cols[c] = om::simd_float4::LoadPtrU(&out[static_cast<size_t>(parent)][c][0]);
        job.from = from;
        job.to = static_cast<int>(end) - 1;
    }
    if (!job.Run()) return false;

    // ozz Float4x4 is column-major like math::Mat4.
    for (size_t i = first; i < end; ++i)
        for (int c = 0; c < 4; ++c) om::StorePtrU(impl_->models[i].cols[c], &out[i][c][0]);
    return true;
}

} // namespace rig
