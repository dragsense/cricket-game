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

bool Rig::localToModel(const Pose& pose, std::vector<math::Mat4>& out) const {
    const size_t n = parents_.size();
    if (!valid() || pose.locals.size() != n || out.size() != n) return false;

    // AoS -> SoA, four joints per lane group (padding lanes get identity).
    namespace om = ozz::math;
    for (size_t g = 0; g < impl_->soaLocals.size(); ++g) {
        math::Transform t[4];
        for (size_t l = 0; l < 4; ++l)
            if (g * 4 + l < n) t[l] = pose.locals[g * 4 + l];
        om::SoaTransform& s = impl_->soaLocals[g];
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

    ozz::animation::LocalToModelJob job;
    job.skeleton = impl_->skeleton.get();
    job.input = ozz::make_span(impl_->soaLocals);
    job.output = ozz::make_span(impl_->models);
    if (!job.Run()) return false;

    // ozz Float4x4 is column-major like math::Mat4.
    for (size_t i = 0; i < n; ++i)
        for (int c = 0; c < 4; ++c) om::StorePtrU(impl_->models[i].cols[c], &out[i][c][0]);
    return true;
}

} // namespace rig
