#include "ik/limb_ik.h"

#include <ozz/animation/runtime/ik_aim_job.h>
#include <ozz/animation/runtime/ik_two_bone_job.h>
#include <ozz/base/maths/simd_math.h>
#include <ozz/base/maths/simd_quaternion.h>

namespace ik {

namespace {

namespace om = ozz::math;

om::Float4x4 toOzz(const math::Mat4& m) {
    om::Float4x4 r;
    for (int c = 0; c < 4; ++c) r.cols[c] = om::simd_float4::LoadPtrU(&m[c][0]);
    return r;
}

om::SimdFloat4 point(math::Vec3 v) { return om::simd_float4::Load(v.x, v.y, v.z, 1.0f); }
om::SimdFloat4 dir(math::Vec3 v) { return om::simd_float4::Load(v.x, v.y, v.z, 0.0f); }

math::Quat fromOzz(const om::SimdQuaternion& q) {
    float f[4];
    om::StorePtrU(q.xyzw, f);
    return math::Quat(f[3], f[0], f[1], f[2]);
}

// Local correction is applied on the right: local = local * correction.
void applyCorrection(rig::Pose& pose, int joint, const om::SimdQuaternion& c) {
    math::Quat& r = pose.locals[static_cast<size_t>(joint)].rotation;
    r = math::normalize(r * fromOzz(c));
}

} // namespace

bool solveTwoBone(const TwoBoneChain& chain, const std::vector<math::Mat4>& models, math::Vec3 target,
                  math::Vec3 pole, float soften, rig::Pose& pose) {
    if (chain.start < 0 || chain.mid < 0 || chain.end < 0) return false;
    const om::Float4x4 start = toOzz(models[static_cast<size_t>(chain.start)]);
    const om::Float4x4 mid = toOzz(models[static_cast<size_t>(chain.mid)]);
    const om::Float4x4 end = toOzz(models[static_cast<size_t>(chain.end)]);

    om::SimdQuaternion startCorrection, midCorrection;
    ozz::animation::IKTwoBoneJob job;
    job.target = point(target);
    job.pole_vector = dir(math::safeNormalize(pole, math::kForward));
    job.mid_axis = dir(chain.midAxis);
    job.soften = soften;
    job.start_joint = &start;
    job.mid_joint = &mid;
    job.end_joint = &end;
    job.start_joint_correction = &startCorrection;
    job.mid_joint_correction = &midCorrection;
    if (!job.Run()) return false;

    applyCorrection(pose, chain.start, startCorrection);
    applyCorrection(pose, chain.mid, midCorrection);
    return true;
}

bool solveAim(int joint, const std::vector<math::Mat4>& models, math::Vec3 target, math::Vec3 forward,
              math::Vec3 up, math::Vec3 pole, float weight, rig::Pose& pose) {
    if (joint < 0) return false;
    const om::Float4x4 m = toOzz(models[static_cast<size_t>(joint)]);

    om::SimdQuaternion correction;
    ozz::animation::IKAimJob job;
    job.target = point(target);
    job.forward = dir(forward);
    job.up = dir(up);
    job.pole_vector = dir(pole);
    job.weight = weight;
    job.joint = &m;
    job.joint_correction = &correction;
    if (!job.Run()) return false;

    applyCorrection(pose, joint, correction);
    return true;
}

} // namespace ik
