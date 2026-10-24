#pragma once
// Debug drawing for a rigged character: bones as lines, joints as spheres,
// per-joint local axes and the whole-body centre of mass.

#include "math/math.h"
#include "render/debug_draw.h"
#include "rig/rig.h"

#include <vector>

namespace debug {

struct SkeletonDrawFlags {
    bool bones = true;
    bool joints = true;
    bool axes = true;  // X red, Y green, Z blue
    bool centreOfMass = true;
};

// `models` are model-space joint matrices; `root` places the model in the world.
void drawSkeleton(render::DebugDraw& dd, const rig::SkeletonDef& def, const rig::Rig& rig,
                  const std::vector<math::Mat4>& models, const math::Transform& root, const SkeletonDrawFlags& flags);

// Mass-weighted centre of the bone segments (joint to child joint or tip), world space.
math::Vec3 centreOfMass(const rig::SkeletonDef& def, const rig::Rig& rig, const std::vector<math::Mat4>& models,
                        const math::Transform& root);

} // namespace debug
