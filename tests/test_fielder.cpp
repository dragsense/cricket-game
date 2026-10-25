// Whole-fielder checks: runs the real fielder from config and measures how the
// body moves (side-to-side sway while running).

#include "agents/fielder.h"
#include "core/config.h"
#include "rig/skeleton_def.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

namespace {

struct Range {
    float lo = 1e9f, hi = -1e9f;
    void add(float v) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    float span() const { return hi - lo; }
};

} // namespace

TEST_CASE("running fielder keeps the body and feet in their lanes") {
    core::Config skeleton, joints, locomotion;
    REQUIRE(skeleton.load(CRICKET_CONFIG_DIR "/fielder_skeleton.json"));
    REQUIRE(joints.load(CRICKET_CONFIG_DIR "/fielder_joints.json"));
    REQUIRE(locomotion.load(CRICKET_CONFIG_DIR "/locomotion.json"));
    rig::SkeletonDef def;
    REQUIRE(rig::loadSkeletonDef(skeleton, def));
    agents::Fielder fielder;
    REQUIRE(fielder.init(def, def.height, joints, locomotion));

    fielder.setGait(motion::Gait::Run);
    fielder.setMoveInput(math::Vec2(0.0f, 1.0f), true, 0.0f);
    const double dt = 1.0 / 120.0;
    Range pelvis, footL, footR;
    for (int i = 0; i < 4 * 120; ++i) {
        fielder.tick(dt);
        if (i < 2 * 120) continue;  // up to speed first
        const math::Transform toModel = math::inverse(fielder.root());
        pelvis.add(fielder.pose().locals[0].position.x);  // the pelvis is the root joint (character space)
        footL.add(toModel.transformPoint(fielder.targets().footL).x);
        footR.add(toModel.transformPoint(fielder.targets().footR).x);
    }
    MESSAGE("pelvis x [", pelvis.lo, ", ", pelvis.hi, "]  footL x [", footL.lo, ", ", footL.hi, "]  footR x [", footR.lo,
            ", ", footR.hi, "]");
    CHECK(pelvis.span() < 0.04f);  // running CoM moves about 1-2 cm sideways
    CHECK(footL.lo > -0.02f);      // feet never cross the midline (+X is left)
    CHECK(footR.hi < 0.02f);
}
