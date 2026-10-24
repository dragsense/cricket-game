#pragma once
// Milestone 0 test scene: field, pitch, creases, stumps and human-sized
// capsules at real-world scale (1 unit = 1 metre), built from code with
// dimensions read from config/scene.json.

#include "core/config.h"
#include "game/world.h"

namespace game {

struct TestScene {
    entt::entity probe = entt::null;  // follows the sim probe
    float pitchLength = 20.12f;       // stump to stump
};

TestScene buildTestScene(World& world, render::Renderer& renderer, const core::Config& scene);

} // namespace game
