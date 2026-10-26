#pragma once
// Milestone 0 test scene: field, pitch, creases, stumps and human-sized
// capsules at real-world scale (1 unit = 1 metre), built from code with
// dimensions read from config/scene.json.

#include "core/config.h"
#include "game/world.h"
#include "match/wickets.h"

#include <array>

namespace game {

struct TestScene {
    entt::entity probe = entt::null;  // follows the sim probe
    entt::entity ball = entt::null;   // static for now; ball physics is M2
    // Stump and bail visuals, in match::WicketLayout piece order; they follow
    // the Jolt bodies (mesh origin at the cylinder's base).
    std::array<entt::entity, match::kWicketPieces> wicketPieces{};
    std::vector<entt::entity> entities;  // everything the scene created
    float pitchLength = 20.12f;       // stump to stump
};

TestScene buildTestScene(World& world, render::Renderer& renderer, const core::Config& scene,
                         const match::WicketLayout& wickets);
// Visual pose of a wicket piece whose body (centre of mass) is at `body`.
math::Transform wicketPieceVisual(const match::WicketLayout& wickets, int piece, const math::Transform& body);
// Removes everything buildTestScene created (used when config/scene.json changes).
void destroyTestScene(World& world, render::Renderer& renderer, TestScene& scene);

} // namespace game
