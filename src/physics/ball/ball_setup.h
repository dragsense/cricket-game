#pragma once
// The cricket ball's settings for the generic sphere solver: size and mass
// from config/scene.json "ball" (one source for the visual and the physics),
// drag, substeps and colliders from config/ball.json. Values:
// docs/BALL_PHYSICS.md.

#include "physics/sim/sphere_solver.h"

namespace core {
class Config;
}

namespace physics {

SphereBody readBallBody(const core::Config& ball, const core::Config& scene);
SolverSettings readSolverSettings(const core::Config& ball);
// Configures `solver` for the ball and loads its colliders. Forces are left as
// they are. Load time.
void configureBallSolver(SphereSolver& solver, const core::Config& ball, const core::Config& scene,
                         const WorldSettings& world);

} // namespace physics
