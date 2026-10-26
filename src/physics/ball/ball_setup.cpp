#include "physics/ball/ball_setup.h"

#include "core/config.h"

#include <algorithm>

namespace physics {

SphereBody readBallBody(const core::Config& ball, const core::Config& scene) {
    SphereBody b;
    b.mass = scene.getFloat("ball.mass", 0.160f);
    b.radius = scene.getFloat("ball.circumference", 0.2265f) / math::kTwoPi;
    b.dragCoefficient = ball.getFloat("drag.coefficient", 0.45f);
    b.inertiaFactor = ball.getFloat("inertiaFactor", b.inertiaFactor);
    return b;
}

SolverSettings readSolverSettings(const core::Config& ball) {
    SolverSettings s;
    s.maxTravelPerSubstep = ball.getFloat("solver.maxTravelPerSubstep", s.maxTravelPerSubstep);
    s.maxSubsteps = std::max(1, ball.getInt("solver.maxSubsteps", s.maxSubsteps));
    s.restSpeed = ball.getFloat("solver.restSpeed", s.restSpeed);
    return s;
}

void configureBallSolver(SphereSolver& solver, const core::Config& ball, const core::Config& scene,
                         const WorldSettings& world) {
    solver.configure(readBallBody(ball, scene), world, readSolverSettings(ball));
    solver.colliders().load(ball);
}

} // namespace physics
