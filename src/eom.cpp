#include "eom.hpp"
#include "wing.hpp"

StateDerivative equationOfMotion(
    double t,
    const State& state,
    const std::vector<Wing>& wings,
    std::vector<SingleWingVectors>& wing_outputs
) {
    // Resize output vector
    wing_outputs.resize(wings.size());

    // Sum forces from all wings
    Vec3 accel = Vec3::Zero();
    for (size_t i = 0; i < wings.size(); ++i) {
        accel += wings[i].computeForce(t, state.vel, wing_outputs[i]);
    }

    // Add gravity
    accel -= Vec3(0.0, 0.0, 1.0);

    return StateDerivative(state.vel, accel);
}
