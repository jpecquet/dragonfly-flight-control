#include "eom.hpp"
#include "wing.hpp"

StateDerivative equationOfMotion(
    double t,
    const State& state,
    const std::vector<Wing>& wings,
    std::vector<SingleWingVectors>& wing_outputs
) {
    // Extract velocity from state
    Vec3 ub(state[3], state[4], state[5]);

    // Resize output vector
    wing_outputs.resize(wings.size());

    // Sum forces from all wings
    Vec3 a(0.0, 0.0, 0.0);
    for (size_t i = 0; i < wings.size(); ++i) {
        a += wings[i].computeForce(t, ub, wing_outputs[i]);
    }

    // Add gravity
    a -= Vec3(0.0, 0.0, 1.0);

    return {state[3], state[4], state[5], a.x(), a.y(), a.z()};
}
