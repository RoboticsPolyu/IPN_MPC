#pragma once

#include <cstddef>
#include <gtsam/base/Matrix.h>

namespace UAVFactor {

struct LqrTerminalWeight {
    gtsam::Matrix information;
    gtsam::Matrix feedback_gain;
    double riccati_residual{0.0};
    double closed_loop_spectral_radius{0.0};
    std::size_t iterations{0};
};

/** Compute the stabilizing DARE solution for the local error state [p, theta, v]. */
LqrTerminalWeight computeAccelerationGyroTerminalWeight(
    double dt, const gtsam::Matrix9& state_information,
    const gtsam::Matrix6& control_information, double tolerance = 1.0e-12,
    std::size_t maximum_iterations = 10000);

} // namespace UAVFactor
