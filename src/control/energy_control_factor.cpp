#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/energy_control_factor.h>

EnergyFactor::EnergyFactor(Key p_i, Key vel_i, Key input_i, Key dt_i, gtsam::Vector3 drag_k,
                           gtsam::Vector3 model_params, const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, input_i, dt_i), drag_k_(drag_k), model_params_(model_params) {}

Vector EnergyFactor::evaluateError(const gtsam::Pose3& /*pose*/, const gtsam::Vector3& /*velocity*/,
                                   const gtsam::Vector4& /*control_input*/, double /*dt*/,
                                   gtsam::OptionalMatrixType /*H1*/,
                                   gtsam::OptionalMatrixType /*H2*/,
                                   gtsam::OptionalMatrixType /*H3*/,
                                   gtsam::OptionalMatrixType /*H4*/) const {
    // This legacy factor is retained for API compatibility but has no implemented model yet.
    return gtsam::Vector1::Zero();
}
