#include "control/Energy_control_factor.h"

EnergyFactor::EnergyFactor(Key p_i, Key vel_i, Key input_i, Key dt_i, gtsam::Vector3 drag_k, 
  gtsam::Vector3 model_params, const SharedNoiseModel &model):
  Base(model, p_i, vel_i, input_i, dt_i),
  drag_k_(drag_k),
  model_params_(model_params){};

Vector EnergyFactor::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector4 &input_i, double dt,
                      boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                      boost::optional<Matrix &> H3, boost::optional<Matrix &> H4) const
{
  gtsam::Vector1 err;

  return err;
}