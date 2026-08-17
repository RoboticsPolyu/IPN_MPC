#pragma once

#include <Eigen/Core>
#include <ipn_mpc/simulation/types.h>

namespace Env_Sim {
class Landmarks {
  public:
    Landmarks(float map_width, float map_depth, float map_height, float center_x, float center_y,
              float center_z, std::size_t landmark_count);

    const Features& measurements() const noexcept {
        return features_;
    }

  private:
    Features features_;

    float map_x_;
    float map_y_;
    float map_z_;

    float map_center_x_;
    float map_center_y_;
    float map_center_z_;

    std::size_t landmark_count_;
};
} // namespace Env_Sim
