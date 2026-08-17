#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/simulation/landmarks.h>
#include <random>

namespace Env_Sim {

Landmarks::Landmarks(float map_width, float map_depth, float map_height, float center_x,
                     float center_y, float center_z, std::size_t landmark_count)
    : map_x_(map_width), map_y_(map_depth), map_z_(map_height), map_center_x_(center_x),
      map_center_y_(center_y), map_center_z_(center_z), landmark_count_(landmark_count) {
    std::mt19937 generator{std::random_device{}()};
    std::uniform_real_distribution<double> unit_distribution{-0.5, 0.5};
    features_.reserve(landmark_count_);

    for (std::size_t index = 0; index < landmark_count_; ++index) {
        Feature feature{};
        feature.feature_id = static_cast<uint32_t>(index);
        feature.x = unit_distribution(generator) * map_x_ + map_center_x_;
        feature.y = unit_distribution(generator) * map_y_ + map_center_y_;
        feature.z = unit_distribution(generator) * map_z_ + map_center_z_;

        features_.push_back(feature);

        IPN_LOG_DEBUG << "Landmark generated: id=" << feature.feature_id << ", position_m=["
                      << feature.x << ", " << feature.y << ", " << feature.z << ']';
    }
}

} // namespace Env_Sim
