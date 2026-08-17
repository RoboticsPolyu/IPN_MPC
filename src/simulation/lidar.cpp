#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/simulation/lidar.h>

namespace Sensors_Sim {
template <typename T> Features Lidar<T>::Measurement(T& env, gtsam::Pose3& body_pose) {
    const Features& features = env.measurements();
    Features features_inview;
    features_inview.reserve(features.size());

    const gtsam::Point3 sensor_position = body_pose.translation();
    for (const Feature& feature : features) {
        const gtsam::Point3 landmark_position(feature.x, feature.y, feature.z);
        const double distance = (landmark_position - sensor_position).norm();

        Feature lidar_point = feature;
        lidar_point.type = distance >= range_min_ && distance <= range_ ? L_VIS : L_NONV;
        features_inview.push_back(lidar_point);
    }

    return features_inview;
}

template class Lidar<Env_Sim::Landmarks>;
} // namespace Sensors_Sim
