#include "env_sensors_sim/Lidar.h"

namespace Sensors_Sim
{
    template<typename T>
    Features Lidar<T>::Measurement(T &env, gtsam::Pose3 &body_pose)
    {
        Features features = env.GetMeasurements();
        Features features_inview;

        for (auto iter = features.begin(); iter != features.end(); iter++)
        {
            double d_b_f = std::sqrt((iter->x - body_pose.x()) * (iter->x - body_pose.x()) +
                                     (iter->y - body_pose.y()) * (iter->y - body_pose.y()) +
                                     (iter->z - body_pose.z()) * (iter->z - body_pose.z()));


            if (d_b_f <= range_ && d_b_f >= range_min_)
            {
                Feature lidar_point;

                gtsam::Vector3 l_body_w((*iter).x - body_pose.x(), (*iter).y - body_pose.y(), (*iter).z - body_pose.z());
                gtsam::Vector3 l_body_body = body_pose.rotation().unrotate(l_body_w);
                
                lidar_point.x  = l_body_body.x();
                lidar_point.y  = l_body_body.y();
                lidar_point.z  = l_body_body.z();

                features_inview.push_back(lidar_point);
            }
        }

        return features_inview;
    }

    template class Lidar<Env_Sim::Landmarks>;
}