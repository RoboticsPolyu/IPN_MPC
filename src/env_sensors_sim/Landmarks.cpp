#include "env_sensors_sim/Landmarks.h"
#include <cstdlib>

namespace Env_Sim
{

    Landmarks::Landmarks(float map_x, float map_y, float map_z, float map_center_x, float map_center_y, float map_center_z, uint32_t feature_size)
        : map_x_(map_x), map_y_(map_y), map_z_(map_z), feature_size_(feature_size), map_center_x_(map_center_x), map_center_y_(map_center_y), map_center_z_(map_center_z)
    {
        Feature feature;
        double random;

        for(uint32_t idx = 0; idx < feature_size; idx++)
        {
            feature.feature_id = idx;
            random = rand() / double(RAND_MAX);
            feature.x = (random - 0.5f)* map_x_ + map_center_x_;

            random = rand() / double(RAND_MAX);
            feature.y = (random - 0.5f)* map_y_ + map_center_y_;

            random = rand() / double(RAND_MAX);
            feature.z = (random - 0.5f)* map_z_ + map_center_z_;

            features_.push_back(feature);

            std::cout << "Landmarks feature: " << feature.feature_id << " , " << feature.x << " , " << feature.y << " , " << feature.z << std::endl;
        }
    };


}