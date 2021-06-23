#include <abb_libegm/egm_controller_interface.h>

namespace abb
{
    class egm_wrapper
    {
        public:
            struct abb_egm_wrapper_config
            {
                /* data */
            };
            
            egm_wrapper() = delete;
            egm_wrapper(abb_egm_wrapper_config);

        private:
            std::shared_ptr<abb::egm::EGMControllerInterface> egm_interface;
            boost::asio::io_service io_service;
            boost::thread_group thread_group;

            abb::egm::wrapper::Input input;
            abb::egm::wrapper::CartesianPose initial_pose;
            abb::egm::wrapper::Output output;
    };
}