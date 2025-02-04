#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <IPN_MPC/INPUT.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/TrustMoments.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


/**
 * Providing a mavros api for any controller based on the input ros message
 * It's middleware
*/

using namespace std;


namespace middleware
{
    enum Ctrl_mode {none = 0, pose = 1, att_thrust = 2, bodyrate_acc = 3, bodyrate_thrust = 4};
    
    class MavBridge
    {
        public:
            typedef boost::shared_ptr<MavBridge> shared_ptr;
            MavBridge(const ros::NodeHandle &nh, int takeoff_alt);

            float RetThrustAccRatio()
            {
                return thrust_acc_ratio_;
            }
            

        private:
            /* Connect the pixhawk */
            bool Connect();

            /* Estimate the thrust(0~1) and acc ratio real time */
            void EstThrustAccRatio();

            /* Control loop */
            void Loop();

            /* Received state from mavros */
            void CtrlInputCb(const IPN_MPC::INPUT::ConstPtr& msg) { internal_ctrl_input_ = *msg; };
            void MavIMU(const sensor_msgs::ImuConstPtr& msg) { mav_imu_ = *msg; };
            void MavstateCb(const mavros_msgs::State::ConstPtr& msg) { current_state_ = *msg; };
            void MavttCb(const mavros_msgs::TrustMoments::ConstPtr& msg) { mav_tt_ = *msg; };


            /* Offboard mode -> armed -> takeoff */
            bool OffboardHover();

            void pubBodyrateCmd(const IPN_MPC::INPUT& input);
            void pubPosYawCmd(const IPN_MPC::INPUT& input);


            ros::NodeHandle    nh_;
            ros::Subscriber    ctrl_input_sub_;
            ros::Subscriber    mav_imu_sub_;
            ros::Subscriber    state_sub_;
            ros::Subscriber    mav_tt_sub_;
            ros::Publisher     bodyrate_pub_;
            ros::Publisher     local_pos_pub_;
            ros::ServiceClient arming_client_;
            ros::ServiceClient land_client_;
            ros::ServiceClient set_mode_client_;

            sensor_msgs::Imu           mav_imu_;
            mavros_msgs::State         current_state_;
            mavros_msgs::TrustMoments  mav_tt_;
            geometry_msgs::PoseStamped current_pose_;
            IPN_MPC::INPUT             internal_ctrl_input_;
            

            int takeoff_altitude_ = 1.0;
            int        ctrl_freq_ = 100;
            float thrust_acc_ratio_ = 1;

    };
}


