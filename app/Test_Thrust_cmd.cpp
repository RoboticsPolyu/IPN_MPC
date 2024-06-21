/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <Eigen/Dense>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <trajectory_generator/Trajectory_generator.h>
#include <yaml-cpp/yaml.h>


using namespace std;

#define FLIGHT_ALTITUDE 1.0f

mavros_msgs::State         current_state;
geometry_msgs::PoseStamped current_pose;

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int _uav_pose_pinrt = 0;
void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    _uav_pose_pinrt++;
}

void publish_bodyrate_ctrl(const ros::Publisher &pub, const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	pub.publish(msg);
}

int main(int argc, char **argv)
{
    YAML::Node traj_config   = YAML::LoadFile("../config/test.yaml");  
    const float thrust       = traj_config["thrust"].as<float>();
    const float control_freq = traj_config["control_freq"].as<float>();
    const float test_time    = traj_config["test_time"].as<float>();

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber    state_sub       = nh.subscribe    <mavros_msgs::State        >("mavros/state", 10, state_cb);
    ros::Publisher     local_pos_pub   = nh.advertise    <geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool  >("mavros/cmd/arming");
    ros::ServiceClient land_client     = nh.serviceClient<mavros_msgs::CommandTOL   >("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode      >("mavros/set_mode");
    ros::Publisher     ctrl_FCU_pub    = nh.advertise    <mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Subscriber    sub_uav_pose    = nh.subscribe    <geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1, uav_pose_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(control_freq);
    
    // wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Connecting to FCT...");
    }

    geometry_msgs::PoseStamped control;
    control.pose.position.x = 0;
    control.pose.position.y = 0;
    control.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("Send a few setpoints before starting");

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(control);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw       = 0;
    land_cmd.request.latitude  = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude  = 0; 

    ros::Time last_request = ros::Time::now();

    ROS_INFO("Change to offboard mode and arm");
    ROS_INFO("%s\n", current_state.mode.c_str());
    std::cout << "armed: " << current_state.armed << std::endl;

    while(ros::ok() && !current_state.armed)
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO("%s\n", current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
		        else
		        {
		            std::cout << "Try to arm";
		        }
		        last_request = ros::Time::now();
		        std::cout << "current state: " << current_state.armed << std::endl;
	        }
        }
        
        //	std::cout << "Publish control " << std::endl;
        local_pos_pub.publish(control);
        ros::spinOnce();
        rate.sleep();
    }

    int test_index = 100* test_time;
    Controller_Output_t cot;

    ROS_INFO("Start test thrust command");
    while(test_index-- && ros::ok())
    {
        cot.thrust = thrust;
        cot.bodyrates = Eigen::Vector3d::Zero();
        publish_bodyrate_ctrl(ctrl_FCU_pub, cot, ros::Time::now());
	    ros::spinOnce();
        rate.sleep();
    }


    ROS_INFO("tring to land");
    while(!(land_client.call(land_cmd) && land_cmd.response.success))
    {
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
