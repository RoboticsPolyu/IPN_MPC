/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <trajectory_generator/Trajectory_generator.h>
#include <yaml-cpp/yaml.h>


using namespace std;

#define FLIGHT_ALTITUDE_LIMIT 1.0f

geometry_msgs::PoseStamped current_pose;
bool rev_pose = false;

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // std::cout << "Rev pose msg. " << std::endl;
    rev_pose = true;
    current_pose = *msg;
}

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "TrajPx4ctrlNode");
    ros::NodeHandle nh;

    YAML::Node traj_config   = YAML::LoadFile("../config/traj.yaml");  
    std::string file_name    = traj_config["file_name"].as<std::string>();
	const float control_freq = traj_config["control_freq"].as<float>();

    ros::Publisher  local_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, uav_pose_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(control_freq);

    // load trajectory 
    std::ifstream traj_file;
    traj_file.open(file_name);
    traj_state trj_state;
    std::vector<traj_state> trajs;
    double t, p_x, p_y, p_z, q_w, q_x, q_y, q_z, 
              v_x, v_y, v_z, w_x, w_y, w_z, 
              a_x, a_y, a_z, a_rot_x, a_rot_y, a_rot_z, 
              u1,  u2,  u3,  u4;
    while(traj_file >> t >> p_x >> p_y >> p_z >> q_w >> q_x >> q_y >> q_z >> 
                           v_x >> v_y >> v_z >> w_x >> w_y >> w_z >> 
                           a_x >> a_y >> a_z >> a_rot_x >> a_rot_y >> a_rot_z >> 
                           u1  >> u2  >> u3  >> u4)
    {
        trj_state.t        = t;
        trj_state.pos      = gtsam::Vector3(p_x, p_y, p_z);
        trj_state.rotation = gtsam::Quaternion(q_w, q_x, q_y, q_z);
        trj_state.vel      = gtsam::Vector3(v_x, v_y, v_z);
        trj_state.acc      = gtsam::Vector3(a_x, a_y, a_z);

        std::cout << p_x << " " << p_y << " " << p_z << std::endl;

        trj_state.angular_speed = gtsam::Vector3(w_x, w_y, w_z);
        trajs.push_back(trj_state);
    }

    std::cout << "Traj size: [" << trajs.size() << " ], " << file_name << std::endl;
    quadrotor_msgs::PositionCommand msg;
    if(trajs.size() <= 0)
    {
	    return;
    }
    
    float duration = 10.0;
    while(rev_pose == false)
    {
        ros::spinOnce();
        rate.sleep();
        std::cout << "waiting pose msg!" << std::endl;
    }

    geometry_msgs::PoseStamped init_pose = current_pose;

    float velocity = 0.2; 
    trj_state  = trajs[0];
    float dis_x = trj_state.pos.x() - init_pose.pose.position.x;
    float dis_y = trj_state.pos.y() - init_pose.pose.position.y;
    float dis_z = trj_state.pos.z() - init_pose.pose.position.z;

    for(int i=0; i <= duration * control_freq; i++)
    {
        msg.position.x     = init_pose.pose.position.x + dis_x* (float)i / duration / control_freq;
        msg.position.y     = init_pose.pose.position.y + dis_y* (float)i / duration / control_freq;
        msg.position.z     = init_pose.pose.position.z + dis_z* (float)i / duration / control_freq;

        msg.velocity.x     = dis_x / duration;
        msg.velocity.y     = dis_y / duration;
        msg.velocity.z     = dis_z / duration;

        msg.acceleration.x = 0;
        msg.acceleration.y = 0;
        msg.acceleration.z = 0;

        msg.kx[0]          = 0;
        msg.kx[1]          = 0;
        msg.kx[2]          = 0;
        
        msg.yaw            = fromQuaternion2yaw(trj_state.rotation);

        gtsam::Vector3 rotv = gtsam::Rot3::Logmap(gtsam::Rot3(trj_state.rotation));
        
        msg.kv[0] = rotv.x();
        msg.kv[1] = rotv.y();
        msg.kv[2] = rotv.z();

        std::cout << "des_p: [" << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << " ]" << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        
    }

	ros::Duration(1.0).sleep();

    int index = 0;
    while(index < trajs.size() && ros::ok())
    {
        trj_state          = trajs[index];
        msg.position.x     = trj_state.pos.x();
        msg.position.y     = trj_state.pos.y();
        msg.position.z     = trj_state.pos.z();

        msg.velocity.x     = trj_state.vel.x();
        msg.velocity.y     = trj_state.vel.y();
        msg.velocity.z     = trj_state.vel.z();

        msg.acceleration.x = trj_state.acc.x();
        msg.acceleration.y = trj_state.acc.y();
        msg.acceleration.z = trj_state.acc.z();
        
        msg.kx[0]          = trj_state.angular_speed.x();
        msg.kx[1]          = trj_state.angular_speed.y();
        msg.kx[2]          = trj_state.angular_speed.z();

        msg.yaw            = fromQuaternion2yaw(trj_state.rotation);

        gtsam::Vector3 rotv = gtsam::Rot3::Logmap(gtsam::Rot3(trj_state.rotation));
       
        msg.kv[0] = rotv.x();
        msg.kv[1] = rotv.y();
        msg.kv[2] = rotv.z();

        std::cout << "des_p:" << trj_state.pos.transpose() << std::endl;
        local_cmd_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        index++;
    }

    return 0;
}
