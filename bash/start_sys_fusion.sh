#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e " ${RED} [ This is a system startup program created by Peiwen peiwen1.yang@connect.polyu.hk] ${NC}" 
echo -e " ${RED} [ 1. Running VICON ] ${NC}" 
source /opt/ros/noetic/setup.bash
source /home/amov/Vicon/devel/setup.bash
roslaunch vrpn_client_ros quad13.launch & sleep 1; 


# echo -e " ${RED} [ 2. Running Xsens_mti ] ${NC}" 
# source /home/amov/ws_livox/devel/setup.bash
# roslaunch xsens_mti_driver xsens_mti_node.launch & sleep 1;


echo -e " ${RED} [ 3. Running mavros ] ${NC}" 
source /home/amov/Mav/devel/setup.bash
# source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch & sleep 1;


echo -e " ${RED} [ 4. Running rs_t265 ] ${NC}" 
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_t265.launch & sleep 1;


echo -e " ${RED} [ 5. Running Orin_board_can nodes ] ${NC}" 
source /home/amov/IPN_MPC/build/devel/setup.bash
cd /home/amov/IPN_MPC/build
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
./devel/lib/IPN_MPC/Orin_board_can & sleep 1;


echo -e " ${RED} [ 6. Running mavcmd ] ${NC}" 
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 
sleep 3
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
echo -e " ${RED} [ Remember to check mavros/imu/data !!! ] ${NC}"


echo -e " ${RED} [ 7. Running px4ctrl nodes ] ${NC}" 
source /home/amov/Fast250/devel/setup.bash

roslaunch jpcm vicon.launch & sleep 1;
# roslaunch px4ctrl run_ctrl.launch & sleep 1;

echo -e " ${RED} [ Running fusion nodes ] ${NC}" 
source /home/amov/Fast250/devel/setup.bash

roslaunch jpcm run_fusion.launch & sleep 1;

wait;

