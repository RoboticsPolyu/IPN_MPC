cd ~/ws_livox
source ~/ws_livox/devel/setup.bash

roslaunch xsens_mti_driver xsens_mti_node.launch & sleep 5;
wait;

