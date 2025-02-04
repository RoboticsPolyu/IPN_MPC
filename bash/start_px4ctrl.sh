#!/bin/bash

echo -e " ${RED} [ 7. Running px4ctrl nodes ] ${NC}" 
source /home/amov/Fast250/devel/setup.bash

roslaunch jpcm run_ctrl.launch & sleep 5;

wait;
