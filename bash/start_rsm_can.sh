#!/bin/bash

source /home/amov/IPN_MPC/build/devel/setup.bash

roscore &

sudo ip link set can0 type can bitrate 500000

sudo ip link set up can0

cd /home/amov/IPN_MPC/build

./devel/lib/IPN_MPC/Orin_rsm_can


