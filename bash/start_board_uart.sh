#!/bin/bash

source /home/amov/IPN_MPC/build/devel/setup.bash

roscore &

cd /home/amov/IPN_MPC/build

./devel/lib/IPN_MPC/Orin_rsm_uart &
