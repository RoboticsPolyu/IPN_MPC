close all;
clear all;

% MPC Based on GNSS positioning
[timestamp_vicon x y z qx qy qz wq] = textread('pose.txt','%f %f %f %f %f %f %f %f ');

[timestamp_imu gx gy gz ax ay az] = textread('imu_data.txt','%f %f %f %f %f %f %f ');

plot(x, y, 'r');

figure;
plot((timestamp_imu(2:1:end) - timestamp_imu(1:1:end-1))* 1e-6);
title('IMU timestamp cycle');
xlabel('Index'); ylabel('ms');