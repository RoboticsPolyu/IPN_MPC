close all;
clear all;
[p1 p2 p3 p4 p5 p6] = textread('/Users/yyqing/Self/wheel_imu_calibration/build/est_pose.txt','%f %f %f %f %f %f');
[timestamp q1 q2 q3 q4 t1 t2 t3] = textread('/Users/yyqing/Documents/VIO/2IMU/2CODE/vio_data_simulation/bin/imu_int_pose_noise.txt','%f %f %f %f %f %f %f %f');
% [timestamp q1 q2 q3 q4 t1 t2 t3] = textread('/Users/yyqing/Documents/VIO/2IMU/2CODE/vio_data_simulation/bin/imu_int_pose_noise.txt','%f %f %f %f %f %f %f %f');
figure; plot(p1, p2, 'black.', p4, p5, 'r*');
hold on;
plot(t1(1:10:end), t2(1:10:end), 'green.');

for i = 1:size(p1)
    line([p1(i) p4(i)],[p2(i) p5(i)]);
end
% 
% for i = 1:size(p1)
%     line([p1(i) t1((i-1)*10+1)],[p2(i) t2((i-1)*10+1)]);
% end

title('wio bRo calibraed');
xlabel('x');
ylabel('y');
legend('groud truth','estimated','imu int');