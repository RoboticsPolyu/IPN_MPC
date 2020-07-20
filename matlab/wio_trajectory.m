close all;
clear all;
[grouth_x grouth_y grouth_z est_x est_y est_z] = textread('../data/est_pose.txt','%f %f %f %f %f %f');
[timestamp q1 q2 q3 q4 int_x int_y int_z] = textread('../data/imu_int_pose_noise.txt','%f %f %f %f %f %f %f %f');

% grouth truth and estimated trajectory
figure; plot(grouth_x, grouth_y, 'black.', est_x, est_y, 'r*');
hold on;
plot(int_x(1:10:end), int_y(1:10:end), 'green.');

for i = 1:size(grouth_x)
    line([grouth_x(i) est_x(i)],[grouth_y(i) est_y(i)]);
end
% 
% for i = 1:size(p1)
%     line([p1(i) t1((i-1)*10+1)],[p2(i) t2((i-1)*10+1)]);
% end

title('wio bRo calibraed');
xlabel('x');
ylabel('y');
legend('groud truth','estimated','imu int');