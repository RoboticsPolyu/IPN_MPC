[p1 p2 p3 p4 p5 p6] = textread('/Users/yyqing/Self/wheel_imu_calibration/build/est_pose.txt','%f %f %f %f %f %f');
figure; plot(p1, p2, 'black.', p4, p5, 'r*');
for i = 1:size(p1)
    line([p1(i) p4(i)],[p2(i) p5(i)]);
end
title('wio bRo calibraed');
xlabel('x');
ylabel('y');
legend('groud truth','estimated');