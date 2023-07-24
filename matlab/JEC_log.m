close all;
clear all;

% MPC Based on GNSS positioning
[x y z rx ry rz vx vy vz ga gy gz in1 m1 m2 m3 x_r y_r z_r rx_r ry_r rz_r vx_r vy_r vz_r ga_r gy_r gz_r in1_r m1_r m2_r m3_r] = textread('../data/JEC_MPC_T1_M0.2_log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');

% Joining Positioning and Control Method
[x2 y2 z2 rx2 ry2 rz2 vx2 vy2 vz2 ga2 gy2 gz2 in12 m12 m22 m32 x_r2 y_r2 z_r2 rx_r2 ry_r2 rz_r2 vx_r2 vy_r2 vz_r2 ga_r2 gy_r2 gz_r2 in1_r2 m1_r2 m2_r2 m3_r2] = textread('../data/JEC_JECM_InfiniteR_Sigma_log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');


[x3 y3 z3 rx3 ry3 rz3 vx3 vy3 vz3 ga3 gy3 gz3 in13 m13 m23 m33 x_r3 y_r3 z_r3 rx_r3 ry_r3, rz_r3 vx_r3 vy_r3 vz_r3 ga_r3 gy_r3 gz_r3 in1_r3 m1_r3 m2_r3 m3_r3] = textread('../data/JEC_MPC_onlyGNSS_NON_5_1__log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');


% index = [1:1:1000]* 0.01;
fig_end = 200;
fig_start = 1;
index = [fig_start:1:fig_end]* 0.01;

figure;
plot(index, vx(fig_start:1:fig_end), 'r'); hold on;
plot(index, vy(fig_start:1:fig_end), 'g'); hold on;
plot(index, vz(fig_start:1:fig_end), 'b');
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('x-axis', 'y-axis', 'z-axis');
title("Real Velocity");

figure;
plot(index, vx(fig_start:1:fig_end) - vx_r(fig_start:1:fig_end), 'r'); hold on;
plot(index, vy(fig_start:1:fig_end) - vy_r(fig_start:1:fig_end), 'g'); hold on;
plot(index, vz(fig_start:1:fig_end) - vz_r(fig_start:1:fig_end), 'b');
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (m)");
legend('x-axis', 'y-axis', 'z-axis');
title("Velocity Control error");


figure;
plot(index, x(fig_start:1:fig_end) - x_r(fig_start:1:fig_end), 'r--', 'linewidth', 1); hold on;
plot(index, x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (m)");
legend('Nominal MPC', 'JPCM-GI');
title("Position X-Axis Control Error");

figure;
plot(index, y(fig_start:1:fig_end) - y_r(fig_start:1:fig_end), 'r--', 'linewidth', 1); hold on;
plot(index, y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (m)");
legend('Nominal MPC', 'JPCM-GI');
title("Position Y-Axis Control Error");

figure;
plot(index, z(fig_start:1:fig_end) - z_r(fig_start:1:fig_end), 'r--', 'linewidth', 1); hold on;
plot(index, z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (m)");
legend('Nominal MPC', 'JPCM-GI');
title("Position Z-Axis Control Error");

figure;
plot(index, check2PI(rx(fig_start:1:fig_end) - rx_r(fig_start:1:fig_end)), 'r--', 'linewidth', 1); hold on;
plot(index, check2PI(rx2(fig_start:1:fig_end) - rx_r2(fig_start:1:fig_end)), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (rad)");
legend('Nominal MPC', 'JPCM-GI');
title("Rotation X-Axis Control Error");

figure;
plot(index, check2PI(ry(fig_start:1:fig_end) - ry_r(fig_start:1:fig_end)), 'r--', 'linewidth', 1); hold on;
plot(index, check2PI(ry2(fig_start:1:fig_end) - ry_r2(fig_start:1:fig_end)), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (rad)");
legend('Nominal MPC', 'JPCM-GI');
title("Rotation Y-Axis Control Error");

figure;
plot(index, check2PI(rz(fig_start:1:fig_end) - rz_r(fig_start:1:fig_end)), 'r--', 'linewidth', 1); hold on;
plot(index, check2PI(rz2(fig_start:1:fig_end) - rz_r2(fig_start:1:fig_end)), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
xlabel("Time (s)");
ylabel("Error (rad)");
legend('Nominal MPC', 'JPCM-GI');
title("Rotation Z-Axis Control Error");


figure;
plot(index, rx2(fig_start:1:fig_end), 'r'); hold on;
plot(index, ry2(fig_start:1:fig_end), 'g'); hold on;
plot(index, rz2(fig_start:1:fig_end), 'b');
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('x-axis', 'y-axis', 'z-axis');
title("Real Ratation");


figure;
plot(index, (x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end)), 'r', 'LineWid',1); hold on;
plot(index, (y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end)), 'g', 'LineWid',1); hold on;
plot(index, (z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end)), 'b', 'LineWid',1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('x-axis', 'y-axis', 'z-axis');
title("Position Control Error Based on JPCM");

figure;
plot(index, check2PI(rx2(fig_start:1:fig_end) - rx_r2(fig_start:1:fig_end)), 'r', 'linewidth', 1); hold on;
plot(index, check2PI(ry2(fig_start:1:fig_end) - ry_r2(fig_start:1:fig_end)), 'g', 'linewidth', 1); hold on;
plot(index, check2PI(rz2(fig_start:1:fig_end) - rz_r2(fig_start:1:fig_end)), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('roll', 'pitch', 'yaw');
title("Ratation Control Error Based on JPCM");

figure;
plot(index, (x(fig_start:1:fig_end) - x_r(fig_start:1:fig_end)), 'r', 'LineWid',1); hold on;
plot(index, (y(fig_start:1:fig_end) - y_r(fig_start:1:fig_end)), 'g', 'LineWid',1); hold on;
plot(index, (z(fig_start:1:fig_end) - z_r(fig_start:1:fig_end)), 'b', 'LineWid',1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('x-axis', 'y-axis', 'z-axis');
title("Position Control Error Based on MPC");

figure;
plot(index, check2PI(rx(fig_start:1:fig_end) - rx_r(fig_start:1:fig_end)), 'r', 'linewidth', 1); hold on;
plot(index, check2PI(ry(fig_start:1:fig_end) - ry_r(fig_start:1:fig_end)), 'g', 'linewidth', 1); hold on;
plot(index, check2PI(rz(fig_start:1:fig_end) - rz_r(fig_start:1:fig_end)), 'b', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('roll', 'pitch', 'yaw');
title("Ratation Control Error Based on MPC");

figure;
plot(index, in1(fig_start:1:fig_end));
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
title('Command Trust');

figure;
plot(index, m1(fig_start:1:fig_end), 'r'); hold on;
plot(index, m2(fig_start:1:fig_end), 'g'); hold on;
plot(index, m3(fig_start:1:fig_end), 'b');
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
title('Command Moments');

figure;
plot(index, in1(fig_start:1:fig_end) - in1_r(fig_start:1:fig_end));
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
title('Command Trust Error');

figure;
plot(index, m1(fig_start:1:fig_end) - m1_r(fig_start:1:fig_end), 'r'); hold on;
plot(index, m2(fig_start:1:fig_end) - m2_r(fig_start:1:fig_end), 'g'); hold on;
plot(index, m3(fig_start:1:fig_end) - m3_r(fig_start:1:fig_end), 'b');
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
title('Command Moments Error');

figure;


plot(x(fig_start:1:fig_end), y(fig_start:1:fig_end), 'r-', 'linewidth', 0.5);
hold on; 
plot(x2(fig_start:1:fig_end), y2(fig_start:1:fig_end), 'b', 'linewidth', 0.5);
r = 1.5; xr= 0; yr =0;
th = 0:pi/50:2*pi;
xunit = r * cos(th) + xr;
yunit = r * sin(th) + yr;
plot(xunit, yunit, 'black-.', 'linewidth', 1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend( 'Normal MPC', 'JPCM-GI', 'Reference Path')
title('Position Control');
xlabel('x-axis (m)');
ylabel('y-axis (m)');

figure;
plot(x2(fig_start:1:fig_end), z2(fig_start:1:fig_end), 'b');
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
title('Position Control Error');
xlabel('x-axis (m)');
ylabel('z-axis (m)');

% figure;
% plot(x(fig_start:1:fig_end), y(fig_start:1:fig_end), 'b');
% hold on; plot(x2(fig_start:1:fig_end), y2(fig_start:1:fig_end), 'r');
% hold on; plot(x3(fig_start:1:fig_end), y3(fig_start:1:fig_end), 'g');
% hold on;
% r = 1.5; x= 0; y =0;
% th = 0:pi/50:2*pi;
% xunit = r * cos(th) + x;
% yunit = r * sin(th) + y;
% h = plot(xunit, yunit, 'black');
% legend('Normal MPC', 'JPCM-GI', 'MPC-NoNoise', 'Reference Path')
% title('Position Control Error');
% xlabel('x-axis (m)');
% ylabel('y-axis (m)');

figure; plot3(x2, y2, z2,'b');
disp('------------------------------- X RMS -------------------------------'); 
disp(rms(x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end)))
disp('------------------------------- X MAE -------------------------------'); 
disp(mean(abs(x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end))))
disp('------------------------------- Y RMS -------------------------------'); 
disp(rms(y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end)))
disp('------------------------------- Y MAE -------------------------------'); 
disp(mean(abs(y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end))))
disp('------------------------------- Z RMS -------------------------------'); 
disp(rms(z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end)))
disp('------------------------------- Z MAE -------------------------------'); 
disp(mean(abs(z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end))))

disp(rms(check2PI(rx2(fig_start:1:fig_end) - rx_r2(fig_start:1:fig_end))));
disp(rms(check2PI(ry2(fig_start:1:fig_end) - ry_r2(fig_start:1:fig_end))));
disp(rms(check2PI(rz2(fig_start:1:fig_end) - rz_r2(fig_start:1:fig_end))));


% figure;
% plot(index, 5*(rx2(fig_start:1:fig_end) - rx_r2(fig_start:1:fig_end)), 'r', 'LineWid',1); hold on;
% plot(index, 3*(ry2(fig_start:1:fig_end) - ry_r2(fig_start:1:fig_end)), 'g', 'LineWid',1); hold on;
% plot(index, 6*(rz2(fig_start:1:fig_end) - rz_r2(fig_start:1:fig_end)), 'b', 'LineWid',1);
% set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
% legend('x-axis', 'y-axis', 'z-axis');
% xlabel('time (s)');
% ylabel('error (m)');
% title("Positioning Error");