close all;
clear all;

[x2 y2 z2 rx2 ry2 rz2 vx2 vy2 vz2 ga2 gy2 gz2 in12 m12 m22 m32 x_r2 y_r2 z_r2 rx_r2 ry_r2 rz_r2 vx_r2 vy_r2 vz_r2 ga_r2 gy_r2 gz_r2 in1_r2 m1_r2 m2_r2 m3_r2] = textread('../data/JEC_SW_JECM_onlyGNSS_5_N2__log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');

fig_end = 3000;
fig_start = 1;
index = [fig_start:1:fig_end]* 0.01;

figure;
plot(index, (x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end)), 'r', 'LineWid',1); hold on;
plot(index, (y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end)), 'g', 'LineWid',1); hold on;
plot(index, (z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end)), 'b', 'LineWid',1);
set(gca,'FontName','Times New Roman','FontSize',12,'LineWid',1);
legend('x-axis', 'y-axis', 'z-axis');
xlabel('time (s)');
ylabel('error (m)')
title("Position Control Error based on SW-JECM");

figure;
plot(index, check2PI(rx2(fig_start:1:fig_end) - rx_r2(fig_start:1:fig_end)), 'r'); hold on;
plot(index, check2PI(ry2(fig_start:1:fig_end) - ry_r2(fig_start:1:fig_end)), 'g'); hold on;
plot(index, check2PI(rz2(fig_start:1:fig_end) - rz_r2(fig_start:1:fig_end)), 'b');
set(gca,'FontName','Times New Roman','FontSize',12,'LineWid',1);
legend('roll', 'pitch', 'yaw');
xlabel('time (s)');
ylabel('error (rad)')
title("Ratation Control Error based on SW-JECM");


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
