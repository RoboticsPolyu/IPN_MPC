close all;
clear all;

%MPC 
[x1 y1 z1 rx1 ry1 rz1 vx1 vy1 vz1 ga1 gy1 gz1 in11 m21 m21 m31 x_r1 y_r1 z_r1 rx_r1 ry_r1, rz_r1 vx_r1 vy_r1 vz_r1 ga_r1 gy_r1 gz_r1 in1_r1 m1_r1 m2_r1 m3_r1] = textread('../data/JEC_JECM_Rev_0.20m_log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');

% JPCM sig = 0.10m
[x2 y2 z2 rx2 ry2 rz2 vx2 vy2 vz2 ga2 gy2 gz2 in12 m12 m22 m32 x_r2 y_r2 z_r2 rx_r2 ry_r2 rz_r2 vx_r2 vy_r2 vz_r2 ga_r2 gy_r2 gz_r2 in1_r2 m1_r2 m2_r2 m3_r2] = textread('../data/JEC_JECM_Rev_0.10m_log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');

% JPCM sig = 0.20m
[x3 y3 z3 rx3 ry3 rz3 vx3 vy3 vz3 ga3 gy3 gz3 in13 m13 m23 m33 x_r3 y_r3 z_r3 rx_r3 ry_r3, rz_r3 vx_r3 vy_r3 vz_r3 ga_r3 gy_r3 gz_r3 in1_r3 m1_r3 m2_r3 m3_r3] = textread('../data/JEC_JECM_Rev_0.20m_log.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ');

fig_end = 3000;
fig_start = 1;
index = [fig_start:1:fig_end]* 0.01;

figure;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
plot(index, (x2(fig_start:1:fig_end) - x_r2(fig_start:1:fig_end)), 'r', 'LineWid',1); hold on;
plot(index, (y2(fig_start:1:fig_end) - y_r2(fig_start:1:fig_end)), 'g', 'LineWid',1); hold on;
plot(index, (z2(fig_start:1:fig_end) - z_r2(fig_start:1:fig_end)), 'b', 'LineWid',1);
% xlabel('time (s)');
% ylabel('error (m)')
% title("Position Control Error of Nominal MPC");

% figure;
hold on;
plot(index, (x3(fig_start:1:fig_end) - x_r3(fig_start:1:fig_end)), 'r-.', 'LineWid',1); hold on;
plot(index, (y3(fig_start:1:fig_end) - y_r3(fig_start:1:fig_end)), 'g-.', 'LineWid',1); hold on;
plot(index, (z3(fig_start:1:fig_end) - z_r3(fig_start:1:fig_end)), 'b-.', 'LineWid',1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('JPCM 0.10m x-axis', 'JPCM 0.10m y-axis', 'JPCM 0.10m z-axis', 'JPCM 0.20m x-axis', 'JPCM 0.20m y-axis', 'JPCM 0.20m z-axis');
xlabel('time (s)');
ylabel('error (m)')
title("Position Control Error Comparision of MPC and JPCM");

figure;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
plot(index, check2PI((rx2(fig_start:1:fig_end))), 'r', 'LineWid',1); hold on;
plot(index, check2PI((ry2(fig_start:1:fig_end))), 'g', 'LineWid',1); hold on;
plot(index, check2PI((rz2(fig_start:1:fig_end))), 'b', 'LineWid',1);
% xlabel('time (s)');
% ylabel('error (m)')
% title("Position Control Error of Nominal MPC");

% figure;
hold on;
plot(index, check2PI((rx3(fig_start:1:fig_end))), 'r-.', 'LineWid',1); hold on;
plot(index, check2PI((ry3(fig_start:1:fig_end))), 'g-.', 'LineWid',1); hold on;
plot(index, check2PI((rz3(fig_start:1:fig_end))), 'b-.', 'LineWid',1);
grid on;
set(gca,'FontName','Times New Roman','FontSize',8,'LineWid',1);
legend('JPCM x-axis', 'JPCM y-axis', 'JPCM z-axis', 'MPC x-axis', 'MPC y-axis', 'MPC z-axis');
xlabel('time (s)');
ylabel('error (m)')
title("Rotation Control Comparison between MPC and JPCM");

dur = 1200;
disp('------------------------------- JPCM  -------------------------------'); 
disp('------------------------------- X RMS -------------------------------'); 
disp(rms(x2(fig_start + dur:1:fig_end) - x_r2(fig_start + dur:1:fig_end)))
disp('------------------------------- X MAE -------------------------------'); 
disp(mean(abs(x2(fig_start + dur:1:fig_end) - x_r2(fig_start + dur:1:fig_end))))
disp('------------------------------- Y RMS -------------------------------'); 
disp(rms(y2(fig_start + dur:1:fig_end) - y_r2(fig_start + dur:1:fig_end)))
disp('------------------------------- Y MAE -------------------------------'); 
disp(mean(abs(y2( fig_start+ dur:1:fig_end) - y_r2(fig_start + dur:1:fig_end))))
disp('------------------------------- Z RMS -------------------------------'); 
disp(rms(z2(fig_start + dur:1:fig_end) - z_r2(fig_start + dur:1:fig_end)))
disp('------------------------------- Z MAE -------------------------------'); 
disp(mean(abs(z2(fig_start + dur:1:fig_end) - z_r2(fig_start + dur:1:fig_end))))

disp('------------------------------- MPC  -------------------------------'); 
disp('------------------------------- X RMS -------------------------------'); 
disp(rms(x3(fig_start + dur:1:fig_end) - x_r3(fig_start + dur:1:fig_end)))
disp('------------------------------- X MAE -------------------------------'); 
disp(mean(abs(x3(fig_start + dur:1:fig_end) - x_r3(fig_start + dur:1:fig_end))))
disp('------------------------------- Y RMS -------------------------------'); 
disp(rms(y3(fig_start + dur:1:fig_end) - y_r3(fig_start + dur:1:fig_end)))
disp('------------------------------- Y MAE -------------------------------'); 
disp(mean(abs(y3( fig_start+ dur:1:fig_end) - y_r3(fig_start + dur:1:fig_end))))
disp('------------------------------- Z RMS -------------------------------'); 
disp(rms(z3(fig_start + dur:1:fig_end) - z_r3(fig_start + dur:1:fig_end)))
disp('------------------------------- Z MAE -------------------------------'); 
disp(mean(abs(z3(fig_start + dur:1:fig_end) - z_r3(fig_start + dur:1:fig_end))))