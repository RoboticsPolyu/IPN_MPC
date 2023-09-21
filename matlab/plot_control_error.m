close all;
clear;
[state_x state_y state_z xep yep zep force M1 M2 M3 Rx Ry Rz] = textread('../data/record_info.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f');

% grouth truth and estimated trajectory


figure;
plot(state_x, state_y, 'r');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
xlabel('x(m)');
ylabel('y(m)');
title('XY');

figure;
plot(state_x, state_z, 'g');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
title('XZ');

figure;
plot(force, 'b-');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
xlabel('Time (s)');
ylabel(' Force (N)')
title('Control Force')

for i = 1:size(Rx)
    if Rx(i) < -3
        Rx(i) = Rx(i) + 6.28;
    end
    if Ry(i) < -3
        Ry(i) = Ry(i) + 6.28;
   end
   if Rz(i) < -3
        Rz(i) = Rz(i) + 6.28;
    end
end

figure;
plot(check2PI(Rx(1:1:end)), 'r');
hold on; plot(check2PI(Ry(1:1:end)), 'g');
hold on; plot(check2PI(Rz(1:1:end)), 'b');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
title('Rotation')

figure;
plot3(state_x, state_y, state_z, 'b');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);

figure;
plot(M1, 'r'); hold on; plot(M2, 'g'); hold on; plot(M3, 'b');
grid on;
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
title('Moments');


time = [1:1:size(xep)]* 0.01;

figure; 
plot(time, (xep(1:1:end)), 'r');
hold on; plot(time, (yep(1:1:end)), 'g');
hold on; plot(time, (zep(1:1:end)), 'b');
%xlim([0 15]);
grid on;
%ylim([-0.06 0.06]);
set(gca,'FontName','Times New Roman','FontSize',11,'LineWid',1);
legend('x-axis', 'y-axis','z-axis');
xlabel('Time (s)');
ylabel('Error (m)')
title('Trajectory control error');

length = size(xep);
disp('------------------------------- X RMS -------------------------------'); 
disp(rms(xep(2000:1:length)));
disp('------------------------------- Y RMS -------------------------------'); 
disp(rms(yep(2000:1:length)));
disp('------------------------------- Z RMS -------------------------------'); 
disp(rms(zep(2000:1:length)));
