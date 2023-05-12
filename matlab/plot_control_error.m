close all;
clear all;
[state_x state_y state_z xep yep zep force M1 M2 M3 Rx Ry Rz] = textread('../data/record_info.txt','%f %f %f %f %f %f %f %f %f %f %f %f %f');

% grouth truth and estimated trajectory
i = [1:1:size(xep)]* 0.01;

figure; plot(i, (xep(1:1:end)), 'r');
hold on; plot(i, (yep(1:1:end)), 'g');
hold on; plot(i, (zep(1:1:end)), 'b');
legend('x-axis', 'y-axis','z-axis');

title('Trajectory control error');

figure;
plot(state_x, state_y, 'r');
xlabel('x(m)');
ylabel('y(m)');
title('XY');

figure;
plot(state_x, state_z, 'g');
title('XZ');

figure;
plot(force, 'b-');
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
plot((Rx(1:1:end)), 'r');
hold on; plot((Ry(1:1:end)), 'g');
hold on; plot((Rz(1:1:end)), 'b');
title('Rotation')


