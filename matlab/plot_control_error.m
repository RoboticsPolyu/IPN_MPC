close all;
clear all;
[state_x state_y state_z xep yep zep force M1 M2 M3] = textread('../data/record_info.txt','%f %f %f %f %f %f %f %f %f %f');

% grouth truth and estimated trajectory
figure; plot((xep(1:1:end)), 'r');
hold on; plot((yep(1:1:end)), 'g');
hold on; plot((zep(1:1:end)), 'b');
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



