close all;
clear all;
[xep yep zep] = textread('err_opt_proba_010.txt','%f %f %f');
[xe ye ze] = textread('err_opt_noproba.txt','%f %f %f ');

% grouth truth and estimated trajectory
figure; plot(abs(xep(1:1:3000)), 'r');
hold on; plot(abs(xe(1:1:3000)), 'g');
legend('Probability', 'No Probability');
title('x err');

figure; plot(abs(yep(1:1:3000)), 'r');
hold on; plot(abs(ye(1:1:3000)), 'g');
legend('Probability', 'No Probability');
title('y err');

figure; plot(abs(zep(1:1:3000)), 'r');
hold on; plot(abs(ze(1:1:3000)), 'g');
legend('Probability', 'No Probability');
title('z err');