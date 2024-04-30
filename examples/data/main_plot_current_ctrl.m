clc
clear
close all

is_angle = false;
data = csvread("current_tracking.csv");
period = 10; % ms

spa = data(:,1);
ia = data(:,2);
spb = data(:,3);
ib = data(:,4);
spc = data(:,5);
ic = data(:,6);
spd = data(:,7);
id = data(:,8);

n = length(ia);
t = (0:n-1)/(1000/period);
t = t';

figure;
subplot(4,1,1);
plot(t, spa, "LineWidth", 2); hold on;
plot(t, ia, "LineWidth", 2); grid on; xlim([min(t) max(t)]);
ylabel("current [miliamp]"); xlabel("time [s]");
title("Magnet 0");
subplot(4,1,2);
plot(t, spb, "LineWidth", 2); hold on;
plot(t, ib, "LineWidth", 2); grid on; xlim([min(t) max(t)]);
ylabel("current [miliamp]"); xlabel("time [s]");
title("Magnet 1");
subplot(4,1,3);
plot(t, spc, "LineWidth", 2); hold on;
plot(t, ic, "LineWidth", 2); grid on; xlim([min(t) max(t)]);
ylabel("current [miliamp]"); xlabel("time [s]");
title("Magnet 2");
subplot(4,1,4);
plot(t, spd, "LineWidth", 2); hold on;
plot(t, id, "LineWidth", 2); grid on; xlim([min(t) max(t)]);
ylabel("current [miliamp]"); xlabel("time [s]");
title("Magnet 3");

