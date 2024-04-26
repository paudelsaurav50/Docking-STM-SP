clc
clear
close all

is_angle = false;
data = csvread("current_regulation.csv");

spa = data(:,1);
ia = data(:,2);
spb = data(:,3);
ib = data(:,4);
spc = data(:,5);
ic = data(:,6);
spd = data(:,7);
id = data(:,8);

figure;
subplot(4,1,1);
plot(spa, "LineWidth", 2); hold on;
plot(ia, "LineWidth", 2); grid on;
ylabel("current [amp]"); xlabel("time");
title("Magnet 0");
subplot(4,1,2);
plot(spb, "LineWidth", 2); hold on;
plot(ib, "LineWidth", 2); grid on;
ylabel("current [amp]"); xlabel("time");
title("Magnet 1");
subplot(4,1,3);
plot(spc, "LineWidth", 2); hold on;
plot(ic, "LineWidth", 2); grid on;
ylabel("current [amp]"); xlabel("time");
title("Magnet 2");
subplot(4,1,4);
plot(spd, "LineWidth", 2); hold on;
plot(id, "LineWidth", 2); grid on;
ylabel("current [amp]"); xlabel("time");
title("Magnet 3");
