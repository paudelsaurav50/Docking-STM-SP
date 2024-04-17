clc
clear
close all

is_angle = false;
data = csvread("data.csv");

a = data(:,1);
b = data(:,2);
c = data(:,3);
d = data(:,4);

if(is_angle)
  yaw = data(:,5);
end

figure;

if(is_angle)
  subplot(2,1,1);
end

plot(a, 'LineWidth', 2); hold on;
plot(b, 'LineWidth', 2); grid on;
plot(c, 'LineWidth', 2);
plot(d, 'LineWidth', 2);
xlim([0, length(a)]);

if(is_angle)
  subplot(2,1,2);
  plot(yaw, 'LineWidth', 2); grid on;
  xlim([0, length(a)]);
end
