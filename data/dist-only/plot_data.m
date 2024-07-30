clc
clear
close all

data = csvread("twenty.csv");

i0 = data(:,2);
i1 = data(:,3);
i2 = data(:,4);
i3 = data(:,5);
d0 = data(:,6);
d1 = data(:,7);
d2 = data(:,8);
d3 = data(:,9);
t = data(:,17);
t = cumsum(t);

md = mean([d0'; d1'; d2'; d3']);

figure;
plot(t, md);
grid on;
title('Mean relative distance [SilverSat]');
xlim([min(t), max(t)]);
xlabel('Time [seconds]');
ylabel('Relative distance [mm]');

figure;
plot(t, i0); grid on; hold on;
plot(t, i1);
plot(t, i2);
plot(t, i3);
title("Current through each electromagnets [SilverSat]");
xlabel("Time [seconds]");
ylabel("Current [Milliamps]");
xlim([min(t), max(t)]);
legend("EM-0", "EM-1", "EM-2", "EM-3");
saveas(gcf,'myfigure.pdf')

figure;
plot(t, d0); grid on; hold on;
plot(t, d1);
plot(t, d2);
plot(t, d3);
title("ToF range measurements [SilverSat]");
xlabel("Time [seconds]");
ylabel("Relative distance [mm]");
xlim([min(t), max(t)]);
legend("ToF-0", "ToF-1", "ToF-2", "ToF-3");

data = csvread("four.csv");

i0 = data(:,2);
i1 = data(:,3);
i2 = data(:,4);
i3 = data(:,5);
d0 = data(:,6);
d1 = data(:,7);
d2 = data(:,8);
d3 = data(:,9);
t = data(:,17);
t = cumsum(t);

hf = figure();
plot(t, i0); grid on; hold on;
plot(t, i1);
plot(t, i2);
plot(t, i3);
title("Current through each electromagnets [GoldSat]");
xlabel("Time [seconds]");
ylabel("Current [Milliamps]");
xlim([min(t), max(t)]);
legend("EM-0", "EM-1", "EM-2", "EM-3");

figure;
plot(t, d0); grid on; hold on;
plot(t, d1);
plot(t, d2);
plot(t, d3);
title("ToF range measurements [GoldSat]");
xlabel("Time [seconds]");
ylabel("Relative distance [mm]");
xlim([min(t), max(t)]);
legend("ToF-0", "ToF-1", "ToF-2", "ToF-3");

md = mean([d0'; d1'; d2'; d3']);

figure;
plot(t, md);
grid on;
title('Mean relative distance [GoldSat]');
xlim([min(t), max(t)]);
xlabel('Time [seconds]');
ylabel('Relative distance [mm]');

