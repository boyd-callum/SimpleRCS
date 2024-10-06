clear all; close all; clc;

[file, path] = uigetfile('*.csv');

sensorData = importdata(fullfile(path, file)).data();
time = sensorData(:,1)/ 1000.0; % in seconds
thetadot = sensorData(:,2);
throttle = sensorData(:,3);

plot(time, [thetadot,throttle], '-')
hold on
yline(0)
title('Example Data')
xlabel('Time (s)')
ylabel('Theta Dot (deg/s)')
legend("Theta Dot", "Throttle")


