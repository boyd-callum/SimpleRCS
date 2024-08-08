clear all; close all; clc;


sinedata = importdata("EXAMPLE.CSV").data();
time = sinedata(:,1)/ 1000.0; % in seconds
values = sinedata(:,2);

plot(time, values, '.')
title('Example Data')
xlabel('Time (s)')
ylabel('Amplitude')

