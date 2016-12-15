% matlab code for extracting data for analysis form rosbag files
clc;
clear all;
close all;
bag = rosbag('bagfile.bag');
bagselect1 = select(bag, 'Topic', '/crazyflie/crazyflie_position');
start = bag.StartTime;

ts = timeseries(bagselect1);
t = ts.Time;
x_pos = ts.Data(:,1);
y_pos = ts.Data(:,2);
z_pos = ts.Data(:,3);
x_mean = mean(x_pos);
y_mean = mean(y_pos);
z_mean = mean(z_pos);
sz = size(x_pos,1);
x_mean_array = repmat(x_mean,sz,1);
y_mean_array = repmat(y_mean,sz,1);
z_mean_array = repmat(z_mean,sz,1);

figure(1);
hold on;
plot(x_pos,'r'), plot(x_mean_array,'b');
title('Crazflie x-position')
legend('data',...
       'mean','Location','east');

figure(2);
hold on;
plot(y_pos,'r'), plot(y_mean_array,'b');
title('Crazflie y-position')
legend('data',...
       'mean','Location','east');

figure(3);
hold on;
plot(z_pos,'r'), plot(z_mean_array,'b');
title('Crazflie z-position')
legend('data',...
       'mean','Location','east');
