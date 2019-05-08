close all
clear all
clc
%%load data from file and initilize
load('./data/studentdata9.mat');
init_script;
%pose = struct('t',[],'p',[]);
time_vicon=time;
time_data=cat(1,data.t);
vel_vicon = vicon(7:9,:);
omg_vicon = vicon(10:12,:);
rpy_vicon = vicon(4:6,:);
omg_imu = [];
acc_imu = [];
dimension = size(time_data,1);
for i=1:dimension
    omg_imu(:,i) = data(i).omg;
    acc_imu(:,i) = data(i).acc;
end

%%
[mean,cov] = EKF(data,vicon,time_vicon,time_data,omg_imu,acc_imu);

%%

figure('Name','orientation');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot

plot(ax1,[data.t],mean(:,4).');
grid(ax1,'on');
title(ax1,'X');
plot(ax2,[data.t],mean(:,5).');
grid(ax2,'on');
title(ax2,'Y');
plot(ax3,[data.t],mean(:,6).');
grid(ax3,'on');
title(ax3,'Z');
%%
plot3(mean(:,1),mean(:,2),mean(:,3))
