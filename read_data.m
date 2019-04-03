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






figure('Name','Velocity');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
t_vicon = time_vicon;
plot(ax1,t_vicon,vel_vicon(1,:));
grid(ax1,'on');
title(ax1,'Vx');
plot(ax2,t_vicon,vel_vicon(2,:));
grid(ax2,'on');
title(ax2,'Vy');
plot(ax3,t_vicon,vel_vicon(3,:));
grid(ax3,'on');
title(ax3,'Vz');

figure('Name','Orientation');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
t_vicon = time_vicon;
plot(ax1,t_vicon,rpy_vicon(1,:));
grid(ax1,'on');
title(ax1,'roll');
plot(ax2,t_vicon,rpy_vicon(2,:));
grid(ax2,'on');
title(ax2,'pitch');
plot(ax3,t_vicon,rpy_vicon(3,:));
grid(ax3,'on');
title(ax3,'Yaw');

figure('Name','Acceleration');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
plot(ax1,time_data,acc_imu(1,:),'g');
hold(ax1,'on'); 
title(ax1,'ax');
plot(ax2,time_data,acc_imu(2,:),'g');
hold(ax2,'on'); 
title(ax2,'ay');
plot(ax3,time_data,acc_imu(3,:),'g');
hold(ax3,'on'); 
title(ax3,'az');

%%
figure('Name','Omega');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
t_vicon = time_vicon;%0:0.01:17.67;

plot(ax1,time_data,omg_imu(1,:),'g');
hold(ax1,'on'); 
plot(ax1,t_vicon,omg_vicon(1,:));
grid(ax1,'on'); 
title(ax1,'Wx');

plot(ax2,time_data,omg_imu(2,:),'g');
hold(ax2,'on'); 
plot(ax2,t_vicon,omg_vicon(2,:));
grid(ax2,'on');
title(ax2,'Wy');

plot(ax3,time_data,omg_imu(3,:),'g');
hold(ax3,'on'); 
plot(ax3,t_vicon,omg_vicon(3,:));
grid(ax3,'on');
title(ax3,'Wz');