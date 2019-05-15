close all
clear all
clc
%%load data from file and initilize
load('./data/studentdata4.mat');

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
[pos_arr,vicon_arr,rpy_arr,vicon_arr_ang,time_arr,vel_arr] = on_board(data,vicon,time_vicon,time_data,omg_imu,acc_imu);
%%
% size(pos_arr)
size(vel_arr)
%%
figure('Name','roll');
plot(time_arr,vicon_arr_ang(:,1)) 
hold on 
plot(time_arr,rpy_arr(:,2))
hold off

figure('Name','pitch');
plot(time_arr,vicon_arr_ang(:,2)) 
hold on 
plot(time_arr,rpy_arr(:,3))
hold off

figure('Name','Yaw');
plot(time_arr,vicon_arr_ang(:,3)) 
hold on 
plot(time_arr,rpy_arr(:,4))
hold off

figure('Name','X');
plot(time_arr,vicon_arr(:,1)) 
hold on 
plot(time_arr,pos_arr(:,1))
hold off

figure('Name','Y');
plot(time_arr,vicon_arr(:,2)) 
hold on 
plot(time_arr,pos_arr(:,2))
hold off

figure('Name','Z');
plot(time_arr,vicon_arr(:,3)) 
hold on 
plot(time_arr,pos_arr(:,3))
hold off

%%
figure('Name','Vx');
plot(time_arr,vicon_arr(:,7)) 
hold on 
plot(time_arr,vel_arr(:,1))
hold off

figure('Name','Vy');
plot(time_arr,vicon_arr(:,8)) 
hold on 
plot(time_arr,vel_arr(:,2))
hold off

figure('Name','Vz');
plot(time_arr,vicon_arr(:,9)) 
hold on 
plot(time_arr,vel_arr(:,3))
hold off

figure('Name','Wx');
plot(time_arr,vicon_arr(:,10)) 
hold on 
plot(time_arr,vel_arr(:,4))
hold off

figure('Name','Wy');
plot(time_arr,vicon_arr(:,11)) 
hold on     
plot(time_arr,vel_arr(:,5))
hold off

figure('Name','Wz');
plot(time_arr,vicon_arr(:,12)) 
hold on 
plot(time_arr,vel_arr(:,6))
hold off


%%
rpy_arr = rpy_arr(:, 1:3);
size(rpy_arr)
on_board_cam = [pos_arr.' ; rpy_arr.' ; vel_arr.'];
%% 
size(on_board_cam)

%% 
[mean, covar] = EKF(data,on_board_cam,time_data,omg_imu,acc_imu);

%%
plot3(mean(:,1),mean(:,2),mean(:,3))

%%
figure('Name','Position Predicted');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
plot(ax1,[data.t],mean(:,1).');
grid(ax1,'on');
title(ax1,'X');
plot(ax2,[data.t],mean(:,2).');
grid(ax2,'on');
title(ax2,'Y');
plot(ax3,[data.t],mean(:,3).');
grid(ax3,'on');
title(ax3,'Z');

figure('Name','Position Vicon');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
plot(ax1,[data.t],vicon_x.');
grid(ax1,'on');
title(ax1,'X');
plot(ax2,[data.t],vicon_y.');
grid(ax2,'on');
title(ax2,'Y');
plot(ax3,[data.t],vicon_z.');
grid(ax3,'on');
title(ax3,'Z');

%%
figure('Name','Position X');
plot([data.t],mean(:,1).');
hold on;
plot([data.t],vicon_x.');
hold off;


figure('Name','Position Y');
plot([data.t],mean(:,2).');
hold on;
plot([data.t],vicon_y.');
hold off;

figure('Name','Position z');
plot([data.t],mean(:,3).');
hold on;
plot([data.t],vicon_z.');
hold off;


figure('Name','Orientation Predicted');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
plot(ax1,[data.t],mean(:,4).');
grid(ax1,'on');
title(ax1,'roll');
plot(ax2,[data.t],mean(:,5).');
grid(ax2,'on');
title(ax2,'pitch');
plot(ax3,[data.t],mean(:,6).');
grid(ax3,'on');
title(ax3,'Yaw');

figure('Name','Orientation Vicon');
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
plot(ax1,[data.t],vicon_r.');
grid(ax1,'on');
title(ax1,'roll');
plot(ax2,[data.t],vicon_p.');
grid(ax2,'on');
title(ax2,'pitch');
plot(ax3,[data.t],vicon_yaw.');
grid(ax3,'on');
title(ax3,'Yaw');


