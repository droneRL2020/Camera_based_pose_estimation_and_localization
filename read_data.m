close all
clear all
clc
%%load data from file and initilize
load('./data/studentdata1.mat');

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
[pos_arr,vicon_arr_pos,rpy_arr,vicon_arr_ang,time_arr,velocity_optical,omega_optical,velo_vicon] = EKF(data,vicon,time_vicon,time_data,omg_imu,acc_imu);

%%
% figure('Name','roll');
% plot(time_arr,vicon_arr_angle(:,1)) 
% hold on 
% plot(time_arr,rpy(:,2))
% hold off
% 
% figure('Name','pitch');
% plot(time_arr,vicon_arr_angle(:,2)) 
% hold on 
% plot(time_arr,rpy(:,3))
% hold off
% 
% figure('Name','Yaw');
% plot(time_arr,vicon_arr_angle(:,3)) 
% hold on 
% plot(time_arr,rpy(:,4))
% hold off
% 
% figure('Name','X');
% plot(time_arr,vicon_arr_pos(:,1)) 
% hold on 
% plot(time_arr,pos(:,1))
% hold off
% 
% figure('Name','Y');
% plot(time_arr,vicon_arr_pos(:,2)) 
% hold on 
% plot(time_arr,pos(:,2))
% hold off
% 
% figure('Name','Z');
% plot(time_arr,vicon_arr_pos(:,3)) 
% hold on 
% plot(time_arr,pos(:,3))
% hold off
% 
% 
figure('Name','Vx');
plot(time_arr,velo_vicon(:,1)) 
hold on 
plot(time_arr,velo_arr_cam(:,1))
hold off
figure('Name','Vy');
plot(time_arr,velo_vicon(:,2)) 
hold on 
plot(time_arr,velo_arr_cam(:,2))
hold off
figure('Name','Vz');
plot(time_arr,velo_vicon(:,3)) 
hold on 
plot(time_arr,velo_arr_cam(:,3))
hold off

