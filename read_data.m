close all
clear all
clc
%%load data from file and initilize
load('./data/studentdata9.mat');
init_script;
%pose = struct('t',[],'p',[]);
%%estimate position using AprilTag tech
time_vicon=time;
time_data=cat(1,data.t);
vel_vicon = vicon(7:9,:);
omg_vicon = vicon(10:12,:);
rpy_vicon = vicon(4:6,:);
dimension = size(time_data,1);
omg_imu = [];
acc_imu = [];

for i=1:dimension
    omg_imu(:,i) = data(i).omg;
    acc_imu(:,i) = data(i).acc;
end

dem_data = size([data.t],2);
dem_vicon = size(time_vicon,2);
vicon_ctr = 1;
imu_data_ctr = 1;
x = [0 0 0.03 0 0 0 0 0 0 0 0 0 0 0 0].';
while true
    if imu_data_ctr > dem_data | vicon_ctr > dem_vicon
        break
    end
    
    
    if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
        % match
        I = eye(15);
        if imu_data_ctr == 1
            delta_t = data(imu_data_ctr).t;      
        else
            delta_t = data(imu_data_ctr).t - data(imu_data_ctr-1).t;    
        end
        A_matrix = zeros(15,15);
        
        U_matrix = zeros(15,12);
        delta_t = data(imu_data_ctr).t;
        
        % imu data
        q = data(imu_data_ctr).rpy;
        q_dot = data(imu_data_ctr).omg;
        p_double_dot = data(imu_data_ctr).acc;
        
        % vicon data
        vicon_data = vicon(:,vicon_ctr);
        
        
        [sigma_t,x_dot] = EKF_KF(delta_t,x,vicon_data(1:12));
        x = x_dot;
        % swap and counter
        q_prev = q;
        q_dot_prev = q_dot;
        p_double_dot_prev = p_double_dot;
        
        
        imu_data_ctr = imu_data_ctr + 1;
        vicon_ctr = vicon_ctr + 1;
    else
        % not match
        vicon_ctr = vicon_ctr + 1;
    end
    
end








figure('Name','Velocity');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
% plot(ax1,t_vicon,vel_vicon(1,:));
% grid(ax1,'on');
% title(ax1,'Vx');
% plot(ax2,t_vicon,vel_vicon(2,:));
% grid(ax2,'on');
% title(ax2,'Vy');
% plot(ax3,t_vicon,vel_vicon(3,:));
% grid(ax3,'on');
% title(ax3,'Vz');
% 
% figure('Name','Orientation');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;
% plot(ax1,t_vicon,rpy_vicon(1,:));
% grid(ax1,'on');
% title(ax1,'roll');
% plot(ax2,t_vicon,rpy_vicon(2,:));
% grid(ax2,'on');
% title(ax2,'pitch');
% plot(ax3,t_vicon,rpy_vicon(3,:));
% grid(ax3,'on');
% title(ax3,'Yaw');
% 
% figure('Name','Acceleration');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% plot(ax1,time_data,acc_imu(1,:),'g');
% hold(ax1,'on'); 
% title(ax1,'ax');
% plot(ax2,time_data,acc_imu(2,:),'g');
% hold(ax2,'on'); 
% title(ax2,'ay');
% plot(ax3,time_data,acc_imu(3,:),'g');
% hold(ax3,'on'); 
% title(ax3,'az');
% 
% %%
% figure('Name','Omega');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% t_vicon = time_vicon;%0:0.01:17.67;
% 
% plot(ax1,time_data,omg_imu(1,:),'g');
% hold(ax1,'on'); 
% plot(ax1,t_vicon,omg_vicon(1,:));
% grid(ax1,'on'); 
% title(ax1,'Wx');
% 
% plot(ax2,time_data,omg_imu(2,:),'g');
% hold(ax2,'on'); 
% plot(ax2,t_vicon,omg_vicon(2,:));
% grid(ax2,'on');
% title(ax2,'Wy');
% 
% plot(ax3,time_data,omg_imu(3,:),'g');
% hold(ax3,'on'); 
% plot(ax3,t_vicon,omg_vicon(3,:));
% grid(ax3,'on');
% title(ax3,'Wz');