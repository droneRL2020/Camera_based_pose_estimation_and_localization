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
[mean,cov] = EKF(data,vicon,time_vicon,omg_imu,acc_imu);

%%
plot3(mean(:,1),mean(:,2),mean(:,3))
%%

dim_data = size([data.t],2);
dim_vicon = size(time_vicon,2);
vicon_ctr = 1;
imu_data_ctr = 1;
vicon_x = [];
vicon_y = [];
vicon_z = [];
vicon_r = [];
vicon_p = [];
vicon_yaw = [];
vicon_vx = [];
vicon_vy = [];
vicon_vz = [];
while true
    if imu_data_ctr > dim_data | vicon_ctr > dim_vicon
        break
    end

    if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
            % match
        vicon_x = [vicon_x;vicon(1,vicon_ctr)];   
        vicon_y = [vicon_y;vicon(2,vicon_ctr)];  
        vicon_z = [vicon_z;vicon(3,vicon_ctr)];  
        vicon_r = [vicon_r;vicon(4,vicon_ctr)];  
        vicon_p = [vicon_p;vicon(5,vicon_ctr)];  
        vicon_yaw = [vicon_yaw;vicon(6,vicon_ctr)]; 
        
        vicon_vx = [vicon_vx;vicon(7,vicon_ctr)];
        vicon_vy = [vicon_vy;vicon(8,vicon_ctr)];
        vicon_vz = [vicon_vz;vicon(9,vicon_ctr)];
        
        imu_data_ctr = imu_data_ctr + 1;
        vicon_ctr = vicon_ctr + 1;
    else
        vicon_ctr = vicon_ctr + 1;
    end

end
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

% 
% figure('Name','Orientation Predicted');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% % t_vicon = time_vicon;
% plot(ax1,[data.t],mean(:,4).');
% grid(ax1,'on');
% title(ax1,'roll');
% plot(ax2,[data.t],mean(:,5).');
% grid(ax2,'on');
% title(ax2,'pitch');
% plot(ax3,[data.t],mean(:,6).');
% grid(ax3,'on');
% title(ax3,'Yaw');
% 
% figure('Name','Orientation Vicon');
% ax1 = subplot(3,1,1); % top subplot
% ax2 = subplot(3,1,2); % middle subplot
% ax3 = subplot(3,1,3); % bottom subplot
% % t_vicon = time_vicon;
% plot(ax1,[data.t],vicon_r.');
% grid(ax1,'on');
% title(ax1,'roll');
% plot(ax2,[data.t],vicon_p.');
% grid(ax2,'on');
% title(ax2,'pitch');
% plot(ax3,[data.t],vicon_yaw.');
% grid(ax3,'on');
% title(ax3,'Yaw');
