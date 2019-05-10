function [velocity_optical,omega_optical] = calculate_velo(data,vicon,time_vicon,time_data,omg_imu,acc_imu,imu_data_ctr,vicon_ctr,dt)
    velocity_optical = [0 0 0];
    omega_optical = [0 0 0];
    num_track = 200;
    flag = 0;
    
    points = detectHarrisFeatures(data(imu_data_ctr).img);
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

    % Initialize the tracker with the initial point locations and the initial
    % video frame.
%     points = points.Location;
%     initialize(pointTracker, points, videoFrame);
%     
    
    
    if imu_data_ctr == 373
        imshow(data(imu_data_ctr).img); hold on;
        plot(points.selectStrongest(50));
    end
    
    
    
%     if flag == 0
%         flag = 1;
%         prev_vel = [0 0 0];
%         prev_omg = [0 0 0];
%         prev_dt = dt;
%         
%         %initialize KLT
%         point_tracker = vision.PointTracker('MaxBidirectionalError', 2);
%         corners = detectFASTFeatures(data(imu_data_ctr).img);
%         points = corners.selectStrongest(num_track);
%         points = double(points.Location);
%         initialize(point_tracker, points, data(imu_data_ctr));
%         
%         
%   %Constants
%         K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
%         inv_k = inv(K);
%         prev_x = vicon(vicon_ctr,1);
%         prev_y = vicon(vicon_ctr,2);
%         prev_z = vicon(vicon_ctr,3);
%     end
    
end