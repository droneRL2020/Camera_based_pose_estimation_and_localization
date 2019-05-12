function [velocity_optical,omega_optical,points] = calculate_velo(data,imu_data_ctr,old_points,T)
    p = detectMinEigenFeatures(data(imu_data_ctr).img);
    points = p.Location;
    if imu_data_ctr ~= 1
        prev_image = data(imu_data_ctr-1).img;
        image = data(imu_data_ctr).img;
        
        pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
        % point tracker initialization
        
        initialize(pointTracker,old_points,prev_image);
        
        % actual tracking
        [coord_end, point_validity] = step(pointTracker, image);
        %visiblePoints = coord_end(point_validity, :);
        %valid_old_points = old_points(point_validity, :);
        
        
        
        len1 = size(coord_end);
        len2 = size(old_points);
        %len1
        %len2
        [velocity_optical,omega_optical] = compute_vel(coord_end,old_points,T(3));
        velocity_optical
        omega_optical
    %[velocity_optical,omega_optical] = compute_vel(u_dot,v_dot,u,v,Z);

    end
%    
    velocity_optical = 0;
    omega_optical = 0;
end