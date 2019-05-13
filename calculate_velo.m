% function [velocity_optical,omega_optical,points] = calculate_velo(data,imu_data_ctr,old_points,T,dt)
%     p = detectMinEigenFeatures(data(imu_data_ctr).img);
%     points = p.Location;
%     if imu_data_ctr ~= 1
%         prev_image = data(imu_data_ctr-1).img;
%         image = data(imu_data_ctr).img;
%         
%         pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
%         % point tracker initialization
%         
%         initialize(pointTracker,old_points,prev_image);
%         
%         % actual tracking
%         [coord_end, point_validity] = step(pointTracker, image);
%         coord_end = coord_end(point_validity, :);
%         old_points = old_points(point_validity, :);
%         
%         [velocity_optical,omega_optical] = compute_vel(coord_end,old_points,T(3),dt);
% 
%     %[velocity_optical,omega_optical] = compute_vel(u_dot,v_dot,u,v,Z);
%     else
%         velocity_optical = [0 0 0];
%         omega_optical = [0 0 0];
%     end
% %    
% end
function [velocity_optical,omega_optical] = calculate_velo(data,imu_data_ctr,T,dt)
    if imu_data_ctr ~= 1
        prev_image = data(imu_data_ctr-1).img;
        image = data(imu_data_ctr).img;
        
        points1 = detectHarrisFeatures(image);
        points2 = detectHarrisFeatures(prev_image);
        
        [features1,valid_points1] = extractFeatures(image,points1);
        [features2,valid_points2] = extractFeatures(prev_image,points2);
        
        indexPairs = matchFeatures(features1,features2);
        
        matchedPoints1 = valid_points1(indexPairs(:,1),:);
        matchedPoints2 = valid_points2(indexPairs(:,2),:);
        
        
%         hGTE2 = vision.fitgeotrans('Transform','Projective','InlierOutputPort',true,'AlgebraicDistanceThreshold',3,'RefineTransformMatrix',true,'NumRandomSamplings',2000);
%         hGT = vision.GeometricTransformer;
%         [tform, inliers] = step(hGTE2,matchedPoints1,matchedPoints2);
%         tform
%         inliers
%         figure;
%         showMatchedFeatures(prev_image,image,matchedPoints1,matchedPoints2);
%         %close;
%         
        velocity_optical = [0 0 0];
        omega_optical = [0 0 0];
        [velocity_optical,omega_optical] = compute_vel(matchedPoints1.Location,matchedPoints2.Location,T(3),dt);

    %[velocity_optical,omega_optical] = compute_vel(u_dot,v_dot,u,v,Z);
    else
        velocity_optical = [0 0 0];
        omega_optical = [0 0 0];
    end
end