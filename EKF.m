    function [pos_arr,vicon_arr,rpy_arr,vicon_arr_ang,time_arr,vel_arr] = EKF(data,vicon,time_vicon,time_data,omg_imu,acc_imu)
     
    pos_arr = [];
    vicon_arr = [];
    rpy_arr = [];
    vicon_arr_ang = [];
    time_arr = [];
    
    vel_arr = [];
    
    
    dim_data = size([data.t],2);
    dim_vicon = size(time_vicon,2);
    vicon_ctr = 1;
    imu_data_ctr = 1;
    
    points = 0;
    
    while true
        if imu_data_ctr > dim_data | vicon_ctr > dim_vicon
            break
        end


        if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
            % match
%             disp("at imu counter")
            imu_data_ctr
            if imu_data_ctr == 1
                dt = data(imu_data_ctr).t;      
            else
                dt = data(imu_data_ctr).t - data(imu_data_ctr-1).t;    
            end
            
            [rpy,T] = compute_pose(data,imu_data_ctr);
            rpy_arr = [rpy_arr; rpy];
            pos_arr = [pos_arr; T];
            quat_vicon = eul2quat([vicon(6,vicon_ctr) vicon(5,vicon_ctr) vicon(4,vicon_ctr)]);
            vicon_arr_ang = [vicon_arr_ang;quat_vicon(2:4)];
            vicon_arr = [vicon_arr;vicon(:,vicon_ctr).'];
            time_arr = [time_arr;data(imu_data_ctr).t];
            %velo_vicon = [velo_vicon;vel_vicon(7,vicon_ctr) vel_vicon(8,vicon_ctr) vel_vicon(9,vicon_ctr)];
            
   
            [velocity_optical,omega_optical,points] = calculate_velo(data,imu_data_ctr,points,T);
            vel_arr = [vel_arr;velocity_optical(1) velocity_optical(2) velocity_optical(3) omega_optical(1) omega_optical(2) omega_optical(3)];
            imu_data_ctr = imu_data_ctr + 1;
            vicon_ctr = vicon_ctr + 1;
        else
            % not match
            vicon_ctr = vicon_ctr + 1;
        end

    end    
end