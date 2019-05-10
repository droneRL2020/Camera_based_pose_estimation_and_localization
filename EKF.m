    function [mean_arr,cov_arr,rpy_arr,vicon_arr,time_arr] = EKF(data,vicon,time_vicon,time_data,omg_imu,acc_imu)
        %Defining C

     C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ;
          0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];

    %Defining Covariance Matrix

    Q = (1e-3)*eye(12);
    R = (1e-3)*eye(6);
    
    %creating a list of all the variables

    mean = [0 ;0; 0.03; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 

    covar = (0.01)*eye(15);
    
    mean_arr = [];
    cov_arr = [];
    rpy_arr = [];
    vicon_arr = [];
    time_arr = []
    
    dim_data = size([data.t],2);
    dim_vicon = size(time_vicon,2);
    vicon_ctr = 1;
    imu_data_ctr = 1;

    while true
        if imu_data_ctr > dim_data | vicon_ctr > dim_vicon
            break
        end


        if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
            % match
            disp("at imu counter")
            imu_data_ctr
            if imu_data_ctr == 1
                dt = data(imu_data_ctr).t;      
            else
                dt = data(imu_data_ctr).t - data(imu_data_ctr-1).t;    
            end
            
            rpy = compute_pose(data,imu_data_ctr);
            [velocity_optical,omega_optical] = calculate_velo(data,vicon,time_vicon,time_data,omg_imu,acc_imu,imu_data_ctr,vicon_ctr)
            rpy_arr = [rpy_arr; rpy];
            quat_vicon = eul2quat([vicon(6,vicon_ctr) vicon(5,vicon_ctr) vicon(4,vicon_ctr)]);
            vicon_arr = [vicon_arr;quat_vicon];
            time_arr = [time_arr;data(imu_data_ctr).t];
            %R = [pose(1,1) pose(1,2) pose(1,3);pose(2,1) pose(2,2) pose(2,3);pose(3,1) pose(3,2) pose(3,3)];
            %j = pose(2,1)/pose(1,1);
            
            %alpha = atan2(pose(2,1),pose(1,1));
%             disp("##########")
%             vicon(1,vicon_ctr) 
%             vicon(2,vicon_ctr)
%             vicon(3,vicon_ctr)
%             vicon(4,vicon_ctr) 
%             vicon(5,vicon_ctr)
%             vicon(6,vicon_ctr)
%             disp("!!!!!!!!!!")
            d_t = dt * eye(15);
            d_tq = dt * eye(12);
            
            A_cal = A_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

            U_cal = U_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            
            x_dot_cal = x_d_v(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

            x_cal = x_v(vicon(1,vicon_ctr), vicon(2,vicon_ctr), vicon(3,vicon_ctr), vicon(4,vicon_ctr), vicon(5,vicon_ctr), vicon(6,vicon_ctr),0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %use this for ques 2

            F = eye(15) + d_t * A_cal;
            Qd = d_tq * Q;
            z = C*x_cal;

            pred_mean = mean + d_t*x_dot_cal;
            pred_covar = F*covar*(F.') + U_cal*Qd*(U_cal.');

            K = pred_covar*(C.')*(inv(C*pred_covar*(C.') + R));
            
%             
            mean = pred_mean + K*(z - (C*pred_mean));
            covar = pred_covar - (K*C*pred_covar);
%             
            
            mean_arr = [mean_arr; mean.'];
            cov_arr = [cov_arr; covar.'];
            
            
            imu_data_ctr = imu_data_ctr + 1;
            vicon_ctr = vicon_ctr + 1;
        else
            % not match
            vicon_ctr = vicon_ctr + 1;
        end

    end    
end