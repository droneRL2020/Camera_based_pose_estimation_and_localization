    function [mean_arr,cov_arr] = EKF(data,on_board_cam,omg_imu,acc_imu)
 
    %Defining C

     C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ;
          0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ; 
          0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 ;
          0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ;
          0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ;
          0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];

    %Defining Covariance Matrix

    Q = (1e-5)*eye(12);
    R = (1e-3)*eye(9);
    %d_t = dt * eye(15);
    %d_tq = dt * eye(12);

    %creating a list of all the variables

  %  ran = [x, y, z, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z, w_x, w_y, w_z, a_x, a_y, a_z, ng_x, ng_y, ng_z, na_x, na_y, na_z, nbg_x, nbg_y, nbg_z, nba_x, nba_y, nba_z];
    %size(ran)
    mean = [0 ;0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 
    %disp('mean')
    %size(mean)
    covar = (0.01)*eye(15);

    %dimension = size(time_data,1);
%     omg_imu = [];
%     acc_imu = [];
    
    mean_arr = [];
    cov_arr = [];

    dim_data = size([data.t],2);
    
    
    imu_data_ctr = 1;

    while true
        if imu_data_ctr > dim_data
            break
        end


%         if data(imu_data_ctr).t == time_vicon(vicon_ctr) 
            % match
            disp("at imu counter")
            imu_data_ctr
            if imu_data_ctr == 1
                dt = data(imu_data_ctr).t;      
            else
                dt = data(imu_data_ctr).t - data(imu_data_ctr-1).t;    
            end
            d_t = dt * eye(15);
            d_tq = dt * eye(12);
            %size(ran)
            %disp('in the loop. the size of the subs')
            %size([mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]])
            
            A_cal = A_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %A_cal = subs(A, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);
            U_cal = U_matrix(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %U_cal = subs(U, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);
            
            x_dot_cal = x_d_v(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7), mean(8), mean(9), mean(10), mean(11), mean(12), mean(13), mean(14), mean(15), omg_imu(1,imu_data_ctr), omg_imu(2,imu_data_ctr), omg_imu(3,imu_data_ctr), acc_imu(1,imu_data_ctr), acc_imu(2,imu_data_ctr), acc_imu(3,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %x_dot_cal = subs(x_dot, ran,[mean.', omg_imu(:,imu_data_ctr).', acc_imu(:,imu_data_ctr).', [0 0 0 0 0 0 0 0 0 0 0 0]]);            
            x_cal = x_v(on_board_cam(1,imu_data_ctr), on_board_cam(2,imu_data_ctr), on_board_cam(3,imu_data_ctr), on_board_cam(4,imu_data_ctr), on_board_cam(5,imu_data_ctr), on_board_cam(6,imu_data_ctr), on_board_cam(7,imu_data_ctr), on_board_cam(8,imu_data_ctr), on_board_cam(9,imu_data_ctr), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            %x_cal = subs(x_vector, ran,[vicon(1:6,vicon_ctr).',[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]]);
            
            %use this for ques 2
            %x_cal = double(subs(x, ran,[[0 0 0 0 0 0], vicon(7:9,i).',[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]]));   
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
%             vicon_ctr = vicon_ctr + 1;
%         else
            % not match
%             vicon_ctr = vicon_ctr + 1;
%         end

    end

    
    
    
    
    
end