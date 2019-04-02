function [sigma_t,mu_t] = EKF_KF(dt,x,U,vicon)
    %EKF
    px = x(1);
    py = x(2);
    pz = x(3);
    
    phi = x(4);
    theta = x(5);
    psi = x(6);
    
    p_dot_x = x(7);
    p_dot_y = x(8);
    p_dot_z = x(9);
    
    bg_x = x(10);
    bg_y = x(11);
    bg_z = x(12);
    
     ba_x = x(13);
     ba_y = x(14);
     ba_z = x(15);
     
    vico_p = vicon(1:3);
    vico_q = vicon(4:6);
    vico_v = vicon(7:9);
    vico_w = vicon(10:12);
    
    wm_x = U(1);
    wm_y = U(2);
    wm_z = U(3);
    
    am_x = U(4);
    am_y = U(5);
    am_z = U(6);
    
    %constant
    C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];
    Q_t = (1e-3)*eye(12);
    R_t = (1e-3)*eye(6);
    d_t_j = dt * eye(15);
    %initial

    mu_t_1 = x;
    sigma_t_1 = (1e-3)*eye(15);
    %recompute
  %  x_dot = x_t(x,U,[0,0,0,0,0,0,0,0,0,0,0,0].');
    [A_t, U_t, X_dot] = calc_jacobian(wm_x, wm_y, wm_z, am_x, am_y, am_z,px,py,pz,phi,theta,psi,p_dot_x,p_dot_y,p_dot_z,bg_x,bg_y,bg_z,ba_x, ba_y, ba_z);
    
   % A_t = getA(x_dot, x);
   % U_t = getU(x_dot, [0,0,0,0,0,0,0,0,0,0,0,0].');
    
    x_t_IMU = x_t(mu_t_1,U,[0,0,0,0,0,0,0,0,0,0,0,0].');  %instead of passing U_t you have to pass wm and am from imu matrix
    
    size(A_t)
    
    F_t = eye(15) + d_t_j * A_t;
    Q_d = dt * Q_t;
    mu_t_cap = mu_t_1+ dt*x_t_IMU;
    sigma_t_cap = F_t * sigma_t_1 * F_t.' + U_t * Q_d * U_t.';
    %KF
    x_t_VICON = [vico_p;vico_q;vico_v;vico_w;0;0;0];
    Z_t = C*x_t_VICON + R_t;
    %Z_t = C*x_t_VICON + v;
    K_t = sigma_t_cap * C.' * inv(C * sigma_t_cap * C.' + R_t);
    mu_t = mu_t_cap + K_t * (Z_t - C * mu_t_cap);
    sigma_t = sigma_t_cap - K_t * C * sigma_t_cap;   
end
    
    

    