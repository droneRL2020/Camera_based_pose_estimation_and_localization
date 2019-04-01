function EKF_prediciton_KF_update=EKF_prediciton_KF_update(dt,px,py,pz,phi,theta,psi,p_dot_x,
                                                           p_dot_y,p_dot_z,bg_x,bg_y,bg_z,ba_x, 
                                                           ba_y, ba_z,vico_p,vico_q,vico_v,vico_w)
    %EKF
    %constant
    C=[1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];
    Q_t=(1e-3)*eye(12);
    R_t=(1e-3)*eye(6);
    %initial
    x0=[0 0 3 0 0 0 0 0 0 0 0 0 0 0 0].T;
    mu_(t-1)=x0;
    sigma_(t-1)=(1e-3)*eye(15);
    %recompute
    A_t,U_t,X_dot = calc_jocbian(px,py,pz,phi,theta,psi,p_dot_x,p_dot_y,p_dot_z,bg_x,bg_y,bg_z,ba_x, ba_y, ba_z);
    x_t_IMU = x_t(mu_(t-1),U_t,0);
    F_t=eye(15)+0.05*A_t;
    mu_t_cap=mu_(t-1)+dt*x_t_IMU;
    sigma_t_cap=F_t*sigma_(t-1)*F_t.T+U_t*0.05*Q_t*U_t.T;
    %KF
    x_t_VICON=[vico_p;vico_q;vico_v;vico_w;0;0;0];
    Z_t=awgn(C*x_t_VICON,R_t);
    K_t=sigma_t_cap*C.T*inv(C*sigma_t_cap*C.T+R_t);
    mu_t=mu_t_cap + K_t*(Z_t-C*mu_t_cap);
    sigma_t=sigma_t_cap - K_t*C*sigma_t_cap;
    
end
    
    

    