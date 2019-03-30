
p = [0 ; 0 ; 0];
q = [0 ; 0 ; 0];
p_dot = [0 ; 0 ; 0];
gyro_bias = [0 ; 0 ; 0];
acc_bias = [0 ; 0 ; 0];

C = [ 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0  ; 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 ; 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 ;  0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 ; 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 ; 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0] ;

x = [p ; q ; p_dot ; gyro_bias ; acc_bias];

z = C*x;

phi_time_current = 0;
theta_time_current = 0;
psi_time_current = 0;

phi_time_current = 0;
theta_time_current = 0;
psi_time_current = 0;


G = g(phi_tc, theta_tc, psi_tc);
R = r(phi_tc, theta_tc, psi_tc);

% rot_mat = rotx(q(1)) * roty(q(2)) * rotz(q(3));
% 
% eul_mat = rotm2eul(rot_mat);
omg_imu = [0 ;0 ; 0;];

omega = G * omg_imu;

