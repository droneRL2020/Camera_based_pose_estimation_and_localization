function [A_t,U_t,X_dot] = calc_jacobian(wm_x, wm_y, wm_z, am_x, am_y, am_z,px,py,pz,phi,theta,psi,p_dot_x,p_dot_y,p_dot_z,bg_x,bg_y,bg_z,ba_x, ba_y, ba_z)
    syms p q p_dot bg ba px py pz phi theta psi p_dot_x p_dot_y p_dot_z bg_x bg_y bg_z ba_x ba_y ba_z
    syms wm_x wm_y wm_z ng_x ng_y ng_z am_x am_y am_z na_x na_y na_z nbg_x nbg_y nbg_z nba_x nba_y nba_z 

    p = [px ; py ; pz];
    q = [phi ; theta ; psi];
    p_dot = [p_dot_x ; p_dot_y ; p_dot_z];
    gyro_bias = [bg_x ; bg_y ; bg_z];
    acc_bias = [ba_x ; ba_y ; ba_z];

    X = [p ; q ; p_dot ; gyro_bias ; acc_bias];

    G = inv(g(q(1),q(2),q(3)));
    R = r(q(1),q(2),q(3));
   %wm = U(1:3);
    wm = [wm_x ; wm_y ; wm_z];
    ng = [0;0;0];
    am = [am_x ; am_y ; am_z];
    na = [0;0;0]; 
    nbg = [0;0;0]; 
    nba = [0;0;0]; 
    grav = [0 ; 0 ; -9.81];

    X_dot = [p_dot ; G*( wm - gyro_bias - ng) ; grav + (R*(am - acc_bias - na)) ; nbg ; nba];

    n = [ng_x;ng_y;ng_z;na_x;na_y;na_z;nbg ; nba];

    A_t = jacobian(X_dot,X);
    U_t = jacobian(X_dot,n);

end