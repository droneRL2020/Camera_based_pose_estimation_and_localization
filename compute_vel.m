function [velocity_optical,omega_optical] = compute_vel(visiblePoints,valid_old_points,Z,dt)
    param = [311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];
    
    len = size(visiblePoints);
    A = [];
    B = [];
    for i=1:len(1)
        pos = [visiblePoints(i,1);visiblePoints(i,2);1];
        pos_n = inv(param)*pos;
        
        prev_pos = [valid_old_points(i,1);valid_old_points(i,2);1];
        prev_pos_n = inv(param)*prev_pos;
        
        u = pos_n(1);
        v = pos_n(2);
        
        u_ = prev_pos_n(1);
        v_ = prev_pos_n(2);
        
        u_dot = (u - u_)/dt;
        v_dot = (u - v_)/dt;
        a = [u_dot;v_dot];
        A = [A;a];
        B = [B;-1/Z 0 u/Z (u*v) -1*(1+u*u) v;0 -1/Z v/Z (1+v*v) -1*(u*v) -1*u];
    end

    C = pinv(B)*A;
    velocity_optical = C(1:3);
    omega_optical = C(4:6);
end