function [velocity_optical,omega_optical] = compute_vel(visiblePoints,valid_old_points,Z)
    f = 311.2673;
    len = length(visiblePoints);
    A = [];
    B = [];
    for i=1:len
        u = visiblePoints(i,1);
        v = visiblePoints(i,2);
        u_dot = abs(u - valid_old_points(i,1));
        v_dot = abs(u - valid_old_points(i,2));
        a = [u_dot;v_dot];
        A = [A;a'];
        B = [B;-f/Z 0 u/Z (u*v)/f -1*(f+u*u/f) v;0 -1*f/Z v/Z (f+v*v/f) -1*(u*v/f) -1*u];
    end
    A = [u_dot;v_dot];
    B = [-f/Z 0 u/Z (u*v)/f -1*(f+u*u/f) v;0 -1*f/Z v/Z (f+v*v/f) -1*(u*v/f) -1*u];
    
    
    
    C = pinv(B)*A;
    velocity_optical = C(1:3);
    omega_optical = C(4:6);
end