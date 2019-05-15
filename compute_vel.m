function [velocity_optical,omega_optical] = compute_vel(visiblePoints,valid_old_points,Z,wRc,dt)
    param = [311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];
    
    len = size(visiblePoints);
    A = [];
    B = [];
    D=[];
    E=[];
    COUNT=[];
    A_TEMP = [];
    B_TEMP = [];
    VEL_TEMP_OPTIM=[]; 
    K=12;
    LIST=[];
    a_inliers = [];
    b_inliers = [];
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
        v_dot = (v - v_)/dt;
        a = [u_dot;v_dot];
        
        B = [B;-1/Z 0 u/Z (u*v) -1*(1+u*u) v;0 -1/Z v/Z (1+v*v) -1*(u*v) -1*u];
        A = [A;a];
       

    end
        D=A;
        E=B;
    for n=1:K    
        ran=randperm(len(1),3);
        U_V_1=[D(2*ran(1)-1);D(2*ran(1))];
        U_V_2=[D(2*ran(2)-1);D(2*ran(2))];
        U_V_3=[D(2*ran(3)-1);D(2*ran(3))];
        
        A_TEMP=[U_V_1;U_V_2;U_V_3];
        
        B1=[E(2*ran(1)-1,:);E(2*ran(1),:)];
        B2=[E(2*ran(2)-1,:);E(2*ran(2),:)];
        B3=[E(2*ran(3)-1,:);E(2*ran(3),:)];
        
        B_TEMP=[B1;B2;B3];
        
        VEL_TEMP_OPTIM(:,n)=(B_TEMP.'*B_TEMP)\(B_TEMP.')*A_TEMP;
        cot=0;
        for j=1:len(1)
            pos = [visiblePoints(j,1);visiblePoints(j,2);1];
            pos_n = inv(param)*pos;
            
            prev_pos = [valid_old_points(j,1);valid_old_points(j,2);1];
            prev_pos_n = inv(param)*prev_pos;
            
            u = pos_n(1);
            v = pos_n(2);
            
            u_ = prev_pos_n(1);
            v_ = prev_pos_n(2);
            
            u_dot = (u - u_)/dt;
            v_dot = (v - v_)/dt;
            a = [u_dot;
                v_dot];
            b = [-1/Z 0 u/Z (u*v) -1*(1+u*u) v;
                0 -1/Z v/Z (1+v*v) -1*(u*v) -1*u];
            
            NORM=norm(a-b*VEL_TEMP_OPTIM);
            
            if (NORM^2)<0.1
                cot=cot+1;
                a_inliers = [a_inliers;a];
                b_inliers = [b_inliers;b];
            end
        end

        COUNT(n)=cot;
        VEL_OPTIM(:,n)=(b_inliers.'*b_inliers)\(b_inliers.')*a_inliers;
    end
    
    LIST=[VEL_OPTIM;COUNT];
    C1 = [0;0;0;0;0;0];
    max = 1;
    for m=1:K
        if LIST(7,m)>LIST(7,max)
           max = m;
        end
    end
    %%find the largest VEL_OPTIM=C1
    C1 = LIST(1:6,max);
    %C1 = (B.'*B)\(B.')*A;

    
%     size(C1)
    velocity_optical = wRc*C1(1:3);
    omega_optical = wRc*C1(4:6);
end