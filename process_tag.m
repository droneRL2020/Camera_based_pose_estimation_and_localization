function pose = process_tag(data,ctr)
    R = 0;
    T = 0;
    k = [311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];
    k_inv = inv(k);
    points = ["p0" "p1" "p2" "p3" "p4"];
    A = [];
    for i = 1:length(data(ctr).id)
        for j = 1:5
            [p,q] = find_tag(data(ctr).id(i));
            [x,y] = tag_corner(p,q,points(j));
            world_cord = data(ctr).p0(:,i);
            x_c = world_cord(1);
            y_c = world_cord(2);
            A = [A(:,:);x y 1 0 0 0 -1*x_c*x -1*x_c*y -1*x_c;0 0 0 x y 1 -1*y_c*x -1*y_c*y -1*y_c];
        end
    end    
    pose = 0;
    %A = [x y 1 0 0 0 -1*x_c*x -1*x_c*y -1*x_c;0 0 0 x y 1 -1*y_c*x -1*y_c*y -1*y_c];
    [U,S,V] = svd(A);
    V
    size(V)
    V_9 = V(:,9);
    
    h = [V_9(1) V_9(4) V_9(7);V_9(2) V_9(5) V_9(8);V_9(3) V_9(6) V_9(9)];
    if V_9(9) < 0
        h = h*(-1);
    end
    
    temp = k_inv * h;
    
    [u,s,v] = svd(temp);
    
    R = u * [1 0 0;0 1 0;0 0 det(u*(v.'))] * (v.');
    
    
    R1_hat = [temp(1,1);temp(2,1);temp(3,1)];
    %R2 = [temp(1,2);temp(2,2);temp(3,2)];
    T_hat = [temp(1,2);temp(2,2);temp(3,2)];
    
    T = T_hat./norm(R1_hat);
    
    bHc = [0.707 -0.707 0 -0.04;
        -0.707 -0.707 0 0;
        0 0 -1 -0.03;
        0 0 0 1];
    
    cHw = [R(1,1) R(1,2) R(1,3) T(1);R(2,1) R(2,2) R(2,3) T(2);R(3,1) R(3,2) R(3,3) T(3);0 0 0 1];
    
    %bHw = bHc*cHw;
    
    pose = inv(cHw)*bHc;
    
    r = [pose(1,1) pose(1,2) pose(1,3);pose(2,1) pose(2,2) pose(2,3);pose(3,1) pose(3,2) pose(3,3)];
    j = rotm2eul(r);
    j
    %alpha = atan2d(pose(2,1),pose(1,1))
end