function [vel, omg] = estimate_vel_draft(sensor)
    persistent points t td;
    K=[311.0520   0       201.8724;...
          0   311.3885    113.6210;...
          0       0             1];
    
    
    if isempty(sensor.id)
        vel = [];
        omg = [];
        t = sensor.t;
    else
        % Initialization
        if isempty(points)
            points = detectFASTFeatures(sensor.img); 
            points = points.selectStrongest(50); 
            prev_points = points.Location;
            tracker = vision.PointTracker('MaxBidirectionalError',0.7);
            initialize(tracker,prev_points,sensor.img); 
            
            td = sensor.t - t;
            vel = zeros(3,1);
            omg = zeros(3,1);
        %Update
        else
            prev_t = t; 
            prev_td = td;
            points = detectFASTFeatures(sensor.img); 
            points = points.selectStrongest(50); 
            prev_points = points.Location;
            tracker = vision.PointTracker('MaxBidirectionalError',0.7);
            initialize(tracker,prev_points,sensor.img); 
            
            [points, accuracy] = step(tracker, sensor.img);
            
            C=K\[double(points) ones(size(points,1),1)]';
            camera_points=C(1:2,:)';
    
            C_prev=K\[double(prev_points) ones(size(prev_points,1),1)]';
            prev_camera_points = C_prev(1:2,:)';
            
            opti_vel=(camera_points-old_camera_points)/td;
    
            
            % H = Homogeneous Matrix from pose estimation
            
            A=[];   B=[];
    
            for j = 1:size(points,1) 
                x=camera_points(j,1);   
                y=camera_points(j,2);
                A=[A;
                   -1/Z(j),0      ,x/Z(j),x*y    ,-(1+x^2),y;
                   0      ,-1/Z(j),y/Z(j),(1+y^2),-x*y    ,-x];
                B=[B;
                   opti_vel(j,1);
                   opti_vel(j,2)];
            end
            V=A\B;
            
            if(isempty(V))
                vel=zeros(3,1);
                omg=zeros(3,1);
            else
                vel = H(1:3,1:3)*V(1:3);
                omg = H(1:3,1:3)*V(4:6);
            end
        end
    end
    end