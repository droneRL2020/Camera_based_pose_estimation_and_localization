function [velocity, angular_velocity] = velocity_estimation(data)

persistent image_i prev_data prev_points prev_x prev_y prev_velocity prev_angular_velocity pointTracker K inv_k; 

% Initialization
if isempty(prev_data)
    image_i = 1;
    prev_dt = 0.5;
    prev_velocity = zeros(3,1);
    prev_angular_velocity = zeros(3,1);
    velocity = zeros(3,1);
    angular_velocity = zeros(3,1);
    K = [311.0520 0        201.8724;
        0         311.3885 113.6210;
        0         0        1       ];
    inv_k = inv(K);
    
    points = detectFASTFeatures(data.img);
    pointTracker = vision.PointTracker('MaxBidirectionalError', 1);
    points = points.selectStrongest(300).Location;
    initialize(pointTracker, points, data.img);
    [length,width]=size(data.img);
    true_index = points(:,1) > width*0 & points(:,1) < width*1 & points(:,2) > length*0 & points(:,2) < length*1;
    points = points(true_index,:);
    prev_points = points;
    prev_data = data;    
    return
end

% Update
points = step(pointTracker, data.img);
[length,width]=size(data.img);
true_index = points(:,1) > width*0 & points(:,1) < width*1 & points(:,2) > length*0 & points(:,2) < length*1;
points = points(true_index,:);
prev_points = prev_points(true_index,:);
dt = data.t - prev_data.t;

points_w = tag_points(data.id); 
points_c = [data.p0, data.p1, data.p2, data.p3, data.p4];
[wRc, wTc] = RT_estimation(points_c, points_w, inv_k);
z_c = z_c_estimation(wRc, wTc, points, inv_k);

temp=K\[points ones(size(points,1),1)]';
camera_points=temp(1:2,:)';
x = camera_points(:,1);
y = camera_points(:,2);
x = x';
y = y';
temp=K\[prev_points ones(size(points,1),1)]';
prev_camera_points=temp(1:2,:)';
prev_x = prev_camera_points(:,1);
prev_y = prev_camera_points(:,2);
prev_x = prev_x';
prev_y = prev_y';

dot_x = [x - prev_x]'./dt;
dot_y = [y - prev_y]'./dt;

n = numel(x');
A_B = [-1./z_c',  zeros(n, 1), x'./z_c', x'.*y',   -(1+x'.^2), y';
         zeros(n,1),-1./z_c',    y'./z_c', (1+y'.^2),-x'.*y',-x'];

linear_angular_velocities_c = A_B\[dot_x;dot_y];

if mod(image_i, 2) == 0
    points = detectFASTFeatures(data.img);
    points = points.selectStrongest(300).Location;
    [length,width]=size(data.img);
    true_index = points(:,1) > width*0 & points(:,1) < width*1 & points(:,2) > length*0 & points(:,2) < length*1;
    points = points(true_index,:);

    points_w = tag_points(data.id); 
    points_c = [data.p0, data.p1, data.p2, data.p3, data.p4];
    [wRc, wTc] = RT_estimation(points_c, points_w, inv_k);
    z_c = z_c_estimation(wRc, wTc, points, inv_k);
end
setPoints(pointTracker, points);

image_i = image_i + 1;
prev_points = points;
prev_data = data;
prev_dt = dt;

% Camera to World
velocity = wRc'*linear_angular_velocities_c(1:3);
angular_velocity = wRc'*linear_angular_velocities_c(4:6);
prev_velocity = velocity;
prev_angular_velocity = angular_velocity;
end