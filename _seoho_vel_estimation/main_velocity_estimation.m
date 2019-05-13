clear;
clc;
load('studentdata4.mat');

velocities = zeros(3, numel(data));
angular_velocities = zeros(3, numel(data));
vicon_velocities = zeros(3, numel(data));
vicon_angular_velocities = zeros(3, numel(data));
data_time = zeros(1,numel(data));

for i = 1:numel(data)
    data_time(i) = data(i).t;
    if numel(data(i).id)
        [velocity, angular_velocity] = velocity_estimation(data(i));
        [~, id] = min(abs(time - data(i).t));
        vicon_velocity = vicon(:, id);
        
        velocities(:, i) = velocity;
        angular_velocities(:, i) = angular_velocity;
        vicon_velocities(:, i) = vicon_velocity(7:9);
        vicon_angular_velocities(:, i) = vicon_velocity(10:end);
    end
end
plot_linear_angular_velocities(velocities, angular_velocities, vicon_velocities, vicon_angular_velocities, data_time);
