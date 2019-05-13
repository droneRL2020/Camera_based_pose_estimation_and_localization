function plot_linear_angular_velocities(velocities, angular_velocities, vicon_velocities, vicon_angular_velocities, data_time)
start = 1;
figure; 
subplot(4,2,1); 
plot(data_time(start:end), velocities(1,start:end));
hold on;
plot(data_time(start:end), vicon_velocities(1,start:end));
hold off;
title('linear velocity x')
legend('estimated', 'vicon');

subplot(4,2,3);
plot(data_time(start:end), velocities(2,start:end));
hold on;
plot(data_time(start:end), vicon_velocities(2,start:end));
hold off;
title('linear velocity y');
legend('estimated', 'vicon');

subplot(4,2,5);
plot(data_time(start:end), velocities(3,start:end));
hold on;
plot(data_time(start:end), vicon_velocities(3,start:end));
hold off;
title('linear velocity z');
legend('estimated', 'vicon');

subplot(4,2,7);
MSE = mean((velocities - vicon_velocities).*2);
plot(data_time(start:end), MSE(start:end));
title('mean squared error');

subplot(4,2,2);
plot(data_time(start:end), angular_velocities(1,start:end));
hold on;
plot(data_time(start:end), vicon_angular_velocities(1,start:end));
hold off;
title('angular velocity x');
legend('estimated', 'vicon');

subplot(4,2,4);
plot(data_time(start:end), angular_velocities(2,start:end));
hold on;
plot(data_time(start:end), vicon_angular_velocities(2,start:end));
hold off;
title('angular velocity y');
legend('estimated', 'vicon');

subplot(4,2,6);
plot(data_time(start:end), angular_velocities(3,start:end));
hold on;
plot(data_time(start:end), vicon_angular_velocities(3,start:end));
hold off;
title('angular velocity z');
legend('estimated', 'vicon');

subplot(4,2,8);
mse = mean((angular_velocities - vicon_angular_velocities).*2);
plot(data_time(start:end), mse(start:end));
title('mean squared error');

end