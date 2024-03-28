% Luke Aagaard
% AEE 3150 Team Project
% March 28, 2024
clc; clear; close all; format compact;
Simulation_Time = 13; %s
sim('RocketLander_version3.mdl',Simulation_Time)
AccelerationData = load('IMU_noisy_data.mat')
AccelerationData = struct2array(AccelerationData)
AccelerationData = AccelerationData';
AccelerationData
figure
hold on
plot(AccelerationData(:,1),AccelerationData(:,2))
plot(AccelerationData(:,1),AccelerationData(:,3))
plot(AccelerationData(:,1),AccelerationData(:,4))
ylabel('Acceleration (m/s^2)'); xlabel('time (s)'); title('Accelerometer Linear Acceleration Data')
legend('A_x_b (m/s^2)','A_y_b (m/s^2)','A_z_b (m/s^2)','location','NorthEast')

figure
hold on
plot(AccelerationData(:,1),AccelerationData(:,5))
plot(AccelerationData(:,1),AccelerationData(:,6))
plot(AccelerationData(:,1),AccelerationData(:,7))
ylabel('Angular Acceleration (rad/s^2)'); xlabel('time (s)'); title('Gyroscope Angular Acceleration Data')
legend('\alpha_x_b (rad/s^2)','\alpha_y_b (rad/s^2)','\alpha_z_b (rad/s^2)','location','NorthEast')
