% Luke Aagaard
% AEE 3150 Team Project
% March 28, 2024
clc; clear; close all; format compact;
Simulation_Time = 13; %s
sim('RocketLander_version3.mdl',Simulation_Time)

AccelerationData = load('Non_Noisy_Data.mat');
AccelerationData = struct2array(AccelerationData);
AccelerationData = AccelerationData'

acc = AccelerationData(:,2:4); %m/s
angVel = AccelerationData(:,5:7); %rad/s
IMU = imuSensor('accel-gyro');
IMU.SampleRate = 100; % hz
IMU.Accelerometer.NoiseDensity = [1 0 1]*0.1;
IMU.Gyroscope.NoiseDensity = [0 1 0]*0.001;
[accelReadings,gyroReadings] = IMU(-acc,-angVel)

AccelerationData = [AccelerationData(:,1) accelReadings gyroReadings];

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
ylabel('Angular Velocity (rad/s)'); xlabel('time (s)'); title('Gyroscope Angular Velocity Data')
legend('\omega_x_b (rad/s)','\omega_y_b (rad/s)','\omega_z_b (rad/s)','location','NorthEast')
save("Acceleration Data", "AccelerationData")
