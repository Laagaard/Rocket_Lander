% Luke Aagaard
% AEE 3150 Team Project
% March 28, 2024
clc; clear; close all; format compact;

t = readmatrix('Altitude Calculations_angles.xlsx','Range','Q2:Q1580');
ax = readmatrix('Altitude Calculations_angles.xlsx','Range','ad2:ad1580');
ay = 0*ax;
az = readmatrix('Altitude Calculations_angles.xlsx','Range','ae2:ae1580');

acc = [ax ay az-9.806];
angVel = [ay ay ay];

IMU = imuSensor('accel-gyro');
IMU.SampleRate = 100; % hz
IMU.Accelerometer.NoiseDensity = [1 1 1]*0.5;
IMU.Gyroscope.NoiseDensity = [1 1 1]*0.5;
[accelReadings, gyroReadings] = IMU(-acc,-angVel)

AccelerationData = [t accelReadings gyroReadings];

figure
hold on
plot(AccelerationData(:,1),AccelerationData(:,2))
plot(AccelerationData(:,1),AccelerationData(:,3))
plot(AccelerationData(:,1),AccelerationData(:,4))
ylabel('Acceleration (m/s^2)'); xlabel('time (s)'); title('Accelerometer Linear Acceleration Data')
legend('A_x (m/s^2)','A_y (m/s^2)','A_z (m/s^2)','location','NorthEast')

figure
hold on
plot(AccelerationData(:,1),AccelerationData(:,5))
plot(AccelerationData(:,1),AccelerationData(:,6))
plot(AccelerationData(:,1),AccelerationData(:,7))
ylabel('Angular Velocity (rad/s)'); xlabel('time (s)'); title('Gyroscope Angular Velocity Data')
legend('\omega_x (rad/s)','\omega_y (rad/s)','\omega_z (rad/s)','location','NorthEast')
save("Acceleration Data", "AccelerationData")
