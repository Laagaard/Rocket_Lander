% TO DO
% add wind and figure out max ranges

clear all;
close all;
clc

body_x_array = zeros(1,1,1);
body_y_array = zeros(1,1,1);

body_x_v_array = zeros(1,1,1);
body_y_v_array = zeros(1,1,1);
body_z_v_array = zeros(1,1,1);

loop_counter = 1;

kpbxr =0.25;    %1 for aerodynamics 
kibxr = 0; % 
kdbxr = 0.5; % 0.08 for aerodynamics 

timingArray = (3.66):(0.001):(3.68);
%timingArray = (3.59):(0.001):(3.62);
v_z_crash = zeros(size(timingArray))
v_x_crash = zeros(size(v_z_crash));
v_y_crash = zeros(size(v_z_crash));
crash_x_angle = zeros(size(timingArray));
crash_y_angle = zeros(size(timingArray));
descent_Motor_burn_altitude = zeros(size(timingArray));
ctr = 1;

for burn_start = timingArray; %s

    % tvc_pid_V2
    %tvc_pid
    %tvc_pid_JustFalling
    %tvc_pid_lookAtDescentTiming
    tvc_pid_V2_KianGainScheduling
    
    figure(1);
    hold on
    plot3(body_x_ctr,body_y_ctr,body_z_ctr)
    plot3(burn_start_x,burn_start_y,burn_start_z,'x')
    % axis equal
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    grid on; %axis equal
    
    figure(2)
    hold on;
    plot(time,body_z_v_ctr)
    xlabel('time (s)'); ylabel('v_z (m/s)')
    
    figure(3)
    hold on
    xlabel('Time (s)')
    ylabel('Angle (Degrees)')
    plot(time,body_x_rot_ctr)
    plot(time,body_y_rot_ctr)
    plot(time,tvc_x_rot_ctr, '--')
    plot(time,tvc_y_rot_ctr, '--')
    plot(time,target_body_x_rot*ones(size(time)),'--k','DisplayName','Target x rotation')
    plot(time,target_body_y_rot*ones(size(time)),'--k','DisplayName','Target y rotation')
    %legend('Body Rotation X','Body Rotation Y','TVC Rotation X','TVC Rotation Y','Location','best')
    
    v_x_crash(ctr) = body_x_v_ctr(end); % m/s
    v_y_crash(ctr) = body_y_v_ctr(end); % m/s
    v_z_crash(ctr) = body_z_v_ctr(end); % m/s    
    crash_x_angle(ctr) = body_x_rot_ctr(end); % deg
    crash_y_angle(ctr) = body_y_rot_ctr(end);
    descent_Motor_burn_altitude(ctr) = burn_start_z;
    ctr = ctr + 1;

    figure(4); hold on; xlabel('time (s)'); ylabel('altitude (m)');
    plot(time,body_z_ctr); grid on;

    figure(5); hold on; xlabel('x [m]'); ylabel('z [m]');
    plot(body_x_ctr,body_z_ctr); grid on;
    burn_start
    figure(1);
    view(-88,13)
    v_z_crash(ctr-1)
    %pause(0.1)

    figure(3);
    %pause(0.1);


end

figure; hold on; xlabel('timing of burn after apogee (s)'); ylabel('crash speed (m/s)'); grid on;
plot(timingArray,v_z_crash)
figure; hold on; xlabel('timing of burn after apogee (s)'); ylabel('crash angle (deg)'); grid on; legend('location','best')
plot(timingArray,crash_x_angle,'DisplayName','x angle')
plot(timingArray,crash_y_angle,'DisplayName','y angle')


minCrashSpeedIndex = find(v_z_crash==-min(abs(v_z_crash)));
minCrashSpeed = v_z_crash(minCrashSpeedIndex); % m/s
tipOverSpeed = sqrt( (v_x_crash(minCrashSpeedIndex))^2 + (v_y_crash(minCrashSpeedIndex))^2); % m/s
optimalBurnTimeAfterApogee = timingArray(minCrashSpeedIndex); % s

disp(sprintf('Minimum crash speed = %.2f m/s.',minCrashSpeed(1)));
disp(sprintf('Corresponding tip-over speed = %.2f m/s.',tipOverSpeed))
disp(sprintf('Optimal timing of descent burn after apogee = %.4f seconds.',optimalBurnTimeAfterApogee(1)));
disp(sprintf('Descent motor burn at %.2f m.',descent_Motor_burn_altitude(minCrashSpeedIndex(1))));
disp(sprintf('With optimal timing x landing angle is %.2f degrees.',crash_x_angle(minCrashSpeedIndex(1))));
disp(sprintf('With optimal timing y landing angle is %.2f degrees.',crash_y_angle(minCrashSpeedIndex(1))));

