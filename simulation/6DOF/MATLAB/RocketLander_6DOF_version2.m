
% RocketLander_6DOF_version1

% ===============================
% Symbol and Conventions Section

% ===============================
% clc; 
% clear
% close all;

tic

% ===============================
% List of all shared variables
% (Initialize now so we know to share with nested functions)
global x0 xdot0_body xdot0_inertial y0 ydot0_body ydot0_inertial z0 zdot0_body zdot0_inertial psi0 theta0 phi0 P0 Q0 R0 m0 I_xx0 I_yy0 I_zz0 C_G0 C_P0 thetaDot0 C_D S 
global thrustCurve ratesOptions rho S C_D g_accel t_fire burnTime
global propellantMass DesiredRange DesiredCrashSpeed animateOptions

% ===============================
% Input Section

% Load Trajectory parameters
Parameters_Trajectory

% Load Rocket parameters
Parameters_Rocket

% Load Motor parameters
Parameters_Motor

% Load Earth Data
Parameters_Earth

% Launch Angle Determination ODE45 options
tspan = [0 20]; % (s)
options = odeset('MaxStep',2.5E-1);
% Set options for running rates function
ratesOptions = [0 0];
% ratesOptions(1)   -   are we firing descent motor?
% ratesOptions(2)   -   are we using TVC?

% Setup Animation
global DART_stl orientation_animation trajectory_animation pos_tracked_X pos_tracked_Y pos_tracked_Z xdot_body ydot_body zdot_body
setup_animation(eul2quat([psi0 theta0 phi0]))

initial_launch_angle = pi/2 - abs(pi/2 - (theta0)) % [rad]

% iterate through potential launch angles
for pitch_angle = rad2deg(initial_launch_angle)
    launch_angle = pitch_angle % set launch angle to theta
    Y0 = [x0 xdot0_inertial y0 ydot0_inertial z0 zdot0_inertial psi0 theta0 phi0 P0 Q0 R0 m0 I_xx0 I_yy0 I_zz0 C_G0]';
    % Launch rocket and calculate landing range
    [t,y] = ode45(@sixDOF_v2,tspan,Y0,options);

    indices = find(y(:,5)>-0.2); % all states with positive altitude
    range = sqrt(y(indices(end),1)^2 + y(indices(end),3)^2);   % (m)
    
    % Check if desired range condition is satisfied within tolerance
    % if (range > DesiredRange)
        % set launch pitch angle
        thetaLaunch = launch_angle; % (degrees)
        freeFlightTime = t;
        freeFlightStates = y;
        fprintf('Desired range possible with launch angle of %.2f degrees.\n', thetaLaunch)

        figure()
        plot3(freeFlightStates(indices,1), freeFlightStates(indices,3), freeFlightStates(indices,5),'b')
        hold on
        [~, index_burnout] = min(abs(freeFlightTime - burnTime));
        plot3(freeFlightStates(index_burnout,1),freeFlightStates(index_burnout,3),freeFlightStates(index_burnout,5),'.r',MarkerSize=10)
        grid minor
        xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
        title('3D Trajectory')
        print('time versus trajectory.png','-dpng','-r300')
        freeFlightStates(:,5) = atan2d(freeFlightStates(:,4),freeFlightStates(:,2)); % (degrees)
        % break
    % end

end

close(orientation_animation)
close(trajectory_animation)

%% Descent Motor Ignition Time Determination
index_Apogee = find(freeFlightStates(:,5)==max(freeFlightStates(indices,5)));
index_Crash = indices(end);

time_Apogee = freeFlightTime(index_Apogee)
time_Crash = freeFlightTime(index_Crash)