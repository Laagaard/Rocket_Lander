clc; clear; close all; format compact
%
tic
% GLOBAL VARIABLES  
global g_accel burnTimeDescent thrustForceDescent C_D rho S 
global descentBurn motorImpulse tdescent_burn_start timeToDescend 
global MDOT_Ascent MDOT_Descent l_t I_zz

% CONSTANTS
thrustCurveDescent = struct2array(load("Aerotech_G25W.mat")); % time (s), force (N)
burnTimeDescent = thrustCurveDescent(:,1); % s
thrustForceDescent = thrustCurveDescent(:,2); % N
descentBurn = false; 
motorImpulse = 120; % N-s
tdescent_burn_start = 100; % sD
PropMass_descent = 0.0625; % kg
PropMass_ascent = PropMass_descent; % kg
MDOT_Descent = (thrustForceDescent/norm(thrustForceDescent)) * PropMass_descent; % kg/s

g_accel = 9.81; % m/s^2
C_D = 0.75;
rho = 1.225; % kg/m^3
diam = 3.1 * (2.54/100) ; % m
S = pi*(diam/2)^2; % m^2
l_t = 0.5; % m
I_zz = .05; % kg-m^2   


    % IC's
    y0 = 62.3; % m
    ydot0 = -38.5; % m/s
    m0 = 1.50 - PropMass_ascent; % kg
    x0 = 45.9; % m
    xdot0 = 4.3; % m/s
    theta0 = pi/2 - atan2(ydot0,xdot0); % rad, angle between vertical and velocity direction
    thetaDot0 = 0; % rad/s
    errorIntegr0 = 0; % radian-s
    errorIntegr02 = 0; % m-s
    tspan = [0 5]; % s
    options = odeset(MaxStep=0.01);
    state0 = [x0 xdot0 y0 ydot0 theta0 thetaDot0 m0 errorIntegr0 errorIntegr02];
    
    [t,states] = ode45(@rates_descent,tspan,state0,options);
    range = states(:,1); % m
    rangeDot = states(:,2); % m/s
    altitude = states(:,3); % m
    altitudeDot = states(:,4); % m/s
    theta = states(:,5); % rad
    thetaDot = states(:,6); % rad/s
    indices = find(altitude>=0);
    rangeDDot = zeros(size(range)); % m/s^2
    altitudeDDot = zeros(size(altitude)); % m/s^2
    thetaDDot = zeros(size(altitude)); % rad/s^2
    error = zeros(size(altitude)); % rad

    for ctr = 1:length(t)
        state_dot = rates_descent(t(ctr),states(ctr,:));
        rangeDDot(ctr) = state_dot(2); % m/s^2
        altitudeDDot(ctr) = state_dot(4); % m/s^2
        thetaDDot(ctr) = state_dot(6); % rad/s^2
        error(ctr) = state_dot(8); % rad
    end

    %Plot stuff
    figure; hold on; xlabel('time (s)'); ylabel('angular stuff')
    plot(t(indices),theta(indices),'-.')
    plot(t(indices),thetaDot(indices),':')
    %plot(t(indices),thetaDDot(indices),'--')
    ylim([-10 10])
    legend('angular position','angular velocity','angular acceleration','Location','best')
    plot(t,t*0)


    figure; hold on; xlabel('x (m)'); ylabel('y (m)')
    plot(range(indices),altitude(indices))


    figure; hold on; xlabel('time (s)'); ylabel('vertical component'); 
    plot(t(indices),altitude(indices))
    plot(t(indices),altitudeDot(indices))
    plot(t(indices),altitudeDDot(indices))
    legend('position (m)','velocity (m/s)','acceleration (m/s^2)')
    title('vertical component')

    figure; hold on; xlabel('time (s)'); ylabel('range component'); 
    plot(t(indices),range(indices))
    plot(t(indices),rangeDot(indices))
    plot(t(indices),rangeDDot(indices))
    legend('position (m)','velocity (m/s)','acceleration (m/s^2)')
    title('range component')
    plot(t(indices),50*ones(size(t(indices))))

    disp(sprintf('For solution:\n\tvertical landing speed = %.2f m/s.',altitudeDot(indices(end))))
    disp(sprintf('\thorizontal landing speed = %.2f m/s.\n\tlanding angle from vertical = %.2f degrees.',[rangeDot(indices(end)) rad2deg(theta(indices(end)))]))

toc





%% FUNCTIONS    
function state_dot = rates_descent(t,states)
    global g_accel burnTimeDescent thrustForceDescent rho C_D S MDOT_Descent
    global l_t I_zz
    x = states(1);    % m
    y = states(3);     % m
    xdot = states(2); % m/s`
    ydot = states(4);  % m/s     
    theta = states(5); % rad
    thetaDot = states(6); % rad/s
    mass = states(7); % kg
    errorIntegr = states(8); % radian-s
    errorIntegr2 = states(9); % m-s

    Tdescent = -interp1(burnTimeDescent,thrustForceDescent,t,'linear',0); % N
   
    
        % construction of PID 1 (for angle)
    referenceTheta       = 0;     % reference theta desired (straight down)
    e       = theta - referenceTheta;   % error signal
    Kp      = 100;                  % proportional gain
    Ki      = 1600;                            % integral gain
    Kd      = 2.25;                  % derivative gain
    u1       = - Kp*e - Ki*errorIntegr - Kd*thetaDot;   % the PID thing
    
            % construction of PID 2 (for range position)    
    referenceX       = 50;     % reference range desired (50 m)
    e2       = x - referenceX;   % error signal
    Kp      = 75;                  % proportional gain
    Ki      = 15;                            % integral gain
    Kd      = .85;                  % derivative gain
    u2       = (- Kp*e2 - Ki*errorIntegr2 - Kd*xdot);   % the PID thing
    
    errorIntegrDot = e; % rad
    errorIntegrDot2 = e2; % m

    waitTime = 0;%.125; % s
    changeAltitude = 1; % m
    if t>=waitTime && y>=changeAltitude
        phi = u1+u2; % range and pitch
    elseif t>=waitTime
        phi = u1; % just pitch
    else
        phi = 0;
    end

    thetaDDot = Tdescent*l_t*sin(phi)/I_zz; % rad/s^2
    Drag = (1/2*rho*(xdot^2 + ydot^2))*S*C_D; % N
    if xdot>=0
        xdotSign = 1;
    else
        xdotSign = -1;
    end
    if ydot>=0
        ydotSign = 1;
    else
        ydotSign = -1;
    end

    xddot = Tdescent/mass*sin(theta+phi) - Drag*xdotSign/mass*sin(theta); % m/s^2
    yddot = Tdescent/mass*cos(theta+phi) - Drag*ydotSign/mass*cos(theta) - g_accel; % m/s^2
    mdot = -interp1(burnTimeDescent,MDOT_Descent,t,'linear',0); % kg/s
    state_dot = [xdot xddot ydot yddot thetaDot thetaDDot mdot errorIntegrDot errorIntegrDot2]';
end