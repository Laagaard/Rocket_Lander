%%
%{
NOTES and TO DO:
add relative wind

%}
%%

clc; clear; close all; format compact
%
tic
% GLOBAL VARIABLES  
global g_accel burnTimeAscent burnTimeDescent thrustForceAscent thrustForceDescent C_D rho S 
global descentBurn motorImpulse tdescent_burn_start timeToDescend MDOT_Ascent MDOT_Descent
global theta0 windSpeed
% CONSTANTS
thrustCurveAscent = struct2array(load("Aerotech_G25W.mat")); % time (s), force (N)
burnTimeAscent = thrustCurveAscent(:,1); % s
thrustForceAscent = thrustCurveAscent(:,2); % N
descentBurn = false; 
motorImpulse = 120; % N-s
tdescent_burn_start = 100; % sD
PropMass_ascent = 0.0625; % kg
MDOT_Ascent = (thrustForceAscent/norm(thrustForceAscent)) * PropMass_ascent; % kg/s

thrustCurveDescent = thrustCurveAscent;
burnTimeDescent = thrustCurveDescent(:,1); % s
thrustForceDescent = thrustCurveDescent(:,2); % N
PropMass_descent = 0.0625; % kg
MDOT_Descent = (thrustForceDescent/norm(thrustForceDescent)) * PropMass_descent; % kg/s

g_accel = 9.81; % m/s^2
windSpeed = 1.5 * (1609/3600);
C_D = 0.75;
rho = 1.225; % kg/m^3
diam = 3.1 * (2.54/100) ; % m
S = pi*(diam/2)^2; % m^2

% IC's
y0 = 0; % m
ydot0 = 0; % m/s
m0 = 1.50; % kg
theta0 = deg2rad(3.825); % rad, angle between vertical and velocity direction

x0 = 0; % m
xdot0 = 0; % m/s
state0 = [x0 xdot0 y0 ydot0 m0];

tspan = [0 20]; % s
options = odeset(MaxStep=0.001);

%% Initial solution (no descent motor)

[t,states] = ode45(@rates,tspan,state0,options);
range = states(:,1); % m
rangeDot = states(:,2); % m/s
altitude = states(:,3); % m
altitudeDot = states(:,4); % m/s
indices = find(altitude>=0);
rangeDDot = zeros(size(range(indices))); % m/s^2
altitudeDDot = zeros(size(altitude(indices))); % m/s^2

for ctr = 1:length(t)
    state_dot = rates(t(ctr),states(ctr,:));
    rangeDDot(ctr) = state_dot(2); % m/s^2
    altitudeDDot(ctr) = state_dot(4); % m/s^2
end

% Add in wind effect
rangeMin = range - t*windSpeed;
rangeMax = range + t*windSpeed;

%export it
twoD_initial = zeros(length(indices),5);
twoD_initial(:,1) = t(indices);
twoD_initial(:,2) = range(indices); % m
twoD_initial(:,3) = rangeDot(indices); % m/s
twoD_initial(:,4) = altitude(indices); % m
twoD_initial(:,5) = altitudeDot(indices); % m/s
save("twoD_initial.mat","twoD_initial")
%% Graph Stuff
figure; hold on; xlabel('range (m)'); ylabel('altitude (m)'); title('trajectory graph')
plot(range(indices),altitude(indices),'.b')
plot(rangeMax(indices),altitude(indices),'.r','MarkerSize',3)
plot(rangeMin(indices),altitude(indices),'.r','MarkerSize',3)


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

toc
%% FUNCTIONS    

function state_dot = rates(t,states)
    global g_accel burnTimeAscent thrustForceAscent rho C_D S MDOT_Ascent
    global theta0 WindSpeed
    x = states(1);    % m
    y = states(3);     % m
    xdot = states(2); % m/s
    ydot = states(4);  % m/s     
    mass = states(5); % kg

    Tascent = interp1(burnTimeAscent,thrustForceAscent,t,'linear',0); % N
    
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
    xddot = (Tascent-Drag*xdotSign)/mass*sin(theta0); % m/s^2
    yddot = (Tascent-Drag*ydotSign)/mass*cos(theta0) - g_accel; % m/s^2
    mdot = -interp1(burnTimeAscent,MDOT_Ascent,t,'linear',0); % kg/s
    state_dot = [xdot xddot ydot yddot mdot]';
end
