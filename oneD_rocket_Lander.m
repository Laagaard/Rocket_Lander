%%
%{
NOTES and TO DO:

    * What altitude is optimal to ignite at?

    * How do you make into 2D motion?

%}
%%

clc; clear; close all; format compact
%
tic
% GLOBAL VARIABLES  
global g_accel burnTimeAscent burnTimeDescent thrustForceAscent thrustForceDescent C_D rho S descentBurn motorImpulse tdescent_burn_start timeToDescend MDOT_Ascent MDOT_Descent

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
C_D = 0.75;
rho = 1.225; % kg/m^3
diam = 3.1 * (2.54/100) ; % m
S = pi*(diam/2)^2; % m^2

% IC's
y0 = 0; % m
ydot0 = 0; % m/s
m0 = 1.50; % kg
state0 = [y0 ydot0 m0];

tspan = [0 25]; % s
options = odeset(MaxStep=0.01);

%% Initial solution (no descent motor)

[t,states] = ode45(@rates,tspan,state0,options);
altitude = states(:,1); % m
velocity = states(:,2); % m/s
indices = find(altitude>=0);
acceleration = zeros(size(altitude(indices))); % m/s^2
for ctr = 1:length(indices)
    state_dot = rates(t(ctr),states(ctr,:));
    acceleration(ctr) = state_dot(2); % m/s^2
end

%% Solution With Descent Motor
% find apogee time and altitude
apogee = max(altitude); % m
ApogeeIndex = find(altitude==apogee);
timeApogee = t(ApogeeIndex); % s
timeLand = t(indices(end)); % s

figure; tiledlayout("flow")
tevaluate = 11.7:0.001:11.9;
landingVelocity = ones(size(tevaluate));
% iterate through times to find ignition time that minimizes landing
% velocity
for outer_ctr = 1:length(tevaluate)
        tdescent_burn_start = tevaluate(outer_ctr);
        [t2,states2] = ode45(@rates2,tspan,state0,options);
        altitude2 = states2(:,1); % m   
        velocity2 = states2(:,2); % m/s
        indices2 = find((t2>burnTimeAscent(end))+(altitude2<=0)==2); % indices of values not to use
        acceleration2 = zeros(size(altitude2(1:1:indices2(1)-1))); % m/s^2

        for inner_ctr = 1:indices2(1) % indices of values to use
            state_dot2 = rates2(t2(inner_ctr),states2(inner_ctr,:));
            acceleration2(inner_ctr) = state_dot2(2); % m/s^2
        end
        
        if mod(outer_ctr,5)==0
            nexttile
            hold on;
            plot(t2(1:indices2(1)),altitude2(1:indices2(1)),'.')
            plot(t2(1:indices2(1)),velocity2(1:indices2(1)),'.')
            plot(t2(1:indices2(1)),acceleration2(1:indices2(1)),'.')
            xlim([(t2(indices2(1))-4) t2(indices2(1))])
        end

        landingVelocity(outer_ctr) = velocity2(indices2(1)); % m/s  
end


% Optimal timing
tol = 1E-3;
index = find(abs(landingVelocity+min(abs(landingVelocity)))<=tol);
tdescent_burn_start_optimal = tevaluate(index); % s
tdescent_burn_start = tdescent_burn_start_optimal; % s
disp(sprintf('Minimum landing velocity is %.2f m/s.',landingVelocity(index)))

% Evaluate trajectory for optimal timing solution
[t_optimal,states_optimal] = ode45(@rates2,tspan,state0,options);
altitude_optimal = states_optimal(:,1); % m
velocity_optimal = states_optimal(:,2); % m/s
mass_optimal = states_optimal(:,3); % kg

acceleration_optimal = zeros(size(t_optimal)); % m/s^2
for ctr = 1:length(t_optimal)
    state_dot_optimal = rates2(t_optimal(ctr),states_optimal(ctr,:));
    acceleration_optimal(ctr) = state_dot_optimal(2); % m/s^2
end


%% GRAPH STUFF

figure; hold on; xlabel('time (s)'); ylabel('landing velocity (m/s)')
title('timing of descent motor burn')
plot(tevaluate,landingVelocity,'.')
xx=1:50; yy = 1.0007*xx;
plot(xx,yy,'-r')

figure; hold on; xlabel('time (s)'); ylabel('vertical component'); 
title('optimal timing of descent motor burn')
plot(t_optimal(1:indices_optimal(1)),altitude_optimal(1:indices_optimal(1)),'.')
plot(t_optimal(1:indices_optimal(1)),velocity_optimal(1:indices_optimal(1)),'.')
plot(t_optimal(1:indices_optimal(1)),acceleration_optimal(1:indices_optimal(1)),'.')
legend('altitude (m)','velocity (m/s)','acceleration (m/s^2)')

figure; hold on; xlabel('time (s)'); ylabel('mass (kg)')
title('mass of rocket with optimal timing of descent motor')
plot(t_optimal(1:indices_optimal(1)),mass_optimal(1:indices_optimal(1)))
toc
%% FUNCTIONS    

function state_dot = rates(t,states)
    global g_accel burnTimeAscent thrustForceAscent rho C_D S MDOT_Ascent
    y = states(1);    % m
    ydot = states(2); % m/s
    mass = states(3); % kg

    Tascent = interp1(burnTimeAscent,thrustForceAscent,t,'linear',0); % N
    Drag = (1/2*rho*ydot*abs(ydot))*S*C_D; % N
    yddot = (Tascent-Drag)/mass - g_accel; % m/s^2
    mdot = -interp1(burnTimeAscent,MDOT_Ascent,t,'linear',0); % kg/s
    state_dot = [ydot yddot mdot]';
end

function state_dot = rates2(t,states)
    global g_accel burnTimeAscent burnTimeDescent thrustForceAscent thrustForceDescent tdescent_burn_start rho C_D S MDOT_Ascent MDOT_Descent
    y = states(1);    % m
    ydot = states(2); % m/s
    mass = states(3); % kg

    Drag = (1/2*rho*ydot*abs(ydot))*S*C_D; % N
    Tascent = interp1(burnTimeAscent,thrustForceAscent,t,'linear',0); % N
    mdotAscent = interp1(burnTimeAscent,MDOT_Ascent,t,'linear',0); % kg/s

    if t>=tdescent_burn_start
        Tdescent = interp1(burnTimeDescent,thrustForceDescent,t-tdescent_burn_start,'linear',0); % N
        mdotDescent = interp1(burnTimeDescent,MDOT_Descent,t-tdescent_burn_start,'linear',0); % kg/s
    else
        Tdescent = 0; % N   
        mdotDescent = 0; % kg/s
    end

    yddot = (Tascent+Tdescent-Drag)/mass - g_accel; % m/s^2
    mdot = -mdotAscent - mdotDescent; % kg/s
    state_dot = [ydot yddot mdot]';
end
