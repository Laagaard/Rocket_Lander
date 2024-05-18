%{
Nathan Tardy
ntardy2022@my.fit.edu
Florida Institute of Technology
%}

%{
Thoughts:
1) Easier: just decide thrust and assume infinite propellent
    More difficult: figure out fuel for thrust and amount of thrust
    possible
2) Possible deliverables:
    a) Plots of possible max distances from landing site for successful
    landing
%}

%{
Preliminary Assumptions:
1) The acceleration due to gravity is a constant 9.81 m/s^2
2) The rocket engine provides a constant thrust when on and 0 when off
%}

clc; clear; close all;

g = 9.81; % [m/s^2] acceleration due to gravity
mass = 21.5; % [kg] (chosen arbitrarily)
weight = mass*g; % [N] weight of the rocket
thrust_2_weight = 2; % thrust to weight ratio (chosen arbitrarily)
thrust = thrust_2_weight * weight; % [N] thrust provided by the engine

% Define an n-t corredinate system with t being the direction tangential to flight
% Define an angle theta between the t axis and the thrust vector
theta = 0; % [deg]
% Define an angle phi between the t axis and the vertical direction
phi = 0; % [deg]
net_force_t = thrust*cosd(theta) - weight*cosd(phi);
net_force_n = thrust*sind(theta) - weight*sind(phi);
net_force = [net_force_n net_force_t];

velocity_n = 0; % [m/s]
velocity_t = 0; % [m/s]
altitude = 0; % [m]
time = 0; % [s]

delta_t = 0.001; % [s] numerical integration time step

t_MECO = 5; % [s] time of main engine cutoff

% Ascent before burnout
for t = 0:delta_t:t_MECO
    time = [time, time(end) + delta_t];
    delta_v_t = net_force(2) * delta_t;
    velocity_t = [velocity_t, velocity_t(end) + delta_v_t];
    delta_altitude = velocity_t(end) * delta_t;
    altitude = [altitude, altitude(end) + delta_altitude];
end

% Ascent after burnout and descent
net_force(2) = net_force(2) - thrust;

while (altitude(end) ~= 0)
    % Checks momentum of the rocket against impulse engines can deliver
    if (abs(mass*velocity_t(end)) >= abs((thrust-weight) * (altitude(end)/velocity_t(end))) && velocity_t(end) < 0)
        % Checks momentum of the rocket against momentum at MECO
        if (abs(mass*velocity_t(end)) >= abs(mass*velocity_t(find(abs(time-t_MECO)<=10^-4))))
            net_force(2) = thrust - weight;
            t_reignition = time(end);
        end
    end
    time = [time, time(end) + delta_t];
    delta_v_t = net_force(2) * delta_t;
    velocity_t = [velocity_t, velocity_t(end) + delta_v_t];
    delta_altitude = velocity_t(end) * delta_t;
    altitude = [altitude, altitude(end) + delta_altitude];
    if (altitude(end) < 1)
        break
    end
end

% int F dt = delta mV
% altitude = altitude./1000; % converting to [km]

figure
hold on
plot(time, altitude)
plot(time, velocity_t)
title('Altitude, Vertical Velocity vs. Time', Interpreter='latex')
xlabel('Time [s]', Interpreter='latex')
ylabel('Altitude [m], Vertical Velocity [m/s]', Interpreter='latex')
yline(0, '--')
text(time(end)/2, 0, {'Ground', 'Level'})
xline(t_MECO, '-.')
text(t_MECO, max(altitude)/2, 'MECO')
xline(t_reignition, '-.')
text(t_reignition, max(altitude)/2, 'Reignition')
legend({'Altitude','Velocity'}, Location="northwest")
hold off
