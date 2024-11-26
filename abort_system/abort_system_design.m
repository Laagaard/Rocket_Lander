% Abort System Design

%% Reset
clear; clc; close all;

set(0,'defaultTextInterpreter','latex')

%% Conversion Factors
feet_2_meters = 0.3048; % [unitless]
meters_2_feet = 1/feet_2_meters; % [unitless]
feet_2_inches = 12; % [unitless]
inches_2_feet = 1/feet_2_inches; % [unitless]

inches_2_meters = 0.0254; % [unitless]

lbs_2_newtons = 4.448; % [unitless]
newtons_2_lbs = 1/lbs_2_newtons; % [unitless]

%% Parachute Drop Test (C_D Determination) & Sizing
g = 9.81; % [m/s^2] gravitational acceleration
rho = 1.225; % [kg/m^3] sea-level air density

quick_link_mass = 0.077; % [kg]
test_parachute_mass = 0.062; % [kg]
test_parachute_radius = 15; % [in]
total_weight = (quick_link_mass + test_parachute_mass)*g; % [N]
height_of_deployment = 48*inches_2_meters; % [m] height at which parachute fully deployed
t_initial = 1.894; % [s]
t_final = 2.502; % [s]
v_terminal = height_of_deployment/(t_final - t_initial); % [m/s]
C_D_parachute = (2*total_weight)/(rho*v_terminal^2*pi*(test_parachute_radius*inches_2_meters)^2); % [unitless]

% Parachute Sizing
m_total = 1.5; % [kg] total mass
m_ascent_propellant = 0.0625; % [kg] ascent motor propellant mass

weight = (m_total - m_ascent_propellant)*g; % [N] rocket weight after ascent motor burnout

r_feet = 0.5:0.01:3; % [ft] potential parachute radii
r_meters = r_feet*feet_2_meters; % [m] potential parachute radii
A = pi*r_meters.^2; % [m^2] potential parachute cross-sectional area

v_meters = sqrt(2*weight/(rho*C_D_parachute)) * sqrt(1./A); % [m/s] corresponding parachute descent velocities
v_feet = v_meters*meters_2_feet; % [ft/s] corresponding parachute descent velocities

IREC_rate_limit = 36; % [ft/s] landing descent rate limit per IREC requirements (just using as an "official" reference)
SAF_05_limit = 8*meters_2_feet; % [ft/s] descet rate limit per SAF.05 project requirement

CHOSEN_RADIUS = test_parachute_radius*inches_2_feet; % [ft] chosen parachute radius

opacities = [0.25, 0.5];
patch_shading = repmat(opacities, 1, 2);

figure
hold on
plot(r_feet, v_meters*meters_2_feet, 'k')
xlim([min(r_feet) max(r_feet)])
ylims = ylim;
title("Descent Rate [ft/s] vs. Parachute Radius [ft]", Interpreter="latex")
xlabel("Parachute Radius [ft]", Interpreter="latex")
ylabel("Descent Rate [ft/s]", Interpreter="latex")
pause();

yline(IREC_rate_limit, '--', Label=sprintf("IREC Limit: %.i ft/s", IREC_rate_limit), LabelHorizontalAlignment="right", LabelVerticalAlignment="top", Interpreter="latex")
fill([r_feet(1), r_feet(end), r_feet(end), r_feet(1)], [SAF_05_limit, SAF_05_limit, IREC_rate_limit, IREC_rate_limit], [0.9290 0.6940 0.1250], FaceAlpha=0.25, EdgeAlpha=0)
pause();
yline(SAF_05_limit, '--', Label=sprintf("SAF.05 Limit: %.2f ft/s", SAF_05_limit), LabelHorizontalAlignment="right", Interpreter="latex")
fill([r_feet(1), r_feet(end), r_feet(end), r_feet(1)], [ylims(1), ylims(1), SAF_05_limit, SAF_05_limit], 'green', FaceAlpha=0.25, EdgeAlpha=0)
pause();

[IREC_rate, IREC_rate_arg] = min(abs(v_feet - IREC_rate_limit));
[SAF_05_rate, SAF_05_rate_arg] = min(abs(v_feet - SAF_05_limit));
[~, CHOSEN_RADIUS_arg] = min(abs(r_feet - CHOSEN_RADIUS));

xline(r_feet(IREC_rate_arg), 'r--', Label=sprintf("%.0f in", r_feet(IREC_rate_arg)*feet_2_inches), LabelHorizontalAlignment="center", LabelVerticalAlignment="top", LabelOrientation="horizontal", Interpreter="latex")
xline(r_feet(SAF_05_rate_arg), 'r--', Label=sprintf("%.0f in", r_feet(SAF_05_rate_arg)*feet_2_inches), LabelHorizontalAlignment="center", LabelVerticalAlignment="top", LabelOrientation="horizontal", Interpreter="latex")
plot(r_feet(IREC_rate_arg), v_feet(IREC_rate_arg), 'r.', MarkerSize=20)
plot(r_feet(SAF_05_rate_arg), v_feet(SAF_05_rate_arg), 'r.', MarkerSize=20)
pause();

line([CHOSEN_RADIUS CHOSEN_RADIUS], [ylims(1) v_feet(CHOSEN_RADIUS_arg)], 'Color', 'blue', 'LineStyle', '--')
line([min(r_feet) CHOSEN_RADIUS], [v_feet(CHOSEN_RADIUS_arg) v_feet(CHOSEN_RADIUS_arg)], 'Color', 'blue', 'LineStyle', '--')
plot(CHOSEN_RADIUS, v_feet(CHOSEN_RADIUS_arg), 'b.', MarkerSize=20)
text(CHOSEN_RADIUS, v_feet(CHOSEN_RADIUS_arg), sprintf("%.1f ft, %.2f ft/s", CHOSEN_RADIUS, v_feet(CHOSEN_RADIUS_arg)), HorizontalAlignment="left", VerticalAlignment="bottom", Interpreter="latex")

hold off

%% Parachute Sizing Tabular Output
fprintf("Parachute Radius [ft] | Descent Rate [ft/s]\n")
fprintf("-------------------------------------------\n")
for radius = r_feet
    fprintf("%-21.2f | %-19.2f\n", radius, v_meters(r_feet == radius)*meters_2_feet)
end

%% Run MATLAB Flight Simulation

run("../simulation/6DOF/RocketLander_6DOF_version2.m") % run flight simulation

%% Shock Cord & Quick Link Load Calculations

flight_speed = sqrt(freeFlightStates(:,2).^2 + freeFlightStates(:,4).^2 + freeFlightStates(:,6).^2); % [m/s] flight speed (from RocketLander_2D_version2p0.m)
deployment_force_N = 0.5*rho*flight_speed.^2*A(CHOSEN_RADIUS_arg)*C_D_parachute; % [N] instantaneous force on abort system components if deployed
deployment_force_lb = deployment_force_N*newtons_2_lbs; % [N] instantaneous force on abort system components if deployed

free_flight_time = freeFlightTime; % [s] flight time (excluding powered descent)

indices_of_interest = find(y(:,5) > -0.1);

figure
hold on
plot(free_flight_time(indices_of_interest), flight_speed(indices_of_interest), 'k')
plot(free_flight_time(indices_of_interest), y(indices_of_interest,5), 'b--')
xlabel("Flight Time [s]")
ylabel("Flight Speed $\left[ \frac{m}{s} \right]$, Altitude [m]", Interpreter="latex")
yyaxis right
ax = gca;
ax.YAxis(2).Color = 'r';
plot(free_flight_time(indices_of_interest), deployment_force_lb(indices_of_interest), 'r')
ylabel("Deployment Load [N]", Interpreter="latex")
title("Parachute Deployment Load [N], Altitude [m] vs. Flight Time [s]")

x_axis_limits = xlim;
y_axis_limits = ylim;
fill([x_axis_limits(1), burnTime, burnTime, x_axis_limits(1)], [0, 0, y_axis_limits(2), y_axis_limits(2)], 'red', FaceAlpha=0.25, EdgeAlpha=0)
xline(burnTime, 'k--', Label="APS Burnout", LabelHorizontalAlignment="right", LabelVerticalAlignment="top", Interpreter="latex")

plot(free_flight_time(indices_of_interest(end)), deployment_force_lb(indices_of_interest(end)), 'r.', MarkerSize=20)
text(free_flight_time(indices_of_interest(end)), deployment_force_lb(indices_of_interest(end)), sprintf("%.0f lb", deployment_force_lb(indices_of_interest(end))), HorizontalAlignment="right", VerticalAlignment="top")

legend({"Speed", "Altitude", "Deployment Load", '', '', ''}, location='NW')
hold off

%% Component Sourcing
%{
Nylon Shock Cord: https://www.amazon.com/MONOBIN-Colors-Paracord-Bracelets-Making/dp/B09RWF3NQT/ref=sr_1_2?dib=eyJ2IjoiMSJ9.obURNx7DLeBG9d6WF0Vu890IIjM3V8tOHUNu2wlLI_sF-vlRyp2Yqc5ZKj5Spou3Z0JR2pvMmiEtl8vH73GmnZLMbHeBF_SbbP-76EB3CUlKbBHpCpW41zJay7QQOgtrn-fDH2yU4SOuaoHBS38ZFQEmgOaTIOjrYBjJOjwdQpzRMHh-PLbSdzxt8-sFU2uj.jAZBed-1Y61X77pORiha5LQrEE6aAlf7HkunTL4goic&dib_tag=se&m=A1B5F4J03US3M5&qid=1729018898&s=merchant-items&sr=1-2&th=1
Kevlar Shock Cord: https://www.9km-outdoor.com/products/100-kevlar-line-string-40lb-2000lb-fishing-assist-cord-strong-made-with-kevlar?variant=39439583150243
Quick Link: https://www.harborfreight.com/316-in-quick-links-3-piece-69062.html
%}

%% Ejection Charge Sizing
airframe_ID = 3; % [in] inner airframe diameter
bulkhead_area = pi*(airframe_ID/2)^2; % [in^2] area of bulkhead
abort_system_length = 8; % [in]
internal_pressure = 8.0:0.1:15; % [psi] internal pressure created by charge
charge_force = internal_pressure.*bulkhead_area; % [lb]
imperial_to_metric_conversion_factor = 0.000516; % to go from imperial units to metric charge size (grams)
charge_size = imperial_to_metric_conversion_factor.*charge_force.*abort_system_length; % [g] ejection charge size (i.e., mass of black powder)

shear_pin_2_strength = 25; % [lb] force required to break #2 size shear pin

figure("Name", "Ejection Charge Size vs. Internal Pressure")
hold on
grid minor
plot(internal_pressure, charge_size)
xline(shear_pin_2_strength/bulkhead_area, '--', Label="1 Shear Pin")
xline(2*shear_pin_2_strength/bulkhead_area, '--', Label="2 Shear Pins")
xlabel("Internal Pressure $[psi]$", Interpreter="latex")
ylabel("Ejection Charge Size $[g]$", Interpreter="latex")
title("Ejection Charge Size $[g]$ vs. Internal Pressure $[psi]$", Interpreter="latex")