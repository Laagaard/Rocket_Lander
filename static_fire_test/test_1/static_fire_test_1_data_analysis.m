%% Import Data & Distinguish Measured Variables
clc; clear; close all;
test_data = readmatrix("static_fire_test_1_data.txt"); % import test data

continuity = test_data(:,1); % [bool] igniter continuity
triggered = test_data(:,2); % [bool] ignition command
load = test_data(:,3); % [g] measured load
time = test_data(:,4); % [ms] DAQ system time since boot-up
SDcard = test_data(:,5); % [bool] writing to SD card

theoretical_data = readmatrix("../../AeroTechG25W_thrustcurve.csv");
theoretical_data = [[0 0]; theoretical_data];

%% Data Processing
time = time/1E3; % [s] convert system time to seconds
load = (load/1E3)*9.81; % [N] convert load to Newtons

theoretical_propellant_mass = 62.5/1E3; % [kg] theoretical mass of the propellant
theoretical_propellant_weight = theoretical_propellant_mass*9.81; % [N] theoretical weight of the propellant

index_of_interest = find(triggered == 1, 1); % find first index of ignition command
time_indices_of_interest = index_of_interest:length(triggered); % time indices following ignition

time_processed = time(time_indices_of_interest); % [s] time following ignition
load_processed = load(time_indices_of_interest); % [N] load cell measurements following ignition

initial_offset = load_processed(1); % [N] pre-test load cell deviation
load_processed = load_processed - (abs(initial_offset) * sign(initial_offset)); % [N] apply load offset

load_cutoff = -theoretical_propellant_weight; % [N] thrust = 0 when load cell measurement equals negative theoretical propellant weight
indices_of_thrust_present = find(load_processed > load_cutoff); % indices with load measurements greater than expended (i.e., negative) propellant mass
time_processed_thrust_present = time_processed(indices_of_thrust_present); % [s] time with thrust present
load_processed_thrust_present = load_processed(indices_of_thrust_present); % [N] load cell measurements with thrust present

pre_thrust_load_cutoff = 0.1; % [N] load cutoff to eliminate early (insignificant) thrust measurements (from igniter gas)
first_index_of_notable_thrust = find(load_processed_thrust_present > pre_thrust_load_cutoff, 1); % first index of notable (i.e., significant) thrust
indices_of_notable_thrust = find(indices_of_thrust_present >= first_index_of_notable_thrust); % all indices of notable thrust
time_processed_notable_thrust = time_processed_thrust_present(indices_of_notable_thrust); % [s] time values with meaningful thrust present (i.e., remove time values during ignition delay)
time_processed_notable_thrust = time_processed_notable_thrust - time_processed_notable_thrust(1); % [s] rescale time values
load_processed_notable_thrust = load_processed_thrust_present(indices_of_notable_thrust); % [N] load measurements with notable thrust present
[~, index_max_thrust] = max(load_processed_notable_thrust); % index of max thrust

% Computation of motor mass weight flow (assuming m_dot is proportional to thrust)
possible_experimental_motor_weights = [];
possible_experimental_motor_weight_flows = [];
scaling_factors = max(load_processed_notable_thrust):0.1:10*max(load_processed_notable_thrust);
for scaling_factor_index = 1:length(scaling_factors)
    iterative_motor_weight_flow = load_processed_notable_thrust./scaling_factors(scaling_factor_index); % [N/s]
    possible_experimental_motor_weight_flows = [possible_experimental_motor_weight_flows, iterative_motor_weight_flow]; % [N/s]
    iterative_experimental_motor_weight = trapz(time_processed_notable_thrust, iterative_motor_weight_flow); % [N]
    possible_experimental_motor_weights = [possible_experimental_motor_weights, iterative_experimental_motor_weight]; % [N]
end
[~, index_best_experimental_motor_weight] = min(abs(possible_experimental_motor_weights - theoretical_propellant_weight));
best_experimental_motor_weight_flow = possible_experimental_motor_weight_flows(:, index_best_experimental_motor_weight); % [N/s] weight flow of motor

motor_weight_offset = cumtrapz(time_processed_notable_thrust, best_experimental_motor_weight_flow); % [N] weight offset of motor
load_processed_accounting_for_weight = load_processed_notable_thrust + abs(motor_weight_offset); % [N] apply changing motor weight

%% Thrust Curve
thrust_curve_animation = VideoWriter("thrust_curve_animation", "MPEG-4");
open(thrust_curve_animation);

figure("Name", "Thrust Curve")
thrust_curves_plot_handle = gcf;
hold on
grid minor
title("Thrust [N] vs. Time [s]", interpreter="latex")
xlabel("Time [s]", interpreter="latex")
ylabel("Thrust [N]", Interpreter="latex")
plot(theoretical_data(:,1), theoretical_data(:,2), 'rs-', 'DisplayName', "Expected Thrust");
plot(time_processed_notable_thrust, load_processed_accounting_for_weight, 'bs-', 'DisplayName', "Measured Thrust")
legend()

animation_current_X = [0, 0];
animation_current_Y = [0, 0];
thrust_curves_plot = plot(animation_current_X, animation_current_Y, 'k.-', 'HandleVisibility', 'off');

thrust_curves_plot.XDataSource = "animation_current_X";
thrust_curves_plot.YDataSource = "animation_current_Y";
for idx = 1:max(size(time_processed_notable_thrust))
    animation_current_X = time_processed_notable_thrust(idx).*ones(1,2);
    animation_current_Y(end) = load_processed_accounting_for_weight(idx);
    refreshdata(thrust_curves_plot)
    pause(0.25)

    % Update orientation plot animation
    thrust_curve_frame = getframe(thrust_curves_plot_handle);
    writeVideo(thrust_curve_animation, thrust_curve_frame);
end
hold off

close(thrust_curve_animation)

%% Performance Comparison
predicted_impulse = 117.5; % [N*s] (https://www.thrustcurve.org/simfiles/5f4294d20002e9000000045f/)
predicted_max_thrust = 41.2; % [N] (https://www.thrustcurve.org/simfiles/5f4294d20002e9000000045f/)
predicted_average_thrust = 24.1; % [N] (https://www.thrustcurve.org/simfiles/5f4294d20002e9000000045f/)
predicted_burn_time = 4.9; % [s] (https://www.thrustcurve.org/simfiles/5f4294d20002e9000000045f/)

% Burn Time Calculation
burn_time_thrust_threshold = 0.05*max(load_processed_notable_thrust); % [N] 5% thrust cutoff per NFPA 1125 standard definition of burn time
indices_for_burn_time = find(load_processed_notable_thrust > burn_time_thrust_threshold);
time_vector_for_burn_time = time_processed_notable_thrust(indices_for_burn_time); % [s]
experimental_burn_time = time_vector_for_burn_time(end) - time_vector_for_burn_time(1); % [s] experimental burn time

experimental_total_impulse = trapz(time_processed_notable_thrust, load_processed_accounting_for_weight); % [N*s] experimental impulse is the time integral of the thrust curve
experimental_max_thrust = max(load_processed_accounting_for_weight); % [N]

load_processed_accounting_for_weight_for_average_thrust = load_processed_accounting_for_weight(indices_for_burn_time); % [N]
experimental_impulse_for_average_thrust = trapz(time_vector_for_burn_time, load_processed_accounting_for_weight_for_average_thrust); % [N*s]
experimental_average_thrust = experimental_impulse_for_average_thrust/experimental_burn_time; % [N] average thrust defined as impulse divided by burn time (https://www.thrustcurve.org/info/glossary.html)

results_table = table('Size', [2, 4], 'VariableTypes', {'double', 'double', 'double', 'double'}, ...
                      'RowNames', {'Expected', 'Measured'}, 'VariableNames', {'Total Impulse [N*s]', 'Max Thrust [N]', 'Average Thrust [N]', 'Burn Time [s]'});
results_table{1,:} = [predicted_impulse, predicted_max_thrust, predicted_average_thrust, predicted_burn_time]; % add predicted values to results table
results_table{2,:} = [experimental_total_impulse, experimental_max_thrust, experimental_average_thrust, experimental_burn_time]; % add experimental values to results table
disp(results_table)

%% Thrust Curve w/ Motor Characteristics
figure("Name", "Thrust Curve")
hold on
area(time_processed_notable_thrust, load_processed_accounting_for_weight, FaceColor=[232/255 232/255 232/255], EdgeColor="none")
text(time_processed_notable_thrust(index_max_thrust), experimental_average_thrust/2, sprintf("Impulse: \n%.2f [N*s]", experimental_total_impulse), HorizontalAlignment="center")
plot(time_processed_notable_thrust, load_processed_accounting_for_weight, 'bs-')
xline(time_vector_for_burn_time(1), '--', Label="Start Burn Time")
xline(time_vector_for_burn_time(end), '--', Label="End Burn Time")
yline(experimental_average_thrust, '--', Label=sprintf("Average Thrust: %.2f N", experimental_average_thrust))
grid minor
title("Thrust [N] vs. Time [s]", interpreter="latex")
xlabel("Time [s]", interpreter="latex")
ylabel("Thrust [N]", Interpreter="latex")
hold off

%% Ignition Delay
ignition_delay = (time_processed_thrust_present(first_index_of_notable_thrust) - time_processed_thrust_present(1))*1E3; % [ms]
fprintf("Motor Ignition Delay: %.2f [ms]\n", ignition_delay)