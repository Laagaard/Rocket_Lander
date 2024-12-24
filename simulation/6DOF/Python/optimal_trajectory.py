# Libraries
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import statistics
from rocketpy import Flight
from setup import DART_rocket, launch_site

if (len(sys.argv) != 3): # check number of command line arguments (sys.argv[0] is the program name)
    landing_zone_x = 30 # [m] inertial x coordinate of desired landing zone center
    landing_zone_y = 40 # [m] inertial y coordinate of desired landing zone center
else:
    landing_zone_x = int(sys.argv[1]) # [m] inertial x coordinate of desired landing zone center
    landing_zone_y = int(sys.argv[2]) # [m] inertial y coordinate of desired landing zone center
landing_zone_radius = 5 # [m] radius of desired landing zone
desired_lateral_displacement = math.sqrt(landing_zone_x**2 + landing_zone_y**2) # [m] lateral displacement of desired landing zone center
print(f"Desired Lateral Displacement: {round(desired_lateral_displacement, 2)} [m]")

output_file_header = ["Inclination", "Heading", "Desired_x", "Desired_y", "x_impact", "y_impact"] # header of output CSV file containing optimal trajectory information
output_file = open("optimal_trajectory_information.csv", 'w', newline="") # output CSV file containing optimal trajectory information
writer = csv.writer(output_file) # CSV writer for output file containing optimal trajectory information
writer.writerow(output_file_header) # write header row of output CSV file containing optimal trajectory information

impact_location_accuracy = 1 # [m] simulation accuracy threshold w.r.t. desired landing location

# Lateral Displacement Plot
x_lz_interval = np.linspace(-landing_zone_radius + landing_zone_x, landing_zone_radius + landing_zone_x, 250) # [m] x-interval to evaluate for plotting landing zone
landing_zone_upper_edge = [math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] upper edge of landing zone
landing_zone_lower_edge = [-math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] lower edge of landing zone
lateral_displacement_fig = plt.figure()
lateral_displacement_ax = lateral_displacement_fig.add_subplot()
lateral_displacement_ax.plot(landing_zone_x, landing_zone_y, 'r+') # plot center of the landing zone
lateral_displacement_ax.plot(x_lz_interval, landing_zone_upper_edge, 'r-', label="Landing Zone") # plot upper semi-circle of the landing zone
lateral_displacement_ax.plot(x_lz_interval, landing_zone_lower_edge, 'r-') # plot lower semi-circle of the landing zone
lateral_displacement_ax.set_xlabel("X - East [m]")
lateral_displacement_ax.set_ylabel("Y - North [m]")
lateral_displacement_ax.set_title("Trajectory & Landing Zone")
lateral_displacement_ax.axis('equal') # set axis limits equivalent
lateral_displacement_ax.grid(which="major", axis="both")

landing_zone_coords = np.array([landing_zone_x, landing_zone_y]) # [m] inertial coordinates of desired landing zone center
wind_altitude_interval = np.linspace(launch_site.elevation, launch_site.elevation + 250, 2000) # [m] altitude interval to evaluate wind profiles
average_wind_x = statistics.mean(launch_site.wind_velocity_x(wind_altitude_interval)) # [m/s] average wind velocity x component
average_wind_y = statistics.mean(launch_site.wind_velocity_y(wind_altitude_interval)) # [m/s] average wind velocity y component
average_wind_vector = np.array([average_wind_x, average_wind_y]) # [m/s] vector of average wind velocity components

success_bool = False # boolean to track simulation success (i.e., finding the required inclination and heading)
convergence_bool = True # boolean to track simulation convergence (i.e., whether possible to find the required inclination and heading with provided step sizes)
launch_inclination = 87.1 # [deg] launch angle from vertical (90 = purely vertical)
target_heading = 0 # [deg] launch heading (0 = North)
impact_location_accuracy = 1 # [m] simulation accuracy threshold w.r.t. desired landing location
lateral_displacements = [] # [m] list of simulated displacements for a given inclination
launch_site.process_standard_atmosphere() # disable wind profiles

# Primary Algorithm Loop
while (not success_bool):
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=1.5, # [m] length of the launch rail (NEED TO DOUBLE CHECK UNITS)
        inclination=launch_inclination, # [deg] rail inclination relative to the ground
        heading=target_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    ) # run trajectory simulation

    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
    flight_time = solution_time[-1] # [s] total time of flight
    displacement_achieved = np.array([test_flight.x_impact, test_flight.y_impact]) # [m] lateral displacement of trajectory
    wind_displacement = average_wind_vector*flight_time # [m] theoretical displacement caused by wind
    target_lateral_displacement = landing_zone_coords - wind_displacement # [m] target lateral displacement coordinates w/o wind
    target_lateral_displacement_x = target_lateral_displacement[0] # [m] x-component of target lateral displacement
    target_lateral_displacement_y = target_lateral_displacement[1] # [m] y-component of target lateral displacement
    target_heading = (90 - abs(math.degrees(math.atan2(target_lateral_displacement_y, target_lateral_displacement_x)))) # [deg] target heading

    # Accuracy Square
    x_accuracy_square_interval = np.linspace(-impact_location_accuracy + target_lateral_displacement_x, impact_location_accuracy + target_lateral_displacement_x, 250) # [m] x-interval to evaluate for plotting accuracy square
    y_accuracy_square_interval = np.linspace(-impact_location_accuracy + target_lateral_displacement_y, impact_location_accuracy + target_lateral_displacement_y, 250) # [m] y-interval to evaluate for plotting accuracy square
    accuracy_square_top_edge = np.full(len(x_accuracy_square_interval), target_lateral_displacement_y + impact_location_accuracy) # [m] top edge of accuracy square
    accuracy_square_bottom_edge = np.full(len(x_accuracy_square_interval), target_lateral_displacement_y - impact_location_accuracy) # [m] bottom edge of accuracy square
    accuracy_square_left_edge = np.full(len(y_accuracy_square_interval), target_lateral_displacement_x - impact_location_accuracy) # [m] left edge of accuracy square
    accuracy_square_right_edge = np.full(len(y_accuracy_square_interval), target_lateral_displacement_x + impact_location_accuracy) # [m] right edge of accuracy square

    print(f"Flight Time: {round(flight_time, 2)} [s], Inclination: {round(test_flight.inclination, 2)}, Heading: {round(test_flight.heading, 2)}, ", end="")
    print(f"Desired (x, y): ({round(target_lateral_displacement_x, 2)}, {round(target_lateral_displacement_y, 2)}), Impact (x, y): ({round(test_flight.x_impact, 2)}, {round(test_flight.y_impact, 2)}), ", end="")

    # Check for success (i.e., impact location within accuracy square of the desired impact location)
    if (abs(test_flight.x_impact - target_lateral_displacement_x) < impact_location_accuracy and abs(test_flight.y_impact - target_lateral_displacement_y) < impact_location_accuracy):
        print("Success")
        lateral_displacement_ax.plot(target_lateral_displacement_x, target_lateral_displacement_y, 'rx', label=f"Target Impact Location") # plot desired impact location
        success_bool = True
        break

    # Adjust launch inclination
    print("Checking lateral displacements...")
    # Average tracked lateral displacement is greater than the target lateral displacement
    if (np.linalg.norm(displacement_achieved, 2) > np.linalg.norm(target_lateral_displacement, ord=2)):
        print(f"Greater Average Tracked: {round(np.linalg.norm(displacement_achieved, 2), 2)} [m], Desired: {round(np.linalg.norm(target_lateral_displacement, ord=2), 2)} [m]")
        launch_inclination += (np.linalg.norm(displacement_achieved, 2) - np.linalg.norm(target_lateral_displacement, ord=2))/10 # [deg] increase launch inclination (to decrease lateral displacement)
    # Average tracked lateral displacement is less than the target lateral displacement
    elif (np.linalg.norm(displacement_achieved, 2) < np.linalg.norm(target_lateral_displacement, ord=2)):
        print(f"Less Average Tracked: {round(np.linalg.norm(displacement_achieved, 2), 2)} [m], Desired: {round(np.linalg.norm(target_lateral_displacement, ord=2), 2)} [m]")
        launch_inclination -= (np.linalg.norm(target_lateral_displacement, ord=2) - np.linalg.norm(displacement_achieved, 2))/10 # [deg] decrease launch inclination (to increase lateral displacement)
    lateral_displacements = [] # reset list of lateral displacements
    new_lateral_displacement = math.sqrt(test_flight.x_impact**2 + test_flight.y_impact**2) # [m] lateral displacement of simulated trajectory
    lateral_displacements.append(new_lateral_displacement) # append latest simulated lateral displacement to list of lateral displacements
    print(f"Lateral Displacement: {round(new_lateral_displacement, 2)} [m]")

# Save optimal trajectory information if it was found
if (success_bool):
    optimal_trajectory_information = [round(test_flight.inclination, 2), round(test_flight.heading, 2), landing_zone_x, landing_zone_y, round(test_flight.x_impact, 2), round(test_flight.y_impact, 2)]
    writer.writerow(optimal_trajectory_information)

# Close output file
output_file.close()

# Add optimal trajectory to lateral displacement plot
lateral_displacement_ax.plot(test_flight.x(solution_time), test_flight.y(solution_time), 'b', label="Trajectory") # plot trajectory

# Plot Accuracy Square
lateral_displacement_ax.plot(x_accuracy_square_interval, accuracy_square_top_edge, 'k--', label="Accuracy Square") # plot top edge of accuracy square
lateral_displacement_ax.plot(x_accuracy_square_interval, accuracy_square_bottom_edge, 'k--') # plot bottom edge of accuracy square
lateral_displacement_ax.plot(accuracy_square_left_edge, y_accuracy_square_interval, 'k--') # plot left edge of accuracy square
lateral_displacement_ax.plot(accuracy_square_right_edge, y_accuracy_square_interval, 'k--') # plot right edge of accuracy square

lateral_displacement_ax.legend(loc="best")
plt.show()

# Trajectory Plot
# trajectory_fig = plt.figure()
# trajectory_ax = trajectory_fig.add_subplot(projection='3d')
# trajectory_ax.scatter(test_flight.x(solution_time), test_flight.y(solution_time), test_flight.z(solution_time), 'b') # plot trajectory
# trajectory_ax.plot(landing_zone_x, landing_zone_y, zs=0, color='r', marker="+") # plot center of the landing zone
# trajectory_ax.plot(x_lz_interval, landing_zone_upper_edge, zs=0, color='r') # plot upper semi-circle of the landing zone
# trajectory_ax.plot(x_lz_interval, landing_zone_lower_edge, zs=0, color='r') # plot lower semi-circle of the landing zone
# trajectory_ax.set_xlabel("X - East [m]")
# trajectory_ax.set_ylabel("Y - North [m]")
# trajectory_ax.set_zlabel("Z - Altitude AGL [m]")
# trajectory_ax.set_zlim(bottom=0) # set lower Z (altitude) plot limit
# trajectory_ax.set_title("Trajectory")