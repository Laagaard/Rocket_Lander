# Libraries
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
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

success_bool = False # boolean to track simulation success (i.e., finding the required inclination and heading)
convergence_bool = True # boolean to track simulation convergence (i.e., whether possible to find the required inclination and heading with provided step sizes)
launch_inclination = 87.1 # [deg] launch angle from vertical (90 = purely vertical)
launch_heading = 0 # [deg] launch heading (0 = North)
impact_location_accuracy = 1 # [m] simulation accuracy threshold w.r.t. desired landing location
lateral_displacements = [] # [m] list of simulated displacements for a given inclination

while (not success_bool):
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=1.5, # [m] length of the launch rail (NEED TO DOUBLE CHECK UNITS)
        inclination=launch_inclination, # [deg] rail inclination relative to the ground
        heading=launch_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    ) # run trajectory simulation

    print(f"Inclination: {round(test_flight.inclination, 2)}, Heading: {round(test_flight.heading, 2)}, ", end="")
    print(f"Desired (x, y): ({round(landing_zone_x, 2)}, {round(landing_zone_y, 2)}), Impact (x, y): ({round(test_flight.x_impact, 2)}, {round(test_flight.y_impact, 2)}), ", end="")

    # Check for success (i.e., impact location close enough to the desired landing zone center)
    if (abs(test_flight.x_impact - landing_zone_x) < impact_location_accuracy and abs(test_flight.y_impact - landing_zone_y) < impact_location_accuracy):
        print("Success?")
        success_bool = True
        break

    # Adjust launch inclination and heading as necessary
    if (launch_inclination == 90): # Heading is irrelevant if inclination = 90 deg
        launch_inclination -= 0.05 # [deg] decrease launch inclination
    elif (launch_heading < 360):
        launch_heading += 0.5 # [deg] increase launch heading
    else:
        print("Checking lateral displacements...")
        if (min(lateral_displacements) > (desired_lateral_displacement)):
            print("Require finer heading and inclination step sizes to achieve convergence")
            convergence_bool = False
            break
        lateral_displacements = [] # reset list of lateral displacements
        launch_heading = 0 # [deg] reset launch heading to 0 deg (North)
        launch_inclination -= 0.05 # [deg] decrease launch inclination
    new_lateral_displacement = math.sqrt(test_flight.x_impact**2 + test_flight.y_impact**2) # [m] lateral displacement of simulated trajectory
    lateral_displacements.append(new_lateral_displacement) # append latest simulated lateral displacement to list of lateral displacements
    print(f"Lateral Displacement: {round(new_lateral_displacement, 2)} [m]")

# Save optimal trajectory information if it was found
if (success_bool):
    optimal_trajectory_information = [test_flight.inclination, test_flight.heading, landing_zone_x, landing_zone_y, round(test_flight.x_impact, 2), round(test_flight.y_impact, 2)]
    writer.writerow(optimal_trajectory_information)

# Close output file
output_file.close()

solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

# Lateral Displacement Plot
x_lz_interval = np.linspace(-landing_zone_radius + landing_zone_x, landing_zone_radius + landing_zone_x, 250) # [m] x-interval to evaluate for plotting landing zone
landing_zone_upper_edge = [math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] upper edge of landing zone
landing_zone_lower_edge = [-math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] lower edge of landing zone
lateral_displacement_fig = plt.figure()
lateral_displacement_ax = lateral_displacement_fig.add_subplot()
lateral_displacement_ax.plot(landing_zone_x, landing_zone_y, 'r+') # plot center of the landing zone
lateral_displacement_ax.plot(x_lz_interval, landing_zone_upper_edge, 'r-') # plot upper semi-circle of the landing zone
lateral_displacement_ax.plot(x_lz_interval, landing_zone_lower_edge, 'r-') # plot lower semi-circle of the landing zone
lateral_displacement_ax.plot(test_flight.x, test_flight.y, 'b') # plot trajectory
lateral_displacement_ax.set_xlabel("X - East [m]")
lateral_displacement_ax.set_ylabel("Y - North [m]")
lateral_displacement_ax.set_title("Lateral Displacement")
lateral_displacement_ax.axis('equal') # set axis limits equivalent
# plt.close()

# Trajectory Plot
trajectory_fig = plt.figure()
trajectory_ax = trajectory_fig.add_subplot(projection='3d')
trajectory_ax.scatter(test_flight.x, test_flight.y, test_flight.z, 'b') # plot trajectory
trajectory_ax.plot(landing_zone_x, landing_zone_y, zs=0, color='r', marker="+") # plot center of the landing zone
trajectory_ax.plot(x_lz_interval, landing_zone_upper_edge, zs=0, color='r') # plot upper semi-circle of the landing zone
trajectory_ax.plot(x_lz_interval, landing_zone_lower_edge, zs=0, color='r') # plot lower semi-circle of the landing zone
trajectory_ax.set_xlabel("X - East [m]")
trajectory_ax.set_ylabel("Y - North [m]")
trajectory_ax.set_zlabel("Z - Altitude AGL [m]")
trajectory_ax.set_zlim(bottom=0) # set lower Z (altitude) plot limit
trajectory_ax.set_title("Trajectory")
plt.show()