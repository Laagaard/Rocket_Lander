# Libraries
import csv
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
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

# CSV Output File
output_file_header = ["Inclination", "Heading", "x_impact", "y_impact"] # header of output CSV file containing trajectory information
output_file = open("trajectory_dataset.csv", 'w', newline="") # output CSV file containing optimal trajectory information
writer = csv.writer(output_file) # CSV writer for output file containing optimal trajectory information
writer.writerow(output_file_header) # write header row of output CSV file containing optimal trajectory information

impact_location_accuracy = 1 # [m] simulation accuracy threshold w.r.t. desired landing location

plt.ion()
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
lateral_displacement_ax.axis('equal') # set axis limits equivalent
lateral_displacement_ax.grid(which="major", axis="both")
lateral_displacement_ax.set_title(f"Trajectory & Landing Zone")

# Accuracy Square
x_accuracy_square_interval = np.linspace(-impact_location_accuracy + landing_zone_x, impact_location_accuracy + landing_zone_x, 250) # [m] x-interval to evaluate for plotting accuracy square
y_accuracy_square_interval = np.linspace(-impact_location_accuracy + landing_zone_y, impact_location_accuracy + landing_zone_y, 250) # [m] y-interval to evaluate for plotting accuracy square
accuracy_square_top_edge = np.full(len(x_accuracy_square_interval), landing_zone_y + impact_location_accuracy) # [m] top edge of accuracy square
accuracy_square_bottom_edge = np.full(len(x_accuracy_square_interval), landing_zone_y - impact_location_accuracy) # [m] bottom edge of accuracy square
accuracy_square_left_edge = np.full(len(y_accuracy_square_interval), landing_zone_x - impact_location_accuracy) # [m] left edge of accuracy square
accuracy_square_right_edge = np.full(len(y_accuracy_square_interval), landing_zone_x + impact_location_accuracy) # [m] right edge of accuracy square

landing_zone_coords = np.array([landing_zone_x, landing_zone_y]) # [m] inertial coordinates of desired landing zone center

print("\nSurface Wind Conditions:")
print(f"Heading: {round(launch_site.wind_heading(launch_site.elevation), 2)} [deg], Direction: {round(launch_site.wind_direction(launch_site.elevation), 2)} [deg]")
print(f"x_vel: {launch_site.wind_velocity_x(launch_site.elevation)} [m/s], y_vel: {launch_site.wind_velocity_y(launch_site.elevation)} [m/s] \n")

for launch_inclination in np.arange(90, 70, -1): # iterate through launch inclinations [start, stop)
    inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1))
    for launch_heading in np.arange(0, 360, 5): # iterate through launch headings [start, stop)
        test_flight = Flight(
            rocket=DART_rocket,
            environment=launch_site,
            rail_length=1.5, # [m] length of the launch rail (NEED TO DOUBLE CHECK UNITS)
            inclination=launch_inclination, # [deg] rail inclination relative to the ground
            heading=launch_heading, # [deg] heading angle relative to North (East = 90)
            time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
        ) # run trajectory simulation

        trajectory_information = [test_flight.inclination, test_flight.heading, test_flight.x_impact, test_flight.y_impact]
        writer.writerow(trajectory_information) # write trajectory information to output CSV file

        print(f"Inclination: {round(launch_inclination, 2)}, Heading: {round(launch_heading, 2)}, Apogee: {round(test_flight.apogee, 2)} [m]")

        if (launch_inclination == 90): # heading sweep is pointless if launching straight up
            lateral_displacement_ax.plot(test_flight.x_impact, test_flight.y_impact, '.', color=inclination_color)
            break
        elif (launch_heading < 350):
            lateral_displacement_ax.plot(test_flight.x_impact, test_flight.y_impact, '.', color=inclination_color)
            continue
        else:
            lateral_displacement_ax.plot(test_flight.x_impact, test_flight.y_impact, '.', color=inclination_color)
            lateral_displacement_ax.legend(loc="best")
            plt.pause(0.5)

# Close output file
output_file.close()