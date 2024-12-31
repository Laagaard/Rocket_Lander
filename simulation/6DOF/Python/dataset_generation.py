# Libraries
import csv
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
from rocketpy import Flight
from skimage.measure import EllipseModel
import sys

if (len(sys.argv) != 3): # check number of command line arguments (sys.argv[0] is the program name)
    landing_zone_x = 30 # [m] inertial x coordinate of desired landing zone center
    landing_zone_y = 40 # [m] inertial y coordinate of desired landing zone center
else:
    landing_zone_x = int(sys.argv[1]) # [m] inertial x coordinate of desired landing zone center
    landing_zone_y = int(sys.argv[2]) # [m] inertial y coordinate of desired landing zone center
landing_zone_radius = 5 # [m] radius of desired landing zone
desired_lateral_displacement = math.sqrt(landing_zone_x**2 + landing_zone_y**2) # [m] lateral displacement of desired landing zone center

x_lz_interval = np.linspace(-landing_zone_radius + landing_zone_x, landing_zone_radius + landing_zone_x, 250) # [m] x-interval to evaluate for plotting landing zone
landing_zone_upper_edge = [math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] upper edge of landing zone
landing_zone_lower_edge = [-math.sqrt(landing_zone_radius**2 - (x - landing_zone_x)**2) + landing_zone_y for x in x_lz_interval] # [m] lower edge of landing zone

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from setup import DART_rocket, launch_site

    # CSV Output File
    output_file_header = ["Inclination", "Heading", "x_impact", "y_impact"] # header of output CSV file containing trajectory information
    output_file = open("trajectory_dataset.csv", 'w', newline="") # output CSV file containing optimal trajectory information
    writer = csv.writer(output_file) # CSV writer for output file containing optimal trajectory information
    writer.writerow(output_file_header) # write header row of output CSV file containing optimal trajectory information

    # Lateral Displacement Plot
    lateral_displacement_fig = plt.figure()
    lateral_displacement_ax = lateral_displacement_fig.add_subplot()
    lateral_displacement_ax.plot(0, 0, 'k.', label="Launch Site") # plot launch site (i.e., inertial CS origin)
    lateral_displacement_ax.plot(landing_zone_x, landing_zone_y, 'r+') # plot center of the landing zone
    lateral_displacement_ax.plot(x_lz_interval, landing_zone_upper_edge, 'r-', label="Landing Zone") # plot upper semi-circle of the landing zone
    lateral_displacement_ax.plot(x_lz_interval, landing_zone_lower_edge, 'r-') # plot lower semi-circle of the landing zone
    lateral_displacement_ax.set_xlabel("X - East [m]")
    lateral_displacement_ax.set_ylabel("Y - North [m]")
    lateral_displacement_ax.grid(which="major", axis="both")

    x_impact_coords = np.array([]) # [m] list to track x-impact coordinates
    y_impact_coords = np.array([]) # [m] list to track y-impact coordinates

    success_bool = False # boolean to track primary algorithm success (w.r.t largest best-fit ellipse encompassing landing zone coordinates)
    launch_inclination = 89 # [deg]
    launch_heading = 0 # [deg]
    inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a random plotting color

    try:
        while (not success_bool):
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

            print(f"Inclination: {round(test_flight.inclination, 2)} deg, Heading: {round(test_flight.heading, 2)} deg")

            if (launch_heading < 360): # launch heading can be increased more
                launch_heading += 5 # [deg] increase launch heading
                x_impact_coords = np.append(x_impact_coords, np.array([test_flight.x_impact])) # [m] add newest x-impact coordinate to list of tracked coordinates
                y_impact_coords = np.append(y_impact_coords, np.array([test_flight.y_impact])) # [m] add newest y-impact coordinate to list of tracked coordinates
            else: # launch heading has reached maximum (practical) value (i.e., 360 deg)
                launch_heading = 0 # [deg] reset launch heading
                launch_inclination -= 1 # [deg] decrease launch inclination

                x_impact_coords = np.append(x_impact_coords, np.array([test_flight.x_impact])) # [m] add newest x-impact coordinate to list of tracked coordinates
                y_impact_coords = np.append(y_impact_coords, np.array([test_flight.y_impact])) # [m] add newest y-impact coordinate to list of tracked coordinates
                impact_coords = np.concatenate((x_impact_coords.reshape(len(x_impact_coords), 1), y_impact_coords.reshape(len(y_impact_coords), 1)), axis=1)

                ellipse = EllipseModel() # create best-fit ellipse model
                if (ellipse.estimate(impact_coords)): # fit the best-fit model
                    ellipse_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor=inclination_color, facecolor="None")
                    lateral_displacement_ax.add_patch(ellipse_patch) # for some reason, this has to be here for the following if statement to work properly
                    if (ellipse_patch.contains_point(point=lateral_displacement_ax.transData.transform(values=(landing_zone_x, landing_zone_y)))):
                        success_bool = True # largest best-fit ellipse encompasses landing zone coordinates
                x_impact_coords = np.array([]) # reset list of x-impact coordinates
                y_impact_coords = np.array([]) # reset list of ys-impact coordinates
                inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a new plotting color
    except (IndexError, ValueError): # Error occurs when the inclination goes too low for RocketPy to handle
        print("Inclination Too Low - Terminating Program")

    # Close output file
    output_file.close()

    if (success_bool):
        print("Houston, we have an INTERPOLATION problem")
    else:
        print("Houston, we have an EXTRAPOLATION problem")

    lateral_displacement_ax.axis('equal') # set axis limits equivalent
    lateral_displacement_ax.set_title(f"Trajectory & Landing Zone \n(Inclination: {round(test_flight.inclination, 2)} deg)") # add graph title
    plt.show() # show the graph