# Libraries
import csv
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
import os
from rocketpy import Flight
import shutil
from skimage.measure import EllipseModel
import stat
import sys
from dataset_generation import landing_zone_x, landing_zone_y, x_lz_interval, landing_zone_upper_edge, landing_zone_lower_edge
from optimal_trajectory import optimal_inclination, optimal_heading
from setup import launch_site, DART_rocket

# Function for `shutil.rmtree` to call on "Access is denied" error from read-only folder
def remove_readonly(func, path, excinfo):
    os.chmod(path, stat.S_IWRITE)
    func(path)

CSV_output_dir = "monte_carlo_trajectories"
if os.path.exists(CSV_output_dir):
    shutil.rmtree(CSV_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
os.mkdir(CSV_output_dir) # Create folder for CSV files of Monte Carlo trajectory simulation data

trajectory_csv_header = ["Time", "x_inertial", "y_inertial", "landing_zone_flag"] # CSV header for all Monte Carlo trajectory simulation files

landing_zone_upper_edge = np.asarray(a=landing_zone_upper_edge) # convert to numpy array
landing_zone_lower_edge = np.asarray(a=landing_zone_lower_edge) # convert to numpy array

full_x_lz_interval = np.concatenate((x_lz_interval.reshape(len(x_lz_interval), 1), x_lz_interval.reshape(len(x_lz_interval), 1)), axis=0) # [m] column vector of duplicated landing zone edge x-coordinates
full_y_lz_coords = np.concatenate((landing_zone_upper_edge.reshape(len(landing_zone_upper_edge), 1), landing_zone_lower_edge.reshape(len(landing_zone_lower_edge), 1)), axis=0) # [m] column vector of landing zone edge y-coordinates
full_lz_coords = np.concatenate((full_x_lz_interval, full_y_lz_coords), axis=1) # [m] complete landing zone coordinates

ellipse = EllipseModel() # create best-fit ellipse model
if (ellipse.estimate(full_lz_coords)): # fit the best-fit model (True if successful)
    landing_zone_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor='r', facecolor="None")

plt.ion()
# Plot of the Domain of Nominal Trajectories
dnt_fig = plt.figure()
dnt_ax = dnt_fig.add_subplot()
dnt_ax.plot(0, 0, 'k.', label="Launch Site") # plot launch site (i.e., inertial CS origin)
dnt_ax.plot(landing_zone_x, landing_zone_y, color='r', marker="+") # plot center of the landing zone
dnt_ax.plot(x_lz_interval, landing_zone_upper_edge, color='r', label="Landing Zone") # plot upper semi-circle of the landing zone
dnt_ax.plot(x_lz_interval, landing_zone_lower_edge, color='r') # plot lower semi-circle of the landing zone
dnt_ax.add_patch(landing_zone_patch) # plot landing zone patch (for some reason, must be plotted for `contains_point` method to work properly)
dnt_ax.set_xlabel("X - East [m]")
dnt_ax.set_ylabel("Y - North [m]")
dnt_ax.axis('equal') # set axis limits equivalent
dnt_ax.grid(which="major", axis="both")

# Number of Trajectory Simulations to Run
if (len(sys.argv) != 2):
    num_of_trajectories = 100
else:
    num_of_trajectories = int(sys.argv[1])

# Loop to Perform Monte Carlo Simulations
for traj in range(num_of_trajectories):
    random_inclination = np.ndarray.item(np.random.normal(loc=optimal_inclination, scale=0.25, size=1)) # [deg] sample launch inclination from normal distribution centered at optimal launch inclination
    random_heading = np.ndarray.item(np.random.normal(loc=optimal_heading, scale=0.25, size=1)) # [deg] sample launch heading from normal distribution centered at optimal launch heading

    print(f"Iteration: {traj+1}, Inclination: {round(random_inclination, 2)}, Heading: {round(random_heading, 2)} deg")
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=1.5, # [m] length in which the rocket will be attached to the launch rail
        inclination=random_inclination, # [deg] rail inclination relative to the ground
        heading=random_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    ) # run trajectory simulation
    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

    if (landing_zone_patch.contains_point(point=dnt_ax.transData.transform(values=(test_flight.x_impact, test_flight.y_impact)))): # trajectory's impact location is within the landing zone
        dnt_ax.plot(test_flight.x_impact, test_flight.y_impact, 'b.')
        landing_zone_flag = 1 # to be written to the CSV file
    else: # trajectory's impact location is not within the landing zone
        dnt_ax.plot(test_flight.x_impact, test_flight.y_impact, 'r.')
        landing_zone_flag = 0 # to be written to the CSV file
    plt.pause(0.5)

    output_file = open(f"{CSV_output_dir}/trajectory_" + str(traj+1) + ".csv", 'w', newline="") # output CSV file containing trajectory information
    writer = csv.writer(output_file) # CSV writer for output file containing trajectory information
    writer.writerow(trajectory_csv_header) # write header row of output CSV file containing optimal trajectory information

    # Write data at each time step to the output CSV file
    for time_step in solution_time:
        time_step_data = [time_step, test_flight.x(time_step), test_flight.y(time_step), landing_zone_flag]
        writer.writerow(time_step_data)
    output_file.close()