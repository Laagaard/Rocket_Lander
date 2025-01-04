# Libraries
import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from rocketpy import Flight
import shutil
import stat

CSV_output_dir = "dnt_trajectories"

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from dataset_generation import landing_zone_patch, landing_zone_x, landing_zone_y, x_lz_interval, landing_zone_upper_edge, landing_zone_lower_edge
    from optimal_trajectory import trajectory_csv_header, inclination_interpolator, heading_interpolator
    from setup import launch_site, DART_rocket

    # Function for `shutil.rmtree` to call on "Access is denied" error from read-only folder
    def remove_readonly(func, path, excinfo):
        os.chmod(path, stat.S_IWRITE)
        func(path)

    if os.path.exists(CSV_output_dir):
        shutil.rmtree(CSV_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(CSV_output_dir) # Create folder for CSV files of DNT simulation data

    optimal_df = pd.read_csv("optimal_trajectory.csv") # df of optimal trajectory data

    full_x_lz_interval = np.append(x_lz_interval, x_lz_interval)

    # Plot DNT Impact Locations
    dnt_trajectories_fig = plt.figure()
    dnt_trajectories_ax = dnt_trajectories_fig.add_subplot()
    dnt_trajectories_ax.plot(0, 0, 'k.', label="Launch Site") # plot launch site (i.e., inertial CS origin)
    dnt_trajectories_ax.plot(landing_zone_x, landing_zone_y, color='r', marker="+") # plot center of the landing zone
    dnt_trajectories_ax.add_patch(landing_zone_patch) # plot landing zone patch
    dnt_trajectories_ax.set_xlabel("X - East [m]")
    dnt_trajectories_ax.set_ylabel("Y - North [m]")
    dnt_trajectories_ax.axis('equal') # set axis limits equivalent
    dnt_trajectories_ax.grid(which="major", axis="both")
    dnt_trajectories_ax.plot(optimal_df["x_pos"], optimal_df["y_pos"], 'b', label="Optimal Trajectory")
    dnt_trajectories_ax.set_title(f"DNT Simulation Results \n(Trajectory Simulations: {len(full_x_lz_interval)})")

    iteration_counter = 0 # counter of the number of trajectories simulated
    for idx in range(len(full_x_lz_interval)): # iterate over x-coordinates in the landing zone interval
        x_coord = full_x_lz_interval[idx] # current x-coordinate of the discretized landing zone
        if (idx < len(x_lz_interval)):
            y_coord = landing_zone_upper_edge[idx] # y-coordinate on the upper edge of the discretized landing zone
        else:
            y_coord = landing_zone_lower_edge[idx - len(x_lz_interval)] # y-coordinate on the lower edge of the discretized landing zone
        launch_inclination = inclination_interpolator.__call__(x_coord, y_coord) # [deg]
        launch_heading = heading_interpolator.__call__(x_coord, y_coord) # [deg]

        # Simulate the Flight with Desired Launch Parameters
        test_flight = Flight(
            rocket=DART_rocket,
            environment=launch_site,
            rail_length=1.5, # [m] length in which the rocket will be attached to the launch rail
            inclination=launch_inclination, # [deg] rail inclination relative to the ground
            heading=launch_heading, # [deg] heading angle relative to North (East = 90)
            time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
        )
        iteration_counter += 1
        print(f"Iteration: {idx}, Inclination: {np.round(launch_inclination, 2)} deg, Heading: {np.round(launch_heading, 2)} deg")

        solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
        dnt_trajectories_ax.plot(test_flight.x_impact, test_flight.y_impact, 'b.')

        output_file = open(f"{CSV_output_dir}/trajectory_" + str(iteration_counter) + ".csv", 'w', newline="") # output CSV file containing trajectory information
        writer = csv.writer(output_file) # CSV writer for output file containing trajectory information
        writer.writerow(trajectory_csv_header) # write header row of output CSV file containing optimal trajectory information

        # Write data at each time step to the output CSV file
        for time_step in solution_time:
            time_step_data = [time_step, test_flight.x(time_step), test_flight.y(time_step), test_flight.z(time_step), test_flight.vx(time_step), test_flight.vy(time_step)]
            writer.writerow(time_step_data)
        output_file.close()

    dnt_trajectories_ax.legend(loc="best")
    plt.show()

    print(f"\n---------- SIMULATION RESULTS ----------")
    print(f"Trajectories Simulated: {iteration_counter}\n")