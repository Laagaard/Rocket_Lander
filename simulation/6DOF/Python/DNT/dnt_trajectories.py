# Libraries
import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from rocketpy import Flight
import shutil
from dataset_generation import figures_output_dir, date_dir_with_time, launch_area_ax, all_landing_zone_perimeters

CSV_output_dir = f"{date_dir_with_time}/dnt_trajectories"

optimal_landing_zone_df = pd.read_csv(f"{date_dir_with_time}/optimal_landing_zone.csv") # df of optimal landing zone information

optimal_perimeter_coords = all_landing_zone_perimeters[optimal_landing_zone_df["index"][0]] # coordinates of optimal landing zone perimeter
number_of_perimeter_points = max(optimal_perimeter_coords.shape) # number of points comprising the optimal landing zone perimeter

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from optimal_trajectory import trajectory_state_history_header, inclination_interpolator, heading_interpolator
    from setup import automation_flag, launch_rail_length, launch_site, DART_rocket_1, remove_readonly

    if os.path.exists(CSV_output_dir):
        shutil.rmtree(CSV_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(CSV_output_dir) # Create folder for all DNT trajectories

    if os.path.exists(CSV_output_dir):
        shutil.rmtree(CSV_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(CSV_output_dir) # Create folder for CSV files of DNT simulation data

    optimal_trajectory_df = pd.read_csv(f"{date_dir_with_time}/optimal_trajectory.csv") # df of optimal trajectory data

    # Plot DNT Impact Locations
    launch_area_ax.plot(optimal_trajectory_df["longitude"], optimal_trajectory_df["latitude"], 'b', label="Optimal Trajectory")
    launch_area_ax.set_title(f"DNT Trajectory Simulation Results")

    iteration_counter = 0 # counter of the number of trajectories simulated
    for idx in range(number_of_perimeter_points): # iterate over all points comprising the optimal landing zone perimeter
        longitude_coord = optimal_perimeter_coords[idx, 0] # longitude coordinate of the current point on the optimal landing zone perimeter
        latitude_coord = optimal_perimeter_coords[idx, 1] # latitude coordinate of the current point on the optimal landing zone perimeter
        launch_inclination = inclination_interpolator.__call__(longitude_coord, latitude_coord) # [deg]
        launch_heading = heading_interpolator.__call__(longitude_coord, latitude_coord) # [deg]

        try:
            # Simulate the Flight with Desired Launch Parameters
            test_flight = Flight(
                rocket=DART_rocket_1,
                environment=launch_site,
                rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
                inclination=launch_inclination, # [deg] rail inclination relative to the ground
                heading=launch_heading, # [deg] heading angle relative to North (East = 90)
                time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
            )
        except (ValueError): # ValueError raised when longitude and latitude coordinates exceed interpolator bounds
            print("SKIPPING DATA POINT")
            continue
        iteration_counter += 1

        if (not automation_flag):
            print(f"Iteration: {idx}, Inclination: {np.round(launch_inclination, 2)} deg, Heading: {np.round(launch_heading, 2)} deg")

        solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
        impact_longitude = test_flight.longitude(solution_time)[-1]
        impact_latitude = test_flight.latitude(solution_time)[-1]
        launch_area_ax.plot(impact_longitude, impact_latitude, 'b.')

        output_file = open(f"{CSV_output_dir}/trajectory_" + str(iteration_counter) + ".csv", 'w', newline="") # output CSV file containing trajectory information
        writer = csv.writer(output_file) # CSV writer for output file containing trajectory information
        writer.writerow(trajectory_state_history_header) # write header row of output CSV file containing optimal trajectory information

        # Write data at each time step to the output CSV file
        for time_step in solution_time:
            time_step_data = [time_step, test_flight.longitude(time_step), test_flight.latitude(time_step), test_flight.z(time_step), test_flight.vx(time_step), test_flight.vy(time_step)]
            writer.writerow(time_step_data)
        output_file.close()

    plt.tight_layout()
    plt.savefig(f"{figures_output_dir}/dnt_trajectories.png", transparent=True, dpi=1000) # save the figure with a transparent background

    if (not automation_flag):
        print(f"\n---------- SIMULATION RESULTS ----------")
        print(f"Trajectories Attempted: {number_of_perimeter_points}")
        print(f"Trajectories Used: {iteration_counter}\n")
        plt.show()