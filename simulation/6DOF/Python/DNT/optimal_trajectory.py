# Libraries
import csv
import math
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import pandas as pd
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import automation_flag, DART_rocket, date_dir_date_only, date_dir_with_time, date_string_date_only, date_string_with_time, launch_rail_length, launch_site

trajectory_dataset_df = pd.read_csv(f"{date_dir_with_time}/trajectory_dataset.csv") # read trajectory dataset into pandas (pd) dataframe (df)
optimal_landing_zone_df = pd.read_csv(f"{date_dir_with_time}/optimal_landing_zone.csv") # read optimal landing zone information into dataframe

landing_zone_number = optimal_landing_zone_df["index"][0] + 1

# Determine Optimal Launch Inclination & Heading
impact_triangulation = mtri.Triangulation(x=trajectory_dataset_df["longitude"], y=trajectory_dataset_df["latitude"])
inclination_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=trajectory_dataset_df["Inclination"])
optimal_inclination = inclination_interpolator.__call__(optimal_landing_zone_df["longitude"][0], optimal_landing_zone_df["latitude"][0]) # [deg] optimal launch inclination for optimal landing zone center
heading_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=trajectory_dataset_df["Heading"])
optimal_heading = heading_interpolator.__call__(optimal_landing_zone_df["longitude"][0], optimal_landing_zone_df["latitude"][0]) # [deg] optimal launch heading for optimal landing zone center

launch_information_header = ["DateTime", "Inclination", "Heading", "Landing Zone", "Impact Angle"] # CSV header for optimal trajectory launch information
trajectory_state_history_header = ["Time", "longitude", "latitude", "altitude", "x_vel", "y_vel"] # CSV header for all DNT trajectory simulation files

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from dataset_generation import figures_output_dir, launch_area_ax

    if (not automation_flag):
        print("\n---------- LAUNCH PARAMETERS ----------")
        print(f"Landing Zone: {landing_zone_number}")
        print(f"Optimal Inclination: {np.round(optimal_inclination, 2)} deg")
        print(f"Optimal Heading: {np.round(optimal_heading, 2)} deg")
        print("---------------------------------------\n")

    # Simulate the Flight with Optimal Launch Parameters
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
        inclination=optimal_inclination, # [deg] rail inclination relative to the ground
        heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    )

    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

    horz_vel = math.sqrt(test_flight.vx(solution_time)[-1]**2 + test_flight.vy(solution_time)[-1]**2) # [m/s] inertial horizontal velocity at impact
    vert_vel = test_flight.vz(solution_time)[-1] # [m/s] inertial vertical velocity at impact
    final_angle = math.degrees(math.atan2(vert_vel, horz_vel)) # [deg] inclination (angle relative to vertical) at impact

    launch_information_file = open(f"{date_dir_date_only}/{date_string_date_only}.csv", 'w', newline="")
    launch_information_writer = csv.writer(launch_information_file) # CSV writer for output file containing optimal trajectory launch information
    launch_information_writer.writerow(launch_information_header) # write header row of output CSV file containing optimal trajectory launch information
    launch_information_writer.writerow([date_string_with_time, np.round(optimal_inclination, 2), np.round(optimal_heading, 2), landing_zone_number, np.round(final_angle, 2)])
    launch_information_file.close()

    trajectory_state_history_file = open(f"{date_dir_with_time}/optimal_trajectory.csv", 'w', newline="") # output CSV file containing trajectory information
    trajectory_state_history_writer = csv.writer(trajectory_state_history_file) # CSV writer for output file containing trajectory information
    trajectory_state_history_writer.writerow(trajectory_state_history_header) # write header row of output CSV file containing optimal trajectory information

    # Write data at each time step to the output CSV file
    for time_step in solution_time:
        time_step_data = [time_step, test_flight.longitude(time_step), test_flight.latitude(time_step), test_flight.z(time_step), test_flight.vx(time_step), test_flight.vy(time_step)]
        trajectory_state_history_writer.writerow(time_step_data)
    trajectory_state_history_file.close()

    launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), 'b', label="Trajectory")
    launch_area_ax.set_title(f"Optimal Trajectory \n(Inclination: {np.round(optimal_inclination, 2)} deg, Heading: {np.round(optimal_heading, 2)} deg)")
    plt.tight_layout()
    print(f"{date_string_with_time}, Inclination: {np.round(optimal_inclination, 2)} deg, Heading: {np.round(optimal_heading, 2)} deg, LZ: {landing_zone_number}, Impact Angle: {np.round(final_angle, 2)} deg")
    plt.savefig(f"{figures_output_dir}/optimal_trajectory.png", transparent=True, dpi=1000) # save the figure with a transparent background

    if (not automation_flag): # if the script is being run manually
        plt.show()