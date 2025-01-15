# Libraries
import contextily as cx
import csv
import math
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
from mpl_toolkits.basemap import Basemap
import numpy as np
import pandas as pd
from rocketpy import Flight

trajectory_dataset_df = pd.read_csv("trajectory_dataset.csv") # read trajectory dataset into pandas (pd) dataframe (df)
optimal_landing_zone_df = pd.read_csv("optimal_landing_zone.csv") # read optimal landing zone information into dataframe

landing_zone_number = optimal_landing_zone_df["index"][0] + 1

# Determine Optimal Launch Inclination & Heading
impact_triangulation = mtri.Triangulation(x=trajectory_dataset_df["longitude"], y=trajectory_dataset_df["latitude"])
inclination_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=trajectory_dataset_df["Inclination"])
optimal_inclination = inclination_interpolator.__call__(optimal_landing_zone_df["longitude"][0], optimal_landing_zone_df["latitude"][0]) # [deg] optimal launch inclination for optimal landing zone center
heading_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=trajectory_dataset_df["Heading"])
optimal_heading = heading_interpolator.__call__(optimal_landing_zone_df["longitude"][0], optimal_landing_zone_df["latitude"][0]) # [deg] optimal launch heading for optimal landing zone center

trajectory_csv_header = ["Time", "longitude", "latitude", "altitude", "x_vel", "y_vel"] # CSV header for all Monte Carlo trajectory simulation files

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from dataset_generation import figures_output_dir, launch_area_ax
    from setup import DART_rocket, launch_site

    print("\n---------- LAUNCH PARAMETERS ----------")
    print(f"Landing Zone: {landing_zone_number}")
    print(f"Optimal Inclination: {np.round(optimal_inclination, 2)} deg")
    print(f"Optimal Heading: {np.round(optimal_heading, 2)} deg")
    print("---------------------------------------\n")

    # Simulate the Flight with Optimal Launch Parameters
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=1.5, # [m] length in which the rocket will be attached to the launch rail
        inclination=optimal_inclination, # [deg] rail inclination relative to the ground
        heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    )

    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

    output_file = open("optimal_trajectory.csv", 'w', newline="") # output CSV file containing trajectory information
    writer = csv.writer(output_file) # CSV writer for output file containing trajectory information
    writer.writerow(trajectory_csv_header) # write header row of output CSV file containing optimal trajectory information

    # Write data at each time step to the output CSV file
    for time_step in solution_time:
        time_step_data = [time_step, test_flight.longitude(time_step), test_flight.latitude(time_step), test_flight.z(time_step), test_flight.vx(time_step), test_flight.vy(time_step)]
        writer.writerow(time_step_data)
    output_file.close()

    launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), 'b', label="Trajectory")
    launch_area_ax.set_title(f"Trajectory \n(Inclination: {np.round(optimal_inclination, 2)} deg, Heading: {np.round(optimal_heading, 2)} deg)")
    launch_area_ax.legend(loc="best")
    plt.savefig(f"{figures_output_dir}/optimal_trajectory.png")
    plt.show()