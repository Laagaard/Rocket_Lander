# Libraries
import csv
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import pandas as pd
from rocketpy import Flight
from dataset_generation import landing_zone_x, landing_zone_y

df = pd.read_csv("trajectory_dataset.csv") # read trajectory dataset into pandas (pd) dataframe (df)

# Determine Optimal Launch Inclination & Heading
impact_triangulation = mtri.Triangulation(x=df["x_impact"], y=df["y_impact"])
inclination_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=df["Inclination"])
optimal_inclination = inclination_interpolator.__call__(landing_zone_x, landing_zone_y) # [deg] optimal launch inclination for desired landing zone center
heading_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=df["Heading"])
optimal_heading = heading_interpolator.__call__(landing_zone_x, landing_zone_y) # [deg] optimal launch heading for desired landing zone center

trajectory_csv_header = ["Time", "x_pos", "y_pos", "z_pos", "x_vel", "y_vel"] # CSV header for all Monte Carlo trajectory simulation files

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    from dataset_generation import landing_zone_patch
    from setup import DART_rocket, launch_site

    print("\n---------- LAUNCH PARAMETERS ----------")
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
        time_step_data = [time_step, test_flight.x(time_step), test_flight.y(time_step), test_flight.z(time_step), test_flight.vx(time_step), test_flight.vy(time_step)]
        writer.writerow(time_step_data)
    output_file.close()

    # Trajectory Plot
    trajectory_fig = plt.figure()
    trajectory_ax = trajectory_fig.add_subplot()
    trajectory_ax.plot(landing_zone_x, landing_zone_y, color='r', marker="+") # plot center of the landing zone
    trajectory_ax.add_patch(landing_zone_patch)
    trajectory_ax.plot(test_flight.x(solution_time), test_flight.y(solution_time), 'b', label="Trajectory")
    trajectory_ax.set_xlabel("X - East [m]")
    trajectory_ax.set_ylabel("Y - North [m]")
    trajectory_ax.set_title(f"Trajectory \n(Inclination: {np.round(optimal_inclination, 2)} deg, Heading: {np.round(optimal_heading, 2)} deg)")
    trajectory_ax.grid(which="major", axis="both")
    trajectory_ax.axis('equal') # set axis limits equivalent
    trajectory_ax.legend(loc="best")
    plt.show()