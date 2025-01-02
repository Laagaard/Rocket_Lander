# Libraries
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import pandas as pd
from rocketpy import Flight
from dataset_generation import landing_zone_x, landing_zone_y, x_lz_interval, landing_zone_upper_edge, landing_zone_lower_edge
from setup import DART_rocket, launch_site

df = pd.read_csv("trajectory_dataset.csv") # read trajectory dataset into pandas (pd) dataframe (df)

# Determine Optimal Launch Inclination & Heading
impact_triangulation = mtri.Triangulation(x=df["x_impact"], y=df["y_impact"])
inclination_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=df["Inclination"])
optimal_inclination = inclination_interpolator.__call__(landing_zone_x, landing_zone_y) # [deg] optimal launch inclination for desired landing zone center
heading_interpolator = mtri.LinearTriInterpolator(impact_triangulation, z=df["Heading"])
optimal_heading = heading_interpolator.__call__(landing_zone_x, landing_zone_y) # [deg] optimal launch heading for desired landing zone center

# Simulate the Flight with Optimal Launch Parameters
test_flight = Flight(
    rocket=DART_rocket,
    environment=launch_site,
    rail_length=1.5, # [m] length in which the rocket will be attached to the launch rail
    inclination=optimal_inclination, # [deg] rail inclination relative to the ground
    heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    print("\n---------- LAUNCH PARAMETERS ----------")
    print(f"Optimal Inclination: {np.round(optimal_inclination, 2)} deg")
    print(f"Optimal Heading: {np.round(optimal_heading, 2)} deg")
    print("---------------------------------------\n")

    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

    # Trajectory Plot
    trajectory_fig = plt.figure()
    trajectory_ax = trajectory_fig.add_subplot()
    trajectory_ax.plot(landing_zone_x, landing_zone_y, color='r', marker="+") # plot center of the landing zone
    trajectory_ax.plot(x_lz_interval, landing_zone_upper_edge, color='r', label="Landing Zone") # plot upper semi-circle of the landing zone
    trajectory_ax.plot(x_lz_interval, landing_zone_lower_edge, color='r') # plot lower semi-circle of the landing zone
    trajectory_ax.plot(test_flight.x(solution_time), test_flight.y(solution_time), 'b', label="Trajectory")
    trajectory_ax.set_xlabel("X - East [m]")
    trajectory_ax.set_ylabel("Y - North [m]")
    trajectory_ax.set_title(f"Trajectory \n(Inclination: {np.round(optimal_inclination, 2)} deg, Heading: {np.round(optimal_heading, 2)} deg)")
    trajectory_ax.grid(which="major", axis="both")
    trajectory_ax.axis('equal') # set axis limits equivalent
    trajectory_ax.legend(loc="best")
    plt.show()