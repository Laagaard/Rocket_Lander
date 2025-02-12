# Libraries
import math
import matplotlib.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import DART_rocket, launch_site, launch_rail_length

PROP04_altitude_threshold = 75 # [m] minimum apogee required by PROP.04
PROP05_lateral_displacement_threshold = 50 # [m] minimum lateral displacement required by PROP.05

launch_inclination = 85 # [deg]
launch_heading = 30 # [deg]

test_flight = Flight(
    rocket=DART_rocket,
    environment=launch_site,
    rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
) # run trajectory simulation

solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

lateral_displacement = [math.sqrt(test_flight.x(time_step)**2 + test_flight.y(time_step)**2) for time_step in solution_time] # [m]

fig = plt.figure()
trajectory_ax = fig.add_subplot()
trajectory_ax.grid(which="both", axis="both")
trajectory_ax.plot(lateral_displacement, test_flight.altitude(solution_time), 'k-') # plot the trajectory

apogee_index = np.argmax(test_flight.altitude(solution_time)) # index of max altitude
vertical_trans = transforms.blended_transform_factory(trajectory_ax.get_yticklabels()[0].get_transform(), trajectory_ax.transData)
trajectory_ax.axhline(y=PROP04_altitude_threshold, color='g', linestyle='--') # plot horizontal line at PROP.04 threshold
trajectory_ax.plot(lateral_displacement[apogee_index], max(test_flight.altitude(solution_time)), 'g.', markersize=10) # plot marker at trajectory apogee
trajectory_ax.text(0, PROP04_altitude_threshold, s=f"{PROP04_altitude_threshold}", color="green", transform=vertical_trans, ha="right", va="center") # add text on vertical axis of PROP.04 threshold

trajectory_ax.axvline(x=PROP05_lateral_displacement_threshold, color='b', linestyle='--') # plot vertical line at PROP.05 threshold
trajectory_ax.plot(lateral_displacement[-1], test_flight.altitude(solution_time[-1]), 'b.', markersize=10) # plot marker at trajectory impact point

trajectory_ax.set_xlabel("Lateral Displacement [m]")
trajectory_ax.set_ylabel("Altitude [m]")
trajectory_ax.set_title(f"Trajectory Simulation \n(Inclination: {launch_inclination} deg)")

plt.tight_layout()
plt.savefig(f"PROP04_PROP05_Verification.png", transparent=True, dpi=1000) # save the figure with a transparent background
plt.show()