# Libraries
import geopy.distance
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from rocketpy import Flight
from skimage.measure import EllipseModel
import sys
sys.path.append("../")
from setup import all_landing_zone_perimeters, DART_rocket, date_dir_with_time, launch_area_ax, launch_rail_length, launch_site
from SYS09 import check_dnt, launch_area_ax, optimal_heading, optimal_inclination

def parachute_trigger(p, h, y):
    '''
    Method to Trigger an In-Flight Abort (i.e., Deploy the Parachute) if the Rocket Exits the DNT

    Parameters
    ----------
    `p`: pressure considering parachute noise signal [Pa]
    `h`: height above ground level considering parachute noise signal [m]
    `y`: state vector in the form [x, y, z, vx, vy, vz, e0, e1, e2, e3, w1, w2, w3]

    Returns
    -------
    Boolean indicating whether or not to deploy the parachute (`True`: deploy the parachute. `False`: do not deploy the parachute.)
    '''
    global abort_counts
    global date_dir_with_time
    global landing_zone_patch
    global launch_site

    DNT_FILE_PATH = f"../DNT/{date_dir_with_time}/DNT.csv" # TBR, not a robust approach, only works from a directory one level higher

    if (not os.path.exists(DNT_FILE_PATH)):
        print("TRIGGER FUNCTION INACTIVE - DNT FILE NOT FOUND")
        return False

    x_pos = y[0] # rocket x position, positive East
    y_pos = y[1] # rocket y position, positive North
    z_vel = y[5] # rocket vertical velocity, positive up

    if (x_pos == 0):
        updated_coordinates = geopy.distance.distance(meters=math.sqrt(x_pos**2 + y_pos**2)).destination((launch_site.latitude, launch_site.longitude), bearing=90) # (https://geopy.readthedocs.io/en/stable/index.html?highlight=destination#geopy.distance.Distance.destination)
    else:
        bearing = 90 - math.degrees(math.atan2(y_pos, x_pos)) # [deg]
        updated_coordinates = geopy.distance.distance(meters=math.sqrt(x_pos**2 + y_pos**2)).destination((launch_site.latitude, launch_site.longitude), bearing=bearing)

    updated_long = updated_coordinates[1]
    updated_lat = updated_coordinates[0]

    abort_count_threshold = 3 # number of trajectory positions that must exit the DNT to trigger an abort

    if (landing_zone_patch.contains_point(point=launch_area_ax.transData.transform(values=(updated_long, updated_lat)))): # if the trajectory is within the landing zone
        abort_bool = False # don't trigger an abort (but don't reset the abort counter)
    else: # check the rocket's position against the DNT
        abort_bool, abort_counts = check_dnt(DNT_FILE_PATH=DNT_FILE_PATH,
                                            current_time=2, # TBR, not a great solution (but technically works, and not sure how else to pass in current flight time)
                                            current_long=updated_long,
                                            current_lat=updated_lat,
                                            abort_counts=abort_counts,
                                            abort_count_threshold=abort_count_threshold) # check the rocket's position against the relevant DNT
    if (abort_bool and z_vel < 0): # if the rocket has exited the DNT and is at or past apogee (i.e., no longer ascending)
        print("PARACHUTE TRIGGERED")
        return True # deploy the parachute
    else:
        return False # do not deploy the parachute

optimal_landing_zone_df = pd.read_csv(f"../DNT/{date_dir_with_time}/optimal_landing_zone.csv") # df of optimal landing zone information
optimal_perimeter_coords = all_landing_zone_perimeters[optimal_landing_zone_df["index"][0]] # coordinates of optimal landing zone perimeter

ellipse = EllipseModel()
if (ellipse.estimate(optimal_perimeter_coords)): # fit the best-fit model to the optimal landing zone perimeter coordss
    landing_zone_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor='r', facecolor="None")
    launch_area_ax.add_patch(landing_zone_patch)

# Parachute Characteristics
C_D = 0.84 # [unitless] parachute drag coefficient
parachute_reference_area=math.pi*(30*0.0254/2)**2 # [m^2] reference area of parachute

# Construct Parachute
main = DART_rocket.add_parachute(
    name="main", # name of the parachute (no impact on simulation)
    cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
    trigger=parachute_trigger,
    sampling_rate=10, # [Hz] sampling rate in which the trigger function works (used to simulate sensor refresh rates)
    lag=0, # [s] time between the ejection system is triggered and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
    noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlation) used to add noise to the pressure signal
)

num_trajectories = 50 # number of trajectories to simulate
for elem in range(num_trajectories):
    launch_inclination = np.random.uniform(low=optimal_inclination - 1, high=optimal_inclination + 1) # [deg] randomly draw launch inclination
    launch_heading = np.random.uniform(low=optimal_heading - 1, high=optimal_heading + 1) # [deg] randomly draw launch heading
    print(f"Iteration: {elem}, Inclination: {round(launch_inclination, 2)} deg, Heading: {round(launch_heading, 2)} deg")

    abort_counts = 0 # counter to track the number of trajectory positions that exited the DNT
    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
        inclination=launch_inclination, # [deg] rail inclination relative to the ground
        heading=launch_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=False # if True, decouples ODE time step from parachute trigger functions sampling rate
    ) # run trajectory simulation

    if (len(test_flight.parachute_events) >= 1): # if the parachute deployed (i.e., an abort was triggered)
        abort_color = 'r'
    else:
        abort_color = 'g'

    solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
    launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), color=abort_color)

launch_area_ax.set_title("SYS.10 Verification")
plt.tight_layout()
plt.savefig(f"SYS10_Verification.png", transparent=True, dpi=1000) # save the figure with a transparent background
plt.show()