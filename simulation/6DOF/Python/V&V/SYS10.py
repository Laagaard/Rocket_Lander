# Libraries
import geopy.distance
import math
import matplotlib.pyplot as plt
import os
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import DART_rocket, date_dir_with_time, launch_area_ax, launch_rail_length, launch_site
from SYS09 import check_dnt, launch_area_ax, optimal_heading, optimal_inclination

def parachute_trigger(p=101325, h=0, y=[0,0,0,0,0,0,0,0,0,0,0,0,0]):
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
    global date_dir_with_time
    global launch_site

    DNT_FILE_PATH = f"../DNT/{date_dir_with_time}/DNT.csv" # TBR, not a robust approach, only works from a directory one level higher

    if (not os.path.exists(DNT_FILE_PATH)):
        print("TRIGGER FUNCTION INACTIVE - DNT FILE NOT FOUND")
        return False
    
    print("TRIGGER FUNCTION ACTIVE")
    print(p, h, y[0], y[1])

    x_pos = y[0] # rocket x position, positive East
    y_pos = y[1] # rocket y position, positive North

    if (x_pos == 0):
        updated_coordinates = geopy.distance.distance(meters=math.sqrt(x_pos**2 + y_pos**2)).destination((launch_site.latitude, launch_site.longitude), bearing=90) # (https://geopy.readthedocs.io/en/stable/index.html?highlight=destination#geopy.distance.Distance.destination)
    else:
        updated_coordinates = geopy.distance.distance(meters=math.sqrt(x_pos**2 + y_pos**2)).destination((launch_site.latitude, launch_site.longitude), bearing=math.degrees(math.atan2(y_pos/x_pos)))

    updated_long = updated_coordinates[1]
    updated_lat = updated_coordinates[0]

    abort_counts = 0 # counter to track the number of trajectory positions that exited the DNT
    abort_count_threshold = 3 # number of trajectory positions that must exit the DNT to trigger an abort

    abort_bool, abort_counts = check_dnt(DNT_FILE_PATH=DNT_FILE_PATH,
                                         current_time=2, # TBR, this is a TERRIBLE fix
                                         current_long=updated_long,
                                         current_lat=updated_lat,
                                         abort_counts=abort_counts,
                                         abort_count_threshold=abort_count_threshold)

    return True

# Parachute Characteristics
C_D = 0.84 # [unitless] parachute drag coefficient
parachute_reference_area=math.pi*(30*0.0254/2)**2 # [m^2] reference area of parachute

# Construct Parachute
main = DART_rocket.add_parachute(
    name="main", # name of the parachute (no impact on simulation)
    cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
    trigger="apogee",
    sampling_rate=10, # [Hz] sampling rate in which the trigger function works (used to simulate sensor refresh rates)
    lag=0, # [s] time between the ejection system is triggers and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
    noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlation) used to add noise to the pressure signal
)

test_flight = Flight(
    rocket=DART_rocket,
    environment=launch_site,
    rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=optimal_inclination, # [deg] rail inclination relative to the ground
    heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
) # run trajectory simulation

solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), 'k')

test_flight.plots.trajectory_3d()
# plt.show()