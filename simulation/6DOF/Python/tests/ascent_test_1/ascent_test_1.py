# Python Libraries
import csv
import datetime
import math
import matplotlib.pyplot as plt
from rocketpy import Environment, Flight, prints
import sys
sys.path.append("../../")
# DART Modules
import setup

launch_date = datetime.datetime.strptime("02-23-2025", "%m-%d-%Y") # launch date
launch_hour = 13
launch_time = datetime.time(hour=launch_hour, minute=00) # # launch time (hr, min) (input as EST)
launch_date_and_time = datetime.datetime.combine(launch_date, launch_time) # launch date and time

launch_site_latitude_ROAR = 28 + (33/60) + (48/3600) # [deg] North, launch site latitude (if launching with ROAR, NAR section 795))
launch_site_longitude_ROAR = -(81 + (1/60) + (2/3600)) # [deg] West, launch site longitude (if launching with ROAR, NAR section 795))

launch_inclination = 88 # [deg] from horizontal
launch_heading = 0 # [deg] CW from North

# Construct Launch Site Environment
launch_site = Environment(
    date=launch_date_and_time, # launch date and time
    latitude=launch_site_latitude_ROAR, # [deg] positive corresponds to North
    longitude=launch_site_longitude_ROAR, # [deg] positive corresponds to East
    elevation=50, # [m] launch site elevation above sea level
    timezone="EST", # specify launch site time zone
    max_expected_height=250 # [m] maximum altitude to keep weather data (must be above sea level)
)

wind_vel_mag = 1.34 # [m/s] magnitude of wind velocity
launch_site.process_custom_atmosphere(wind_u=-wind_vel_mag*math.cos(math.radians(67.5)), wind_v=wind_vel_mag*math.sin(math.radians(67.5)))

launch_site_prints = prints.environment_prints._EnvironmentPrints(launch_site)
launch_site_prints.all()

# Simulate the Flight (w/o parachute)
test_flight_no_parachute = Flight(
    rocket=setup.DART_rocket_1,
    environment=launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

# Parachute Characteristics
C_D = 0.84 # [unitless] parachute drag coefficient
parachute_reference_area=math.pi*(30*0.0254/2)**2 # [m^2] reference area of parachute

# Construct Parachute
main_parachute = setup.DART_rocket_1.add_parachute(
    name="main", # name of the parachute (no impact on simulation)
    cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
    trigger="apogee",
    sampling_rate=10, # [Hz] sampling rate in which the trigger function works (used to simulate sensor refresh rates)
    lag=0, # [s] time between the ejection system is triggered and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
    noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlation) used to add noise to the pressure signal
)

# Simulate the Flight (w/ parachute)
test_flight_parachute = Flight(
    rocket=setup.DART_rocket_1,
    environment=launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

solution_time_no_parachute = [solution_step[0] for solution_step in test_flight_no_parachute.solution] # [s] time array of solution
setup.launch_area_ax.plot(test_flight_no_parachute.longitude(solution_time_no_parachute), test_flight_no_parachute.latitude(solution_time_no_parachute), 'r-')

solution_time_parachute = [solution_step[0] for solution_step in test_flight_parachute.solution] # [s] time array of solution
setup.launch_area_ax.plot(test_flight_parachute.longitude(solution_time_parachute), test_flight_parachute.latitude(solution_time_parachute), 'b-')

test_flight_no_parachute.export_data("ascent_test_1_trajectory_no_parachute.csv", "longitude", "latitude", "x", "y", "altitude", "e0", "e1", "e2", "e3")
test_flight_parachute.export_data("ascent_test_1_trajectory_parachute.csv", "longitude", "latitude", "x", "y", "altitude", "e0", "e1", "e2", "e3")

print("\n---------- FLIGHT INFORMATION ----------")
print(f"Inclination: {round(launch_inclination, 2)} deg")
print(f"Heading: {round(launch_heading, 2)} deg")
print(f"Apogee Time: {round(test_flight_parachute.apogee_time, 2)} [s]")
print(f"Apogee Altitude: {round(test_flight_parachute.apogee, 2)} [m]")
print("----------------------------------------\n")

plt.show()