# Python Libraries
import csv
import math
import matplotlib.pyplot as plt
from rocketpy import Flight
import sys
sys.path.append("../")
# DART Modules
import setup

launch_inclination = 88 # [deg] from horizontal
launch_heading = 0 # [deg] CW from North

# Simulate the Flight (w/o parachute)
test_flight_no_parachute = Flight(
    rocket=setup.DART_rocket_1,
    environment=setup.launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

solution_time_no_parachute = [solution_step[0] for solution_step in test_flight_no_parachute.solution] # [s] time array of solution
setup.launch_area_ax.plot(test_flight_no_parachute.longitude(solution_time_no_parachute), test_flight_no_parachute.latitude(solution_time_no_parachute), 'r-')

launch_information_file_no_parachute = open(f"ascent_test_1_trajectory_no_parachute.csv", 'w', newline="")
launch_information_writer = csv.writer(launch_information_file_no_parachute) # CSV writer for output file containing optimal trajectory launch information
launch_information_writer.writerow(["t", "long", "lat", "x", "y"]) # write header row of output CSV file containing optimal trajectory launch information
for time_step in solution_time_no_parachute:
    launch_information_writer.writerow([time_step, test_flight_no_parachute.longitude(time_step), test_flight_no_parachute.latitude(time_step), test_flight_no_parachute.x(time_step), test_flight_no_parachute.y(time_step)])
launch_information_file_no_parachute.close()

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
    environment=setup.launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

solution_time_parachute = [solution_step[0] for solution_step in test_flight_parachute.solution] # [s] time array of solution

setup.launch_area_ax.plot(test_flight_parachute.longitude(solution_time_parachute), test_flight_parachute.latitude(solution_time_parachute), 'b-')

launch_information_file_parachute = open(f"ascent_test_1_trajectory_parachute.csv", 'w', newline="")
launch_information_writer = csv.writer(launch_information_file_parachute) # CSV writer for output file containing optimal trajectory launch information
launch_information_writer.writerow(["t", "long", "lat", "x", "y"]) # write header row of output CSV file containing optimal trajectory launch information
for time_step in solution_time_parachute:
    launch_information_writer.writerow([time_step, test_flight_parachute.longitude(time_step), test_flight_parachute.latitude(time_step), test_flight_parachute.x(time_step), test_flight_parachute.y(time_step)])
launch_information_file_parachute.close()

print("\n---------- FLIGHT INFORMATION ----------")
print(f"Inclination: {round(launch_inclination, 2)} deg")
print(f"Heading: {round(launch_heading, 2)} deg")
print(f"Apogee Time: {round(test_flight_parachute.apogee_time, 2)} [s]")
print(f"Apogee Altitude: {round(test_flight_parachute.apogee, 2)} [m]")
print("----------------------------------------\n")

plt.show()