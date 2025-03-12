# Python Libraries
import math
import matplotlib.pyplot as plt
import pandas as pd
from rocketpy import Flight
import sys
sys.path.append("../../")
# DART Modules
import setup

launch_information_df = pd.read_csv(f"{setup.date_dir_date_only}/{setup.date_string_date_only}.csv") # read CSV of launch parameters into pandas df
launch_information_df_filtered = launch_information_df[launch_information_df["Time"] == int(setup.date_string_time_only)] # filter df based on launch time

optimal_inclination = launch_information_df_filtered["Inclination"][0] # [deg] optimal launch inclination
optimal_heading = launch_information_df_filtered["Heading"][0] # [deg] optimal launch inclination

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

if __name__ == "__main__":
    for elem in range(2):
        if (elem == 0): # perform trajectory simulation without parachute first
            trajectory_color = 'r'
            export_file_base_name = "ascent_test_2_trajectory_no_parachute"
        elif (elem == 1): # perform trajectory simulation with parachute second
            trajectory_color = 'b'
            export_file_base_name = "ascent_test_2_trajectory_parachute"

        test_flight = Flight(
            rocket=setup.DART_rocket_1,
            environment=setup.launch_site,
            rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
            inclination=optimal_inclination, # [deg] rail inclination relative to the ground
            heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
            time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
        ) # run trajectory simulation

        solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

        test_flight.export_data(export_file_base_name + ".csv", "longitude", "latitude", "x", "y", "altitude", "e0", "e1", "e2", "e3")
        test_flight.export_kml(file_name=export_file_base_name + ".kml", extrude=True, altitude_mode="relativetoground")
        setup.launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), color=trajectory_color)

    plt.tight_layout()
    plt.savefig(f"ascent_test_2_trajectories.png", dpi=1000) # save the figure
    plt.show()