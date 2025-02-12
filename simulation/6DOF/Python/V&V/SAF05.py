# Libraries
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import DART_rocket, launch_site, launch_rail_length
from SYS10 import C_D, parachute_reference_area

SAF05_velocity_threshold = -8 # [m/s] maximum descent velocity under parachute permitted by requirement SAF.05

launch_inclination = 85 # [deg] launch inclination
launch_heading = 30 # [deg] launch heading, measured CW from North

# Construct Parachute
main_parachute = DART_rocket.add_parachute(
    name="main", # name of the parachute (no impact on simulation)
    cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
    trigger="apogee", # "apogee" acceptable for SAF.05 verification, but must be trigger function for SYS.10
    sampling_rate=10, # [Hz] sampling rate in which the trigger function works (used to simulate sensor refresh rates)
    lag=0, # [s] time between the ejection system is triggered and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
    noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlation) used to add noise to the pressure signal
)

test_flight = Flight(
    rocket=DART_rocket,
    environment=launch_site,
    rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=launch_inclination, # [deg] rail inclination relative to the ground
    heading=launch_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=False # if True, decouples ODE time step from parachute trigger functions sampling rate
) # run trajectory simulation

solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

fig = plt.figure()
descent_velocity_ax = fig.add_subplot()
descent_velocity_ax.grid(which="both", axis="both")
descent_velocity_ax.plot(solution_time, test_flight.vz(solution_time), 'k-')
descent_velocity_ax.axhline(y=SAF05_velocity_threshold, color='r', linestyle='--')

trans = transforms.blended_transform_factory(descent_velocity_ax.get_yticklabels()[0].get_transform(), descent_velocity_ax.transData)
descent_velocity_ax.text(0, SAF05_velocity_threshold, s=f"{SAF05_velocity_threshold}", color="red", transform=trans, ha="right", va="center")

descent_velocity_ax.set_xlabel("T+0 [s]")
descent_velocity_ax.set_ylabel("Vertical Velocity [m/s]")
descent_velocity_ax.set_title("SAF.05 Verification")

print("\n----------------------------------------------------")
print(f"Ground-Impact Velocity: {round(test_flight.vz(solution_time[-1]), 2)} [m/s]")
print("----------------------------------------------------\n")

plt.tight_layout()
plt.savefig(f"SAF05_Verification.png", transparent=True, dpi=1000) # save the figure with a transparent background
plt.show()