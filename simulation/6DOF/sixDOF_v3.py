# Libraries
from rocketpy import Environment, SolidMotor, Rocket, Fins, Flight
import math

# Construct Launch Site Environment
launch_site = Environment(
    latitude=(27 + (55/60) + (58/3600)), # [deg] positive corresponds to North
    longitude=-(80 + (42/60) + (30/3600)), # [deg] positive corresponds to East
    elevation=4 # [m] launch site elevation above sea level
)

# AeroTech-G25W Motor Characteristics
propellant_length=0.09235 # [m] length of propellant grain
propellant_OD=0.02355 # [m] outer diameter of propellant grain
propellant_ID=0.00608 # [m] diameter of propellant core
propellant_mass=62.5/1000 # [kg] mass of propellant

nozzle_length=0.0224 # [m] nozzle length (measured from exit plane to plane abutting propellant grain)

CG_position_dry=0.07309 # [m] positiion of motor CG without propellant (relative to mozzle exit plane)

# Construct AeroTechG25W Solid Rocket Motor
AeroTechG25W = SolidMotor(
    thrust_source="AeroTechG25W_thrustcurve.csv", # [s, N]
    dry_mass=101.72/1000, # [kg]
    dry_inertia=None, # [NEEDS TO BE UDPATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    nozzle_radius=7.70/2/1000, # [m] nozzle exit radius
    grain_number=1, # [unitless]
    grain_density=propellant_mass/(propellant_length*(math.pi*(propellant_OD/2)**2 - math.pi*(propellant_ID/2)**2)), # [kg/m^2]
    grain_outer_radius=propellant_OD/2, # [m]
    grain_initial_inner_radius=propellant_ID/2, # [m]
    grain_initial_height=propellant_length, # [m]
    grain_separation=0, # [m] all one propellant grain
    grains_center_of_mass_position=nozzle_length + (propellant_length/2), # [m]
    center_of_dry_mass_position=CG_position_dry, # [m]
    nozzle_position=0.0, # [m] position of nozzle exit area, relative to motor coordinate system origin
    burn_time=None, # [s] derived from `thrust_source`
    throat_radius=3.56/2/1000, # [m] radius of nozzle throat
    reshape_thrust_curve=False,
    interpolation_method="linear",
    coordinate_system_orientation="nozzle_to_combustion_chamber" # direction of positive coordinate system axis
)

# Rocket Characteristics
allowable_mass = 1.5 # [kg] maximum allowable rocket mass per 14 CFR Part 101.22
motor_mass = 2*(AeroTechG25W.propellant_initial_mass + AeroTechG25W.dry_mass) # [kg] total mass of both motors

# Construct Rocket
DART_rocket = Rocket(
    radius=76.2/1000, # [m] largest outer radius
    mass=allowable_mass - motor_mass, # [kg] dry mass of the rocket
    inertia=None, # [NEEDS TO BE UPDATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    power_off_drag=1.6939, # [unitless] C_D without motor firing
    power_on_drag=1.6939, # [unitless] C_D with motor firing
    center_of_mass_without_motor=0, # [m] position of the rocket CG w/o motors relative to the rocket's coordinate system
    coordinate_system_orientation="tail_to_nose" # direction of positive coordinate system axis
)

'''
-------------------- Add Ascent Motor --------------------
postion: Position of the motor's coordinate system origin relative to the user defined rocket coordinate system
'''
DART_rocket.add_motor(AeroTechG25W, position=None) # [NEEDS TO BE UPDATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


'''
-------------------- Add Rail Buttons --------------------
upper_button_position: Position of the rail button furthest from the nozzle relative to the rocket's coordinate system
lower_button_position: Position of the rail button closest to the nozzle relative to the rocket's coordinate system
'''
DART_rocket.set_rail_buttons(upper_button_position=-0.3, lower_button_position=-0.5) # [ARBITRARILY CHOSEN AND NEEDS TO BE UPDATED]

'''
-------------------- Add Nose Cone --------------------
length: length of the nose cone (excluding the shoulder)
kind: One of {Von Karman, conical, ogive, lvhaack, powerseries}
position: Nose cone tip coordinate relative to the rocket's coordinate system
'''
DART_rocket.add_nose(length=0.14986, kind="ogive", position=0.459)

# Construct Fins
DART_fins = Fins(
    n=3, # [unitless] number of fins
    root_chord=None, # [NEEDS TO BE UPDATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    span=None, # [NEEDS TO BE UPDATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    rocket_radius=DART_rocket.radius, # [m] rocket reference radius used for lift coefficient normalization
    cant_angle=0, # [deg] cant (i.e., tilt) angle of fins (non-zero will induce roll)
    airfoil=None, # [NEEDS TO BE UPDATED] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
)

# Parachute Characteristics
C_D = 0.84 # [unitless] parachute drag coefficient
parachute_reference_area=math.pi*(0.762/2)**2 # [m^2] reference area of parachute

# Construct Parachute
main = DART_rocket.add_parachute(
    name="main", # name of the parachute (no impact on simulation)
    cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
    trigger="apogee", # will trigger the parachute deployment at apogee (can also use a callable function based on fresstream pressure, altitude, and state vector)
    sampling_rate=100, # [Hz] sampling rate in which the parachute trigger will be checked at (CHECK WITH JACK OHARA FOR SENSOR REFRESH RATES) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    lag=0, # [s] time between the ejection system is triggers and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
    noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlatio) used to add noise to the pressure signal
)

flight_test = Flight(
    rocket=DART_rocket,
    environment=launch_site,
    rail_length=1.5, # [m] length of the launch rail (NEED TO DOUBLE CHECK UNITS)
    inclination=89, # [deg] rail inclination relative to the ground
    heading=0, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)