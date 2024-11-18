# Libraries
from rocketpy import SolidMotor
import math

propellant_length=0.09235 # [m] length of propellant grain
propellant_OD=0.02355 # [m] outer diameter of propellant grain
propellant_ID=0.00608 # [m] diameter of propellant core
propellant_mass=62.5/1000 # [kg] mass of propellant

nozzle_length=0.0224 # [m] nozzle length (measured from exit plane to plane abutting propellant grain)

CG_position_dry=0.07309 # [m] positiion of motor CG without propellant (relative to mozzle exit plane)

ascent_motor = SolidMotor(
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
    coordinate_system_orientation="nozzle_to_combustion_chamber"
)