# Libraries
import math
from rocketpy import SolidMotor
import sys

FILE_NAME = sys.argv[0][2:] # [str] name of the module executed on the command line

# If one of the following modules was executed directly: motors.py, rockets.py, setup.py
if (FILE_NAME == "motors.py" or FILE_NAME == "rockets.py" or FILE_NAME == "setup.py"):
    # Set Path to the Thrust Curve Source
    thrust_source_path = "../../../DART_AeroTechG25W_thrustcurve.csv" # TODO, not a robust solution (only works from a directory one level higher)
    # Set Path to the Fin Airfoil Geometry Source
    fin_airfoil_source_path = "NACA0012.csv"
else:
    thrust_source_path = "../../../../DART_AeroTechG25W_thrustcurve.csv"
    fin_airfoil_source_path = "../NACA0012.csv" # same TBR as above

# AeroTech G25W Motor Characteristics
propellant_length=0.09235 # [m] length of propellant grain
propellant_OD=0.02355 # [m] outer diameter of propellant grain
propellant_ID=0.00608 # [m] diameter of propellant core
propellant_mass=62.5/1000 # [kg] mass of propellant

nozzle_length=0.0224 # [m] nozzle length (measured from exit plane to plane abutting propellant grain)

CG_position_dry=0.07309 # [m] positiion of motor CG without propellant (relative to nozzle exit plane)

# Construct AeroTechG25W Solid Rocket Motor
AeroTechG25W = SolidMotor(
    thrust_source=thrust_source_path, # [s, N]
    dry_mass=101.72/1000, # [kg]
    dry_inertia=(235307.21*(1000**(-3)), 235307.21*(1000**(-3)), 13414.14*(1000**(-3))), # [kg*m^2] motor's dry mass inertia tensor components (e_3 = rocket symmetry axis)
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