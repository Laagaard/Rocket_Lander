# Libraries
import math
from rocketpy import SolidMotor

# AeroTech G25W Motor Characteristics
propellant_length_G25W=0.09235 # [m] length of propellant grain
propellant_OD_G25W=0.02355 # [m] outer diameter of propellant grain
propellant_ID_G25W=0.00608 # [m] diameter of propellant core
propellant_mass_G25W=62.5/1000 # [kg] mass of propellant

nozzle_length=0.0224 # [m] nozzle length (measured from exit plane to plane abutting propellant grain)

CG_position_dry=0.07309 # [m] positiion of motor CG without propellant (relative to nozzle exit plane)

# AeroTech G79W Motor Characteristics
propellant_mass_G79W = 59/1000 # [kg] mass of propellant

# AeroTech G77R Motor Characteristics
propellant_mass_G77R = 55/1000 # [kg] mass of propellant

# AeroTech G138T Motor Characteristics
propellant_mass_G138T = 70/1000 # [kg] mass of propellant

directory_levels_to_try = 100 # arbitrarily set (just make it high enough)
thrust_source_file_path_prefix = ""
for ctr in range(directory_levels_to_try):
    try:
        # Set Path to the Thrust Curve Sources
        thrust_source_path_G25W = thrust_source_file_path_prefix + "DART_AeroTechG25W_thrustcurve.csv"
        thrust_source_path_G77R = thrust_source_file_path_prefix + "AeroTechG77R_thrustcurve.csv"
        thrust_source_path_G79W = thrust_source_file_path_prefix + "AeroTechG79W_thrustcurve.csv"
        thrust_source_path_G138T = thrust_source_file_path_prefix + "AeroTechG138T_thrustcurve.csv"

        # Construct AeroTechG25W Solid Rocket Motor
        AeroTechG25W = SolidMotor(
            thrust_source=thrust_source_path_G25W, # [s, N]
            dry_mass=102.72/1000, # [kg]
            dry_inertia=(235307.21*(1000**(-3)), 235307.21*(1000**(-3)), 13414.14*(1000**(-3))), # [kg*m^2] motor's dry mass inertia tensor components (e_3 = rocket symmetry axis)
            nozzle_radius=7.70/2/1000, # [m] nozzle exit radius
            grain_number=1, # [unitless]
            grain_density=propellant_mass_G25W/(propellant_length_G25W*(math.pi*(propellant_OD_G25W/2)**2 - math.pi*(propellant_ID_G25W/2)**2)), # [kg/m^2]
            grain_outer_radius=propellant_OD_G25W/2, # [m]
            grain_initial_inner_radius=propellant_ID_G25W/2, # [m]
            grain_initial_height=propellant_length_G25W, # [m]
            grain_separation=0, # [m] all one propellant grain
            grains_center_of_mass_position=nozzle_length + (propellant_length_G25W/2), # [m]
            center_of_dry_mass_position=CG_position_dry, # [m]
            nozzle_position=0.0, # [m] position of nozzle exit area, relative to motor coordinate system origin
            burn_time=None, # [s] derived from `thrust_source`
            throat_radius=3.56/2/1000, # [m] radius of nozzle throat
            reshape_thrust_curve=False,
            interpolation_method="linear",
            coordinate_system_orientation="nozzle_to_combustion_chamber" # direction of positive coordinate system axis
        )

        # Construct AeroTechG79W Solid Rocket Motor
        AeroTechG79W = SolidMotor(
            thrust_source=thrust_source_path_G79W, # [s, N]
            dry_mass=97/1000, # [kg]
            dry_inertia=(235307.21*(1000**(-3)), 235307.21*(1000**(-3)), 13414.14*(1000**(-3))), # [kg*m^2] motor's dry mass inertia tensor components (e_3 = rocket symmetry axis)
            nozzle_radius=7.70/2/1000, # [m] nozzle exit radius
            grain_number=1, # [unitless] (# TODO, G79 has two grains)
            grain_density=propellant_mass_G79W/(propellant_length_G25W*(math.pi*(propellant_OD_G25W/2)**2 - math.pi*(propellant_ID_G25W/2)**2)), # [kg/m^2]
            grain_outer_radius=propellant_OD_G25W/2, # [m]
            grain_initial_inner_radius=propellant_ID_G25W/2, # [m]
            grain_initial_height=propellant_length_G25W, # [m]
            grain_separation=0, # [m] all one propellant grain
            grains_center_of_mass_position=nozzle_length + (propellant_length_G25W/2), # [m]
            center_of_dry_mass_position=CG_position_dry, # [m]
            nozzle_position=0.0, # [m] position of nozzle exit area, relative to motor coordinate system origin
            burn_time=None, # [s] derived from `thrust_source`
            throat_radius=3.56/2/1000, # [m] radius of nozzle throat
            reshape_thrust_curve=False,
            interpolation_method="linear",
            coordinate_system_orientation="nozzle_to_combustion_chamber" # direction of positive coordinate system axis
        )

        # Construct AeroTechG77R Solid Rocket Motor
        AeroTechG77R = SolidMotor(
            thrust_source=thrust_source_path_G77R, # [s, N]
            dry_mass=101.72/1000, # [kg]
            dry_inertia=(235307.21*(1000**(-3)), 235307.21*(1000**(-3)), 13414.14*(1000**(-3))), # [kg*m^2] motor's dry mass inertia tensor components (e_3 = rocket symmetry axis)
            nozzle_radius=7.70/2/1000, # [m] nozzle exit radius
            grain_number=1, # [unitless]
            grain_density=propellant_mass_G77R/(propellant_length_G25W*(math.pi*(propellant_OD_G25W/2)**2 - math.pi*(propellant_ID_G25W/2)**2)), # [kg/m^2]
            grain_outer_radius=propellant_OD_G25W/2, # [m]
            grain_initial_inner_radius=propellant_ID_G25W/2, # [m]
            grain_initial_height=propellant_length_G25W, # [m]
            grain_separation=0, # [m] all one propellant grain
            grains_center_of_mass_position=nozzle_length + (propellant_length_G25W/2), # [m]
            center_of_dry_mass_position=CG_position_dry, # [m]
            nozzle_position=0.0, # [m] position of nozzle exit area, relative to motor coordinate system origin
            burn_time=None, # [s] derived from `thrust_source`
            throat_radius=3.56/2/1000, # [m] radius of nozzle throat
            reshape_thrust_curve=False,
            interpolation_method="linear",
            coordinate_system_orientation="nozzle_to_combustion_chamber" # direction of positive coordinate system axis
        )
    
        # Construct AeroTechG138T Solid Rocket Motor
        AeroTechG138T = SolidMotor(
            thrust_source=thrust_source_path_G138T, # [s, N]
            dry_mass=101.72/1000, # [kg]
            dry_inertia=(235307.21*(1000**(-3)), 235307.21*(1000**(-3)), 13414.14*(1000**(-3))), # [kg*m^2] motor's dry mass inertia tensor components (e_3 = rocket symmetry axis)
            nozzle_radius=7.70/2/1000, # [m] nozzle exit radius
            grain_number=1, # [unitless]
            grain_density=propellant_mass_G138T/(propellant_length_G25W*(math.pi*(propellant_OD_G25W/2)**2 - math.pi*(propellant_ID_G25W/2)**2)), # [kg/m^2]
            grain_outer_radius=propellant_OD_G25W/2, # [m]
            grain_initial_inner_radius=propellant_ID_G25W/2, # [m]
            grain_initial_height=propellant_length_G25W, # [m]
            grain_separation=0, # [m] all one propellant grain
            grains_center_of_mass_position=nozzle_length + (propellant_length_G25W/2), # [m]
            center_of_dry_mass_position=CG_position_dry, # [m]
            nozzle_position=0.0, # [m] position of nozzle exit area, relative to motor coordinate system origin
            burn_time=None, # [s] derived from `thrust_source`
            throat_radius=3.56/2/1000, # [m] radius of nozzle throat
            reshape_thrust_curve=False,
            interpolation_method="linear",
            coordinate_system_orientation="nozzle_to_combustion_chamber" # direction of positive coordinate system axis
        )
    except (ValueError): # ValueError raised when the CSV file isn't found
        thrust_source_file_path_prefix += "../"
    else:
        break