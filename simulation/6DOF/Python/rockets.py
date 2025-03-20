# Libraries
import copy
import math
from rocketpy import GenericSurface, NoseCone, Rocket, TrapezoidalFins
import sys
# DART Modules
import launch_site
import motors

FILE_NAME = sys.argv[0][2:] # [str] name of the module executed on the command line

# Rocket 1 Characteristics
total_mass_rocket_1 = 1.29658 # [kg] total wet mass of rocket (launch configuration)

# Construct Rocket 1 (for ascent and unpowered descent flight phases)
DART_rocket_1 = Rocket(
    radius=80.73/1000, # [m] largest outer radius
    mass=1.29658, # [kg] dry mass of the rocket (# TODO, corroborate with true value once Rylen updates the Excel sheet)
    inertia=(0.046172709, 0.046138602, 0.001703419, -1.53675E-05, -2.59173E-05, 3.91814E-05), # [kg*m^2] rocket inertia tensor components (e_3 = rocket symmetry axis)
    power_off_drag=0.456, # [unitless] C_D without motor firing
    power_on_drag=0.456, # [unitless] C_D with motor firing
    center_of_mass_without_motor=0, # [m] position of the rocket CG w/o motors relative to the rocket's coordinate system
    coordinate_system_orientation="tail_to_nose" # direction of positive coordinate system axis
)

def lift_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 1 Lift Coefficient (Ascent & Unpowered Descent Configuration)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Lift coefficient of the rocket.
    '''
    C_L = -7.527*(alpha**3) + 0.658*(alpha**2) + 12.170*alpha - 0.091 # [unitless] R**2 = 0.999
    return C_L

def drag_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 1 Drag Coefficient (Ascent & Unpowered Descent Configuration)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Drag coefficient of the rocket.
    '''
    C_D = 0.460*(alpha**5) - 4.510*(alpha**4) + 1.113*(alpha**3) + 11.685*(alpha**2) - 0.098*alpha + 0.456 # [unitless] R**2 = 1.000
    return C_D

def pitch_moment_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 1 Pitching Moment Coefficient (Ascent & Unpowered Descent Configuration)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Pitching moment coefficient of the rocket.
    '''
    C_M = 15.255*(alpha**3) - 2.519*(alpha**2) - 16.606*alpha - 0.208 # [unitless] R**2 = 0.995
    return C_M

# `GenericSurface` to Define the Aerodynamics of Rocket 1
DART_rocket_1_aero_surface = GenericSurface(
    reference_area=DART_rocket_1.area, # [m^2] reference area of the aerodynamic surface
    reference_length=2*DART_rocket_1.radius, # [m] reference length of the aerodynamic surface
    coefficients={
        "cL": lift_coefficient_rocket_1,
        "cD": drag_coefficient_rocket_1,
        "cm": pitch_moment_coefficient_rocket_1
    },
    center_of_pressure=(0,0,0),
    name="Rocket 1 Generic Surface"
)

# Construct Rocket 2 (for powered descent phase w/ legs stowed)
DART_rocket_2 = copy.deepcopy(x=DART_rocket_1) # create an independent copy (changes to the copy do not affect the original)

def lift_coefficient_rocket_2(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 2 Lift Coefficient (Powered Descent Configuration w/ Legs Stowed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Lift coefficient of the rocket.
    '''
    C_L = -7.039*(alpha**3) + 0.862*(alpha**2) + 10.378*alpha - 0.022 # [unitless] R**2 = 0.999
    return C_L

def drag_coefficient_rocket_2(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 2 Drag Coefficient (Powered Descent Configuration w/ Legs Stowed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Drag coefficient of the rocket.
    '''
    C_D = -1.287*(alpha**5) - 7.126*(alpha**4) + 2.149*(alpha**3) + 12.712*(alpha**2) - 0.138*alpha + 1.073 # [unitless] R**2 = 0.997
    return C_D

def pitch_moment_coefficient_rocket_2(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 2 Pitching Moment Coefficient (Powered Descent Configuration w/ Legs Stowed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Pitching moment coefficient of the rocket.
    '''
    C_M = 17.701*(alpha**3) - 2.495*(alpha**2) - 20.942*alpha + 0.157 # [unitless] R**2 = 0.995
    return C_M

# `GenericSurface` to Define the Aerodynamics of Rocket 2
DART_rocket_2_aero_surface = GenericSurface(
    reference_area=DART_rocket_2.area, # [m^2] reference area of the aerodynamic surface
    reference_length=2*DART_rocket_2.radius, # [m] reference length of the aerodynamic surface
    coefficients={
        "cL": lift_coefficient_rocket_2,
        "cD": drag_coefficient_rocket_2,
        "cm": pitch_moment_coefficient_rocket_2
    },
    center_of_pressure=(0,0,0),
    name="Rocket 2 Generic Surface"
)

# Construct Rocket 3 (for powered descent phase w/ legs deployed)
DART_rocket_3 = copy.deepcopy(x=DART_rocket_1) # create an independent copy (changes to the copy do not affect the original)

def lift_coefficient_rocket_3(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 3 Lift Coefficient (Powered Descent Configuration w/ Legs Deployed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Lift coefficient of the rocket.
    '''
    C_L = -6.894*(alpha**3) + 0.604*(alpha**2) + 9.234*alpha - 0.033 # [unitless] R**2 = 0.997
    return C_L

def drag_coefficient_rocket_3(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 3 Drag Coefficient (Powered Descent Configuration w/ Legs Deployed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Drag coefficient of the rocket.
    '''
    C_D = -0.266*(alpha*5) - 6.975*(alpha**4) + 1.366*(alpha**3) + 11.572*(alpha**2) - 0.097*alpha + 2.181 # [unitless] R**2 = 0.996
    return C_D

def pitch_moment_coefficient_rocket_3(alpha, beta, Ma, Re, q, r, p):
    '''
    @brief Callable Function to Calculate Rocket 3 Pitching Moment Coefficient (Powered Descent Configuration w/ Legs Deployed)

    @param alpha: angle of attack [radians]
    @param beta: angle of side-slip [radians]
    @param Ma: Mach number
    @param Re: Reynolds number
    @param q: Pitch rate
    @param r: Yaw rate
    @param p: Roll rate

    @return Pitching moment coefficient of the rocket.
    '''
    C_M = 16.411*(alpha**3) - 4.037*(alpha**2) - 18.277*alpha + 0.184 # [unitless] R**2 = 0.993
    return C_M

# `GenericSurface` to Define the Aerodynamics of Rocket 3
DART_rocket_3_aero_surface = GenericSurface(
    reference_area=DART_rocket_3.area, # [m^2] reference area of the aerodynamic surface
    reference_length=2*DART_rocket_3.radius, # [m] reference length of the aerodynamic surface
    coefficients={
        "cL": lift_coefficient_rocket_3,
        "cD": drag_coefficient_rocket_3,
        "cm": pitch_moment_coefficient_rocket_3
    },
    center_of_pressure=(0,0,0),
    name="Rocket 3 Generic Surface"
)

airfoil_source_file_path_prefix = ""
for ctr in range(motors.directory_levels_to_try):
    try:
        # Set Path to the Fin Airfoil Geometry Source
        fin_airfoil_source_path = airfoil_source_file_path_prefix + "NACA0012.csv"

        # Construct Fins for Rocket 1
        DART_fins_rocket_1 = TrapezoidalFins(
            n=3, # [unitless] number of fins
            root_chord=0.125223, # [m]
            tip_chord=0.062611, # [m]
            span=0.08636, # [m]
            rocket_radius=DART_rocket_1.radius, # [m] reference radius to calculate lift coefficient
            cant_angle=0, # [deg] cant (i.e., tilt) angle of fins (non-zero will induce roll)
            airfoil=(fin_airfoil_source_path, "degrees") # [CSV of {alpha,C_L}, alpha provided in degrees]
        )

        # Construct Fins for Rocket 2
        DART_fins_rocket_2 = TrapezoidalFins(
            n=3, # [unitless] number of fins
            root_chord=0.125223, # [m]
            tip_chord=0.062611, # [m]
            span=0.08636, # [m]
            rocket_radius=DART_rocket_1.radius, # [m] reference radius to calculate lift coefficient
            cant_angle=0, # [deg] cant (i.e., tilt) angle of fins (non-zero will induce roll)
            # sweep_angle=-0.001, # [deg] fins sweep angle with respect to the rocket centerline
            airfoil=(fin_airfoil_source_path, "degrees") # [CSV of {alpha,C_L}, alpha provided in degrees]
        )
    except (ValueError): # ValueError raised when the CSV file isn't found
        airfoil_source_file_path_prefix += "../"
    else:
        break

'''
-------------------- Add Nose Cone --------------------
length: [m] length of the nose cone (excluding the shoulder)
kind: One of {Von Karman, conical, ogive, lvhaack, powerseries}
position: [m] Nose cone tip coordinate relative to the rocket's coordinate system
'''
DART_nose = NoseCone(
    length=0.145836, # [m]
    kind="ogive",
    base_radius=DART_rocket_1.radius, # [m] nose cone base radius
    bluffness=0.6/1.5 # ratio between the radius of the circle on the tip of the ogive and the radius of the base of the ogive (will truncate the length if less than 1)
)

'''
-------------------- Add Rail Buttons to Rocket 1 --------------------
upper_button_position: # [m] position of the rail button furthest from the nozzle relative to the rocket's coordinate system
lower_button_position: # [m] position of the rail button closest to the nozzle relative to the rocket's coordinate system
'''
DART_rocket_1_rail_buttons = DART_rocket_1.set_rail_buttons(upper_button_position=-0.027064, lower_button_position=-0.179464)

# Add Motor to Rocket 1
DART_rocket_1.add_motor(motors.AeroTechG79W, position=-0.366905) # postion: [m] Position of the motor's coordinate system origin relative to the user defined rocket coordinate system

# Apply the `GenericSurface` aerodynamics to Rocket 1
DART_rocket_1.add_surfaces(surfaces=DART_rocket_1_aero_surface, positions=(0,0,0))

# Apply the fins to Rocket 1 (position was altered from true value to make initial SM = 1.066 cal (as desired))
DART_rocket_1.add_surfaces(surfaces=DART_fins_rocket_1, positions=(0,0,-0.1532))

# Apply the parachute to Rocket 1 (per command line arguments or JSON file)
if (launch_site.parachute_flag):
    # Parachute Characteristics
    C_D = 0.84 # [unitless] parachute drag coefficient
    parachute_reference_area=math.pi*(30*0.0254/2)**2 # [m^2] reference area of parachute

    # Construct Parachute
    main_parachute = DART_rocket_1.add_parachute(
        name="main", # name of the parachute (no impact on simulation)
        cd_s=C_D*parachute_reference_area, # [m^2] drag coefficient times parachute reference area
        trigger="apogee",
        sampling_rate=10, # [Hz] sampling rate in which the trigger function works (used to simulate sensor refresh rates)
        lag=0, # [s] time between the ejection system is triggered and the parachute is fully opened (SHOULD BE QUANTIFIED WITH EJECTION TESTING)
        noise=(0,0,0) # [Pa] (mean, standard deviation, time-correlation) used to add noise to the pressure signal
    )

# Add Rail Buttons to Rocket 2 (# TODO, update positions with Excel sheet values)
DART_rocket_2_rail_buttons = DART_rocket_2.set_rail_buttons(upper_button_position=-0.027064, lower_button_position=-0.179464)

# TODO
# 1) Update position with Excel sheet value
# 2) Adjust the CP location to get correct initial SM (-1.598)
# Add Motor to Rocket 2
DART_rocket_2.add_motor(motors.AeroTechG25W, position=-0.366905) # postion: [m] Position of the motor's coordinate system origin relative to the user defined rocket coordinate system

# Apply the `GenericSurface` aerodynamics to Rocket 2
DART_rocket_2.add_surfaces(surfaces=DART_rocket_2_aero_surface, positions=(0,0,0))

# Apply the fins to Rocket 2 (position was altered from true value to make initial SM = 1.598 cal (as desired))
DART_rocket_2.add_surfaces(surfaces=DART_fins_rocket_2, positions=(0,0,-0.2393))

# Add Rail Buttons to Rocket 3 (# TODO, update positions with Excel sheet values)
DART_rocket_3_rail_buttons = DART_rocket_3.set_rail_buttons(upper_button_position=-0.027064, lower_button_position=-0.179464)

# TODO
# 1) Update position with Excel sheet value
# 2) Adjust the CP location to get correct initial SM (-1.345)
# Add Motor to Rocket 3
DART_rocket_3.add_motor(motors.AeroTechG25W, position=-0.366905) # postion: [m] Position of the motor's coordinate system origin relative to the user defined rocket coordinate system

# Apply the `GenericSurface` aerodynamics to Rocket 3
DART_rocket_3.add_surfaces(surfaces=DART_rocket_3_aero_surface, positions=(0,0,0))

# Apply the fins to Rocket 3 (position was altered from true value to make initial SM = -1.345 cal (as desired))
DART_rocket_3.add_surfaces(surfaces=DART_fins_rocket_2, positions=(0,0,0.2076))

# If one of the following modules was executed directly: rockets.py, setup.py
if (FILE_NAME == "rockets.py" or FILE_NAME == "setup.py"):
    # Print `DART_rocket_1` Information
    print("------------------------------------------------------")
    print("DART ROCKET 1 INFORMATION (Ascent & Unpowered Descent)")
    print("------------------------------------------------------", end="")
    DART_rocket_1.info()

    # Print `DART_rocket_2` Information
    print("----------------------------------------------------------")
    print("DART ROCKET 2 INFORMATION (Powered Descent w/ Legs Stowed)")
    print("----------------------------------------------------------", end="")
    DART_rocket_2.info()
    DART_rocket_2.draw()

    # Print `DART_rocket_3` Information
    print("------------------------------------------------------------")
    print("DART ROCKET 3 INFORMATION (Powered Descent w/ Legs Deployed)")
    print("------------------------------------------------------------", end="")
    DART_rocket_3.info()