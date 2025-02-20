# Libraries
import contextily as cx
import copy
import datetime
import geopandas as gpd
import math
import numpy as np
import os
import pandas as pd
import re
from rocketpy import Environment, GenericSurface, NoseCone, prints, Rocket, SolidMotor, TrapezoidalFins
import stat
import sys
import warnings

# Function for `shutil.rmtree` to call on "Access is denied" error from read-only folder
def remove_readonly(func, path, excinfo):
    os.chmod(path, stat.S_IWRITE)
    func(path)

results_dir = "Results"

if os.path.exists(results_dir): # If the results directory already exists
    None
else:
    os.mkdir(results_dir) # Create folder for all results for the given date/time

command_line_args = {
  "date": "",
  "time": "",
  "location": "independent", # default location corresponds to launching independently in the Compound (at the SRA launch location)
  "automated": False # default value indicates the script is being run manually
} # dictionary of recognized command line arguments

for idx in range(1, len(sys.argv), 2): # iterate over command line arguments
    try:
        command_line_regex = re.match("--[A-Za-z]+", sys.argv[idx]).group(0)[2:] # attempt to match the argument key to the format "--<KEY>""
    except (AttributeError): # AttributeError thrown when the regex fails to match
        warnings.warn(f"\"{sys.argv[idx]}\" is not a valid command line argument key")
        continue # continue to the next command line argument (will leave the dictionary entry untouched)
    if (command_line_regex in command_line_args): # if regex-compliant argument matches one of the keys in the dictionary
        command_line_args[command_line_regex] = sys.argv[idx + 1] # assign the corresponding command line arg to the dictionary value

# Establish Launch Date and Time (EST)
if (command_line_args["date"] != "" and command_line_args["time"] != ""): # if the launch date and time were passed as command line arguments
    launch_date = datetime.datetime.strptime(command_line_args["date"], "%m-%d-%Y") # launch date
    launch_hour = int(command_line_args["time"])
    launch_time = datetime.time(hour=launch_hour, minute=00) # # launch time (hr, min) (input as EST)
    launch_date_and_time = datetime.datetime.combine(launch_date, launch_time) # launch date and time
else: # the launch date and time were NOT passed as command line arguments
    launch_date_and_time = datetime.datetime.now() # launch date and time

automation_flag = command_line_args["automated"] # flag to signal whether or not the program is being executed by an automatic runner (`True` if it is)

'''
Establish Launch Site Latitude & Longitude
Independent: https://www.google.com/maps/@27.933873,-80.7094486,55m/data=!3m1!1e3?entry=ttu&g_ep=EgoyMDI1MDEwOC4wIKXMDSoASAFQAw%3D%3D
ROAR, NAR section 795: https://www.google.com/maps/@28.5633031,-81.0187189,261m/data=!3m1!1e3?authuser=1&hl=en&entry=ttu&g_ep=EgoyMDI1MDEyOS4xIKXMDSoASAFQAw%3D%3D
'''
launch_site_latitude_independent = 27.933880 # [deg] North, launch site latitude (if launching independently (i.e., without a NAR section))
launch_site_longitude_independent = -80.709505 # [deg] West, launch site longitude (if launching independently (i.e., without a NAR section))
launch_site_latitude_ROAR = 28.563321 # [deg] North, launch site latitude (if launching with ROAR, NAR section 795))
launch_site_longitude_ROAR = -81.018022 # [deg] West, launch site longitude (if launching with ROAR, NAR section 795))

# Set the Launch Site Location
match (command_line_args["location"]):
    case ("independent"): # launching independently (i.e., without a NAR section) at the SRA launch site in the Compound
        launch_site_latitude = launch_site_latitude_independent # [deg]
        launch_site_longitude = launch_site_longitude_independent # [deg]
    case ("ROAR"): # launching with Regional Orlando Applied Rocketry (NAR Section 795, https://www.flroar.space/)
        launch_site_latitude = launch_site_latitude_ROAR # [deg]
        launch_site_longitude = launch_site_longitude_ROAR # [deg]

# Construct Launch Site Environment
launch_site = Environment(
    date=launch_date_and_time, # launch date and time
    latitude=launch_site_latitude, # [deg] positive corresponds to North
    longitude=launch_site_longitude, # [deg] positive corresponds to East
    elevation=4, # [m] launch site elevation above sea level
    timezone="EST", # specify launch site time zone (# TODO, see if this fixed the time issues with the automatic runner)
    max_expected_height=250 # [m] maximum altitude to keep weather data (must be above sea level)
)

# Landing Zone Coordinates (if launching independently)
landing_zone_lats_independent = [27.935514, 27.935504, 27.935499, 27.934703, 27.934706, 27.933877, 27.933002, 27.932312, 27.932334, 27.932334, 27.932334] # [deg] latitude coordinates of landing zone centers (clockwise around launch site)
landing_zone_longs_independent = [-80.711389, -80.710455, -80.709524, -80.709506, -80.708361, -80.708351, -80.708350, -80.708283, -80.709533, -80.710461, -80.711391] # [deg] longitude coordinates of landing zone centers (clockwise around launch site)

# Landing Zone Coordinates (if launching with ROAR, NAR section 795)
landing_zone_lats_ROAR = [launch_site.latitude + 0.001,         launch_site.latitude + 0.001,       launch_site.latitude + 0.001,       launch_site.latitude + 0.001,       launch_site.latitude + 0.001,
                          launch_site.latitude + (2*0.001/3),   launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3),
                          launch_site.latitude + 0.001/3,       launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3,
                          launch_site.latitude,                 launch_site.latitude,                                                   launch_site.latitude,               launch_site.latitude,
                          launch_site.latitude - 0.001/3,       launch_site.latitude - 0.001/3,     launch_site.latitude - 0.001/3,     launch_site.latitude - 0.001/3,     launch_site.latitude - 0.001/3,
                          launch_site.latitude - (2*0.001/3),   launch_site.latitude - (2*0.001/3), launch_site.latitude - (2*0.001/3), launch_site.latitude - (2*0.001/3), launch_site.latitude - (2*0.001/3),
                          launch_site.latitude - 0.001,         launch_site.latitude - 0.001,       launch_site.latitude - 0.001,       launch_site.latitude - 0.001,       launch_site.latitude - 0.001] # [deg] latitude coordinates of landing zone centers
landing_zone_longs_ROAR = [launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,                                        launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001] # [deg] longitude coordinates of landing zone centers

# Set Landing Zone Coordinates
match (command_line_args["location"]):
    case ("independent"): # launching independently (i.e., without a NAR section) at the SRA launch site in the Compound
        landing_zone_lats = landing_zone_lats_independent # [deg]
        landing_zone_longs = landing_zone_longs_independent # [deg]
    case ("ROAR"): # launching with Regional Orlando Applied Rocketry (NAR Section 795, https://www.flroar.space/)
        landing_zone_lats = landing_zone_lats_ROAR # [deg]
        landing_zone_longs = landing_zone_longs_ROAR # [deg]

# String Formatting Based on Launch Date and Time (for organization of results)
date_format_string_date_only = "%m-%d-%Y"
date_string_date_only = launch_site.local_date.strftime(date_format_string_date_only)
date_dir_date_only = f"{results_dir}/{date_string_date_only}"
date_format_string_time_only = "%H"
date_string_time_only = launch_site.local_date.strftime(date_format_string_time_only)
date_dir_with_time = f"{date_dir_date_only}/{date_string_time_only}"
date_string_with_time = f"{date_string_date_only}-{date_string_time_only}"

'''
-------------------- Add Forecast (i.e., Wind) Information --------------------
Ensemble, GEFS: 1-deg geographical resolution, updated every 6 hours (00, 06, 12, 18UTC) (experimentally determined to have the same forecast depth as GFS)
Forecast, GFS: 0.25-deg geographical resolution, updated every 6 hours (good balance)
Forecast, RAP: 0.19-deg geographical resolution, updated hourly (best temporal resolution and update frequency)
Forecast, NAM: ~0.045-deg geographical resolution, updated every 6 hours with points spaced every 3 hours (best geographical resolution) (https://www.ncei.noaa.gov/products/weather-climate-models/north-american-mesoscale)
'''
try:
    launch_site.set_atmospheric_model(type="Forecast", file="RAP") # RAP updates hourly
except (ValueError): # ValueError thrown when "Chosen launch time is not available in the provided file"
    try:
        launch_site.set_atmospheric_model(type="Forecast", file="NAM") # NAM has 3-hour point spacing and updates every 6 hours
    except (ValueError): # same ValueError as above
        try:
            launch_site.set_atmospheric_model(type="Forecast", file="GFS") # GFS updates every 6 hours
        except (ValueError): # same ValueError as above
            try:
                launch_site.set_atmospheric_model(type="Ensemble", file="GEFS") # GEFS experimentally determined to have same forecast depth as GFS, but can't hurt to include just in case
            except (ValueError): # same ValueError as above
                launch_site.set_atmospheric_model(type="standard_atmosphere") # will default to the ISA (no wind)
                print("Weather Model: ISA (NO WIND)")
            else:
                print("Weather Model: GEFS")
        else:
            print("Weather Model: GFS")
    else:
        print("Weather Model: NAM")
else:
    print("Weather Model: RAP")

# Run if the script is executed directly (i.e., not called from another file)
if __name__ == "__main__":
    # Print information of launch site conditions (if not using ISA)
    if (launch_site.atmospheric_model_type != "standard_atmosphere"):
        launch_site_prints = prints.environment_prints._EnvironmentPrints(launch_site)
        launch_site_prints.all()
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

# Rocket Characteristics
total_mass = 1434.96/1000 # [kg] total wet mass of rocket (launch configuration) (# TODO, update with final CAD assembly)
motor_mass = AeroTechG25W.propellant_initial_mass + AeroTechG25W.dry_mass # [kg] total mass of ONE motor

# Construct Rocket 1 (for ascent and unpowered descent flight phases)
DART_rocket_1 = Rocket(
    radius=(3.28/2*25.4)/1000, # [m] largest outer radius
    mass=total_mass - motor_mass, # [kg] dry mass of the rocket
    inertia=(46065894.35*(1000**(-3)), 46057059.28*(1000**(-3)), 1796848.98*(1000**(-3)),
             4106.46*(1000**(-3)), 53311.54*(1000**(-3)), 60557.58*(1000**(-3))), # [kg*m^2] rocket inertia tensor components (e_3 = rocket symmetry axis) (# TODO, update with final CAD assembly)
    power_off_drag=1.6939, # [unitless] C_D without motor firing
    power_on_drag=1.6939, # [unitless] C_D with motor firing
    center_of_mass_without_motor=0, # [m] position of the rocket CG w/o motors relative to the rocket's coordinate system
    coordinate_system_orientation="tail_to_nose" # direction of positive coordinate system axis
)

# Add Motor to Rocket 1
DART_rocket_1.add_motor(AeroTechG25W, position=-0.370869) # postion: [m] Position of the motor's coordinate system origin relative to the user defined rocket coordinate system

def lift_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    Callable Function to Calculate Rocket 1 Lift Coefficient (Ascent & Unpowered Descent Configuration)
    - Lift coefficient equation pulled from CDR slide 46

    Parameters
    ----------
    `alpha`: angle of attack [radians]
    `beta`: angle of side-slip [radians]
    `Ma`: Mach number
    `Re`: Reynolds number
    `q`: Pitch rate
    `r`: Yaw rate
    `p`: Roll rate

    Returns
    -------
    `C_L`: Lift coefficient [unitless]
    '''
    C_L = -5.395*(alpha**3) + 0.267*(alpha**2) + 12.082*alpha + 0.136 # [unitless] R**2 = 0.999
    return C_L

def drag_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    Callable Function to Calculate Rocket 1 Drag Coefficient (Ascent & Unpowered Descent Configuration)
    - Drag coefficient equation pulled from CDR slide 46

    Parameters
    ----------
    `alpha`: angle of attack [radians]
    `beta`: angle of side-slip [radians]
    `Ma`: Mach number
    `Re`: Reynolds number
    `q`: Pitch rate
    `r`: Yaw rate
    `p`: Roll rate

    Returns
    -------
    `C_D`: Drag coefficient [unitless]
    '''
    C_D = 10.560*(alpha**2) + 0.523*alpha + 0.507 # [unitless] R**2 = 0.997
    return C_D

def pitch_moment_coefficient_rocket_1(alpha, beta, Ma, Re, q, r, p):
    '''
    Callable Function to Calculate Rocket 1 Pitching Moment Coefficient (Ascent & Unpowered Descent Configuration)

    Parameters
    ----------
    `alpha`: angle of attack [radians]
    `beta`: angle of side-slip [radians]
    `Ma`: Mach number
    `Re`: Reynolds number
    `q`: Pitch rate
    `r`: Yaw rate
    `p`: Roll rate

    Returns 
    -------
    `C_M`: Pitching moment coefficient [unitless]
    '''
    C_M = 3.520*(alpha**3) - 1.546*(alpha**2) - 16.508*alpha - 0.901 # [unitless] R**2 = 0.997
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

# TODO Make a `GenericSurface` to define the aerodynamics of Rocket 2

# Construct Fins for Rocket 1
DART_fins_rocket_1 = TrapezoidalFins(
    n=3, # [unitless] number of fins
    root_chord=0.125223, # [m]
    tip_chord=0.062611, # [m]
    span=0.0848, # [m] v1: 0.08636
    rocket_radius=DART_rocket_1.radius, # [m] reference radius to calculate lift coefficient
    cant_angle=0, # [deg] cant (i.e., tilt) angle of fins (non-zero will induce roll)
    airfoil=(fin_airfoil_source_path, "degrees"), # [CSV of {alpha,C_L}, alpha provided in degrees]
) # TODO, chord and span lengths need to be checked against CAD

# Construct Fins for Rocket 2
DART_fins_rocket_2 = TrapezoidalFins(
    n=3, # [unitless] number of fins
    root_chord=0.125223, # [m]
    tip_chord=0.062611, # [m]
    span=0.0848, # [m] v1: 0.08636
    rocket_radius=DART_rocket_1.radius, # [m] reference radius to calculate lift coefficient
    cant_angle=0, # [deg] cant (i.e., tilt) angle of fins (non-zero will induce roll)
    sweep_angle=-0.001, # [deg] fins sweep angle with respect to the rocket centerline
    airfoil=(fin_airfoil_source_path, "degrees"), # [CSV of {alpha,C_L}, alpha provided in degrees]
) # TODO, chord and span lengths need to be checked against CAD

'''
-------------------- Add Nose Cone --------------------
length: [m] length of the nose cone (excluding the shoulder)
kind: One of {Von Karman, conical, ogive, lvhaack, powerseries}
position: [m] Nose cone tip coordinate relative to the rocket's coordinate system
'''
DART_nose = NoseCone(
    length=0.145836, # [m] (# TODO, check against CAD)
    kind="ogive",
    base_radius=DART_rocket_1.radius, # [m] nose cone base radius
    bluffness=0.6/1.5 # ratio between the radius of the circle on the tip of the ogive and the radius of the base of the ogive (will truncate the length if less than 1)
)

'''
-------------------- Add Rail Buttons to Rocket 1 --------------------
upper_button_position: # [m] position of the rail button furthest from the nozzle relative to the rocket's coordinate system
lower_button_position: # [m] position of the rail button closest to the nozzle relative to the rocket's coordinate system
'''
DART_rocket_1.set_rail_buttons(upper_button_position=-0.1, lower_button_position=-0.3) # TODO, positions need to be checked against CAD

DART_rocket_2 = copy.deepcopy(x=DART_rocket_1) # create an independent copy (changes to the copy do not affect the original)

# Apply the `GenericSurface` aerodynamics to Rocket 1
DART_rocket_1.add_surfaces(surfaces=DART_rocket_1_aero_surface, positions=(0,0,0))

# Apply the fins to Rocket 1
DART_rocket_1.add_surfaces(surfaces=DART_fins_rocket_1, positions=(0,0,-0.244369)) # TODO, positions need to be checked against CAD

# Apply the nose cone to Rocket 1
DART_rocket_1.add_surfaces(surfaces=DART_nose, positions=(0, 0, 0.389190)) # TODO, positions need to be checked against CAD

# Apply the fins to Rocket 2
DART_rocket_2.add_surfaces(surfaces=DART_fins_rocket_2, positions=(0,0,0)) # TODO, positions need to be adjusted to reflect the distance between the TVC motor and the fins

# GeoDataFrame of Launch Site Coordinates
gdf_launch_site = gpd.GeoDataFrame(data={"longitude": [launch_site.longitude],
                                         "latitude": [launch_site.latitude],
                                         "color": "launch_site"},
                                    geometry=gpd.points_from_xy([launch_site.longitude], [launch_site.latitude]),
                                    crs="EPSG:4326") # create GeoDataFrame from lat/longs

# GeoDataFrame of Landing Zone Coordinates (Centers Only)
gdf_landing_zone_centers = gpd.GeoDataFrame(data={"longitude": landing_zone_longs,
                                                  "latitude": landing_zone_lats,
                                                  "color": "landing_zone"},
                                            geometry=gpd.points_from_xy(landing_zone_longs, landing_zone_lats),
                                            crs="EPSG:4326") # create GeoDataFrame from lat/longs

landing_zone_radius = 5 # [m] radius of desired landing zone

# Create GeoDataFrame of Landing Zone Perimeters
gdf_landing_zone_perimeters = gdf_landing_zone_centers.copy(deep=True)
gdf_landing_zone_perimeters.crs = "epsg:4326" # establish CRS for GeoDataFrame with landing zone perimeters
gdf_landing_zone_perimeters = gdf_landing_zone_perimeters.to_crs(crs=3857) # transition GeoDataFrame with landing zone perimeters to projected coordinate system
gdf_landing_zone_perimeters.geometry = gdf_landing_zone_perimeters.buffer(distance=landing_zone_radius) # apply buffer (radius) to landing zone centers
gdf_landing_zone_perimeters = gdf_landing_zone_perimeters.to_crs(crs=4326)

landing_zone_perimeters_series = [perimeter for perimeter in gdf_landing_zone_perimeters.geometry]

all_landing_zone_perimeters = [] # list to store lat/long coordinates of landing zone perimeters
for perimeter in landing_zone_perimeters_series:
    longitudes = np.reshape(perimeter.boundary.coords.xy[0], (len(perimeter.boundary.coords.xy[0]), 1))
    latitudes = np.reshape(perimeter.boundary.coords.xy[1], (len(perimeter.boundary.coords.xy[1]), 1))
    coords = np.concatenate((longitudes, latitudes), axis=1)
    all_landing_zone_perimeters.append(coords)
all_landing_zone_perimeters = np.array(all_landing_zone_perimeters)

# GeoDataFrame of All Geographical Data for Plotting
total_gdf = pd.concat([gdf_launch_site, gdf_landing_zone_centers, gdf_landing_zone_perimeters])

# Figure of Launch Site and Landing Zones Overlaid on Satellite Image Base Layer
launch_area_ax = total_gdf.plot(column="color", facecolor="none", markersize=4)
cx.add_basemap(ax=launch_area_ax, source="Esri.WorldImagery", crs=total_gdf.crs.to_string(), attribution_size=2)
launch_area_ax.set_xlabel("Longitude [deg]")
launch_area_ax.tick_params(axis='x', labelrotation=45)
launch_area_ax.set_ylabel("Latitude [deg]")
launch_area_ax.grid(which="major", axis="both")

'''
Effective Launch Rail Length: length in which the rocket will be attached to the rail, only moving along a fixed direction (the line parallel to the rail)
Source: (https://docs.rocketpy.org/en/latest/reference/classes/Flight.html#rocketpy.Flight.__init__)

Measurements:
- Total Rail Length: 71 [in]
- Rail Length above Launch Stand: 63.25 [in]
- Distance between Launch Stand Pad and L-Bracket Hardstop: 2.125 [in]
- Lower Rail Button to Bottom of Ascent Motor Mount: 8.14 [in]
- Lower Rail Button to Bottom of Ascent Motor Mount Aft Closure: 8.58 [in]
- Upper Rail Button to Bottom of Ascent Motor Mount Aft Closure: 16.58 [in]

Final (Calculated) Length Options:
- Distance between the Top Rail Button and the Top of the Rail: 1.131 [m], 44.545 [in] (accounting for L-bracket hardstop)
- Distance between the Lower Rail Button and the Top of the Rail: 1.335 [m], 52.545 [in] (accounting for L-bracket hardstop)
'''
launch_rail_length = 1.131 # [m]