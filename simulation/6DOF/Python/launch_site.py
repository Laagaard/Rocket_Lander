# Python Libraries
import datetime
import json
import math
import re
from rocketpy import Environment, prints
import sys
import warnings
# DART Modules
import motors

FILE_NAME = sys.argv[0][2:] # [str] name of the module executed on the command line

command_line_args = {
  "date": "",
  "time": "",
  "location": "ROAR", # default location corresponds to launching independently in the Compound (at the SRA launch location)
  "windmag": None, # default will pull API
  "windhead": None, # default will pull API
  "parachute": False, # default will not add a parachute to the rocket
  "automated": False # default value indicates the script is being run manually
} # dictionary of recognized command line arguments

if (len(sys.argv) > 1): # if launch parameters are passed on the command line
    for idx in range(1, len(sys.argv), 2): # iterate over command line arguments
        try:
            command_line_regex = re.match("--[A-Za-z]+", sys.argv[idx]).group(0)[2:] # attempt to match the argument key to the format "--<KEY>""
        except (AttributeError): # AttributeError thrown when the regex fails to match
            warnings.warn(f"\"{sys.argv[idx]}\" is not a valid command line argument key")
            continue # continue to the next command line argument (will leave the dictionary entry untouched)
        if (command_line_regex in command_line_args): # if regex-compliant argument matches one of the keys in the dictionary
            command_line_args[command_line_regex] = sys.argv[idx + 1] # assign the corresponding command line arg to the dictionary value
else: # read launch parameters from config file
    config_file_path_prefix = ""
    for ctr in range(motors.directory_levels_to_try):
        try:
            config_file = open(config_file_path_prefix + "launch_parameters.json")
        except (FileNotFoundError): # FileNotFoundError raised when the config file isn't found
            config_file_path_prefix += "../"
        else:
            break
    command_line_args_json = json.load(config_file) # load the config file
    for key, value in command_line_args_json.items():
        try:
            command_line_regex = re.match("--[A-Za-z]+", key).group(0)[2:] # attempt to match the argument key to the format "--<KEY>""
        except (AttributeError): # AttributeError thrown when the regex fails to match
            warnings.warn(f"\"{key}\" is not a valid command line argument key")
            continue # continue to the next command line argument (will leave the dictionary entry untouched)
        if (command_line_regex in command_line_args): # if regex-compliant argument matches one of the keys in the dictionary
            command_line_args[command_line_regex] = value # assign the corresponding command line arg to the dictionary value

# Establish Launch Date and Time (EST)
if (command_line_args["date"] != "" and command_line_args["time"] != ""): # if the launch date and time were not set
    launch_date = datetime.datetime.strptime(command_line_args["date"], "%m-%d-%Y") # launch date
    launch_hour = int(command_line_args["time"])
    launch_time = datetime.time(hour=launch_hour, minute=00) # # launch time (hr, min) (input as EST)
    launch_date_and_time = datetime.datetime.combine(launch_date, launch_time) # launch date and time
else: # the launch date and time were NOT passed as command line arguments
    launch_date_and_time = datetime.datetime.now() # launch date and time

parachute_flag = command_line_args["parachute"] # flag to signal whether or not to add the parachute to the rocket (`True` will add the parachute)
automation_flag = command_line_args["automated"] # flag to signal whether or not the program is being executed by an automatic runner (`True` if it is)

'''
Establish Launch Site Latitude & Longitude
Independent: https://www.google.com/maps/@27.933873,-80.7094486,55m/data=!3m1!1e3?entry=ttu&g_ep=EgoyMDI1MDEwOC4wIKXMDSoASAFQAw%3D%3D
ROAR, NAR section 795: https://www.google.com/maps/@28.5633031,-81.0187189,261m/data=!3m1!1e3?authuser=1&hl=en&entry=ttu&g_ep=EgoyMDI1MDEyOS4xIKXMDSoASAFQAw%3D%3D
'''
launch_site_latitude_independent = 27.933880 # [deg] North, launch site latitude (if launching independently (i.e., without a NAR section))
launch_site_longitude_independent = -80.709505 # [deg] West, launch site longitude (if launching independently (i.e., without a NAR section))
launch_site_latitude_ROAR = 28 + (33/60) + (48/3600) # [deg] North, launch site latitude (if launching with ROAR, NAR section 795)
launch_site_longitude_ROAR = -(81 + (1/60) + (2/3600)) # [deg] West, launch site longitude (if launching with ROAR, NAR section 795)
launch_site_latitude_SRA = 27.932715  # [deg] North, launch site latitude (if launching with SRA)
launch_site_longitude_SRA = -80.709536  # [deg] West, launch site longitude (if launching with SRA)

# Set the Launch Site Location
match (command_line_args["location"]):
    case ("independent"): # launching independently (i.e., without a NAR section) at the SRA launch site in the Compound
        launch_site_latitude = launch_site_latitude_independent # [deg]
        launch_site_longitude = launch_site_longitude_independent # [deg]
    case ("ROAR"): # launching with Regional Orlando Applied Rocketry (NAR Section 795, https://www.flroar.space/)
        launch_site_latitude = launch_site_latitude_ROAR # [deg]
        launch_site_longitude = launch_site_longitude_ROAR # [deg]
    case ("SRA"): # launching with Spaceport Rocketry Association (https://www.spaceportrocketry.org/index.html)
        launch_site_latitude = launch_site_latitude_SRA # [deg]
        launch_site_longitude = launch_site_longitude_SRA # [deg]

# Construct Launch Site Environment
launch_site = Environment(
    date=launch_date_and_time, # launch date and time
    latitude=launch_site_latitude, # [deg] positive corresponds to North
    longitude=launch_site_longitude, # [deg] positive corresponds to East
    elevation=4, # [m] launch site elevation above sea level
    timezone="EST", # specify launch site time zone
    max_expected_height=250 # [m] maximum altitude to keep weather data (must be above sea level)
)

# Landing Zone Coordinates (if launching independently)
landing_zone_lats_independent = [27.935514, 27.935504, 27.935499, 27.934703, 27.934706, 27.933877, 27.933002, 27.932312, 27.932334, 27.932334, 27.932334] # [deg] latitude coordinates of landing zone centers (clockwise around launch site)
landing_zone_longs_independent = [-80.711389, -80.710455, -80.709524, -80.709506, -80.708361, -80.708351, -80.708350, -80.708283, -80.709533, -80.710461, -80.711391] # [deg] longitude coordinates of landing zone centers (clockwise around launch site)

# Landing Zone Coordinates (if launching with ROAR, NAR section 795)
landing_zone_lats_ROAR = [launch_site.latitude + 0.001,         launch_site.latitude + 0.001,       launch_site.latitude + 0.001,       launch_site.latitude + 0.001,       launch_site.latitude + 0.001,
                          launch_site.latitude + (2*0.001/3),   launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3), launch_site.latitude + (2*0.001/3),
                          launch_site.latitude + 0.001/3,       launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3,     launch_site.latitude + 0.001/3] # [deg] latitude coordinates of landing zone centers
landing_zone_longs_ROAR = [launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,
                           launch_site.longitude - 0.001,       launch_site.longitude - 0.001/2,    launch_site.longitude,              launch_site.longitude + 0.001/2,    launch_site.longitude + 0.001,] # [deg] longitude coordinates of landing zone centers

# Landing Zone Coordinates (if launching with Spaceport Rocketry Association)
landing_zone_lats_SRA = landing_zone_lats_independent  # [deg] latitude coordinates of landing zone centers (clockwise around launch site)
landing_zone_longs_SRA = landing_zone_longs_independent  # [deg] longitude coordinates of landing zone centers (clockwise around launch site)

# Set Landing Zone Coordinates
match (command_line_args["location"]):
    case ("independent"): # launching independently (i.e., without a NAR section) at the SRA launch site in the Compound
        landing_zone_lats = landing_zone_lats_independent # [deg]
        landing_zone_longs = landing_zone_longs_independent # [deg]
    case ("ROAR"): # launching with Regional Orlando Applied Rocketry (NAR Section 795, https://www.flroar.space/)
        landing_zone_lats = landing_zone_lats_ROAR # [deg]
        landing_zone_longs = landing_zone_longs_ROAR # [deg]
    case ("SRA"): # launching with Spaceport Rocketry Association (https://www.spaceportrocketry.org/index.html)
        landing_zone_lats = landing_zone_lats_SRA # [deg]
        landing_zone_longs = landing_zone_longs_SRA # [deg]

'''
-------------------- Add Forecast (i.e., Wind) Information --------------------
Ensemble, GEFS: 1-deg geographical resolution, updated every 6 hours (00, 06, 12, 18UTC) (experimentally determined to have the same forecast depth as GFS)
Forecast, GFS: 0.25-deg geographical resolution, updated every 6 hours (good balance)
Forecast, RAP: 0.19-deg geographical resolution, updated hourly (best temporal resolution and update frequency)
Forecast, NAM: ~0.045-deg geographical resolution, updated every 6 hours with points spaced every 3 hours (best geographical resolution) (https://www.ncei.noaa.gov/products/weather-climate-models/north-american-mesoscale)
'''
if (command_line_args["windmag"] != None and command_line_args["windhead"] != None): # if wind parameters are set on the command line or in the config file
    wind_mag = float(command_line_args["windmag"]) # [m/s] wind velocity magnitude
    wind_head = float(command_line_args["windhead"]) # [deg] direction in which the wind is blowing, CW from North
    wind_u = wind_mag * math.sin(math.radians(wind_head)) # [m/s] East/West component of wind velocity
    wind_v = wind_mag * math.cos(math.radians(wind_head)) # [m/s] North/South component of wind velocity
    launch_site.process_custom_atmosphere(wind_u = wind_u, wind_v = wind_v)
else: # pull wind information from API
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

# If one of the following modules was executed directly: launch_site.py, setup.py
if (FILE_NAME == "launch_site.py" or FILE_NAME == "setup.py"):
    # Print information of launch site conditions (if not using ISA)
    launch_site_prints = prints.environment_prints._EnvironmentPrints(launch_site)
    launch_site_prints.all()