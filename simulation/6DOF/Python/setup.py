# Python Libraries
import contextily as cx
import geopandas as gpd
import numpy as np
import os
import pandas as pd
import stat
# DART Modules
from launch_site import landing_zone_longs, landing_zone_lats, launch_site
from launch_site import automation_flag # imported to simplify DNT and TVC imports
import motors
from rockets import DART_rocket_1, DART_rocket_2, DART_rocket_3 # imported to simplify DNT and TVC imports
import rockets

# Function for `shutil.rmtree` to call on "Access is denied" error from read-only folder
def remove_readonly(func, path, excinfo):
    os.chmod(path, stat.S_IWRITE)
    func(path)

results_dir_file_path_prefix = ""
for ctr in range(motors.directory_levels_to_try):
    try:
        results_dir = results_dir_file_path_prefix + "DNT/Results"
        if os.path.exists(results_dir): # If the results directory already exists
            None
        else:
            os.mkdir(results_dir) # Create folder for all results for the given date/time
    except (FileNotFoundError): # FileNotFoundError raised when the `DNT/Results` directory is not found
        results_dir_file_path_prefix += "../"
    else:
        break

# String Formatting Based on Launch Date and Time (for organization of results)
date_format_string_date_only = "%m-%d-%Y"
date_string_date_only = launch_site.local_date.strftime(date_format_string_date_only)
date_dir_date_only = f"{results_dir}/{date_string_date_only}"
date_format_string_time_only = "%H"
date_string_time_only = launch_site.local_date.strftime(date_format_string_time_only)
date_dir_with_time = f"{date_dir_date_only}/{date_string_time_only}"
date_string_with_time = f"{date_string_date_only}-{date_string_time_only}"

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

DART_rocket_1.TVC = None  # Explicitly disable TVC for Rocket 1

ENABLE_TVC = False  # This will be modified in flight script

# Initialize TVC System
if ENABLE_TVC:
    tvc_system = TVC(max_gimbal=10, servo_rate=375)
    DART_rocket_2.TVC = tvc_system  # Attach TVC to descent rocket
else:
    DART_rocket_2.TVC = None  # Disable TVC if not used

# --- LQR Controller Setup ---
if ENABLE_TVC:
    sampling_rate = 50  # Hz
    controller = _Controller(
        interactive_objects=[DART_rocket_2.TVC, DART_rocket_2],
        controller_function= controller.tvc_lqr_controller,
        sampling_rate=sampling_rate,
        name="LQR Controller"
    )
    DART_rocket_2.controller = controller  # Attach controller to the rocket
