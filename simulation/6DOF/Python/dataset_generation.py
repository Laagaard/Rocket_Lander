# Libraries
import contextily as cx
import csv
import geopandas as gpd
import math
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from rocketpy import Flight
from scipy.spatial import ConvexHull
import shutil
from setup import DART_rocket, launch_site, remove_readonly, results_dir

date_format_string = "%Y-%m-%d-%H-%M-%S"
date_dir = f"{results_dir}/{launch_site.local_date.strftime(date_format_string)}"

figures_output_dir = f"{date_dir}/figures" # output directory for matplotlib figures

landing_zone_radius = 5 # [m] radius of desired landing zone

gdf_launch_site = gpd.GeoDataFrame(data={"longitude": [launch_site.longitude],
                                         "latitude": [launch_site.latitude],
                                         "color": "launch_site"},
                                    geometry=gpd.points_from_xy([launch_site.longitude], [launch_site.latitude]),
                                    crs="EPSG:4326") # create GeoDataFrame from lat/longs

landing_zone_lats = [27.935514, 27.935504, 27.935499, 27.934703, 27.934706, 27.933877, 27.933002, 27.932312, 27.932334, 27.932334, 27.932334] # [deg] coordinates of landing zone centers (clockwise around launch site)
landing_zone_longs = [-80.711389, -80.710455, -80.709524, -80.709506, -80.708361, -80.708351, -80.708350, -80.708283, -80.709533, -80.710461, -80.711391] # [deg] coordinates of landing zone centers (clockwise around launch site)

gdf_landing_zone_centers = gpd.GeoDataFrame(data={"longitude": landing_zone_longs,
                                                  "latitude": landing_zone_lats,
                                                  "color": "landing_zone"},
                                            geometry=gpd.points_from_xy(landing_zone_longs, landing_zone_lats),
                                            crs="EPSG:4326") # create GeoDataFrame from lat/longs

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

total_gdf = pd.concat([gdf_launch_site, gdf_landing_zone_centers, gdf_landing_zone_perimeters])

# Figure of Launch Site and Landing Zones Overlaid on Satellite Image Base Layer
launch_area_ax = total_gdf.plot(column="color", facecolor="none", markersize=4)
cx.add_basemap(ax=launch_area_ax, source="Esri.WorldImagery", crs=total_gdf.crs.to_string())
launch_area_ax.set_xlabel("Longitude [deg]")
launch_area_ax.tick_params(axis='x', labelrotation=45)
launch_area_ax.set_ylabel("Latitude [deg]")
launch_area_ax.grid(which="major", axis="both")

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    if os.path.exists(date_dir):
        shutil.rmtree(date_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(date_dir) # Create folder for all results for the given date/time

    if os.path.exists(figures_output_dir):
        shutil.rmtree(figures_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(figures_output_dir) # Create folder for all results for the given date/time

    # CSV Output File
    trajectory_dataset_output_file_header = ["Inclination", "Heading", "longitude", "latitude"] # header of output CSV file containing trajectory information
    trajectory_dataset_output_file = open("trajectory_dataset.csv", 'w', newline="") # output CSV file containing optimal trajectory information
    trajectory_dataset_writer = csv.writer(trajectory_dataset_output_file) # CSV writer for output file containing optimal trajectory information
    trajectory_dataset_writer.writerow(trajectory_dataset_output_file_header) # write header row of output CSV file containing optimal trajectory information

    impact_longs = np.array([]) # [m] list to track impact longitudes
    impact_lats = np.array([]) # [m] list to track impact latitudes

    success_bool = False # boolean to track primary algorithm success (w.r.t largest Path encompassing an entire landing zone)
    launch_inclination = 89 # [deg]
    launch_heading = 0 # [deg]
    inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a random plotting color

    # try:
    while (not success_bool):
        test_flight = Flight(
            rocket=DART_rocket,
            environment=launch_site,
            rail_length=1.5, # [m] length in which the rocket will be attached to the launch rail
            inclination=launch_inclination, # [deg] rail inclination relative to the ground
            heading=launch_heading, # [deg] heading angle relative to North (East = 90)
            time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
        ) # run trajectory simulation

        solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

        impact_longitude = test_flight.longitude(solution_time)[-1] # [deg] longitude of impact location
        impact_latitude = test_flight.latitude(solution_time)[-1] # [deg] latitude of impact location

        trajectory_information = [test_flight.inclination, test_flight.heading, impact_longitude, impact_latitude]
        trajectory_dataset_writer.writerow(trajectory_information) # write trajectory information to output CSV file

        print(f"Inclination: {round(test_flight.inclination, 2)} deg, Heading: {round(test_flight.heading, 2)} deg")

        if (launch_heading < 360): # launch heading can be increased more
            launch_heading += 5 # [deg] increase launch heading
            impact_longs = np.append(impact_longs, np.array([test_flight.longitude(solution_time)[-1]])) # [m] add newest impact longitude to list of tracked longitudes
            impact_lats = np.append(impact_lats, np.array([test_flight.latitude(solution_time[-1])])) # [m] add newest impact latitude to list of tracked latitudes
        else: # launch heading has reached maximum (practical) value (i.e., 360 deg)
            launch_heading = 0 # [deg] reset launch heading
            launch_inclination -= 1 # [deg] decrease launch inclination

            impact_longs = np.append(impact_longs, np.array([test_flight.longitude(solution_time)[-1]])) # [m] add newest x-impact coordinate to list of tracked coordinates
            impact_lats = np.append(impact_lats, np.array([test_flight.latitude(solution_time)[-1]])) # [m] add newest y-impact coordinate to list of tracked coordinates
            impact_coords = np.concatenate((impact_longs.reshape(len(impact_longs), 1), impact_lats.reshape(len(impact_lats), 1)), axis=1)
            hull = ConvexHull(points=impact_coords) # fit convex hull to all impact locations
            hull_path = Path(vertices=impact_coords[hull.vertices], closed=True) # create a Path with the hull vertices, only works for 2D points
            hull_path_patch = PathPatch(path=hull_path, edgecolor=inclination_color, facecolor="None") # create a Patch of the path (for plotting)
            launch_area_ax.add_patch(hull_path_patch) # for some reason, this has to be here for the following if statement to work properly
            for perimeter_coords in all_landing_zone_perimeters:
                if (all(hull_path.contains_points(points=perimeter_coords))): # check if the path encompasses an entire landing zone
                    success_bool = True # the path encompasses an entire landing zone
                    break
            inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a new plotting color

    # Close trajectory dataset output file
    trajectory_dataset_output_file.close()

    # Optimal Landing Zone Parameters Output File
    optimal_landing_zone_output_file = open("optimal_landing_zone.csv", mode="w", newline="")
    optimal_landing_zone_writer = csv.writer(optimal_landing_zone_output_file) # CSV writer for output file containing optimal landing zone information
    optimal_landing_zone_writer.writerow(["index", "longitude", "latitude"]) # write header row of output CSV file containing optimal landing zone information

    # Record Latitude and Longitude of Optimal Landing Zone Center
    if (success_bool):
        print("Houston, we have an INTERPOLATION problem")
        optimal_index = np.where(np.any(all_landing_zone_perimeters == perimeter_coords, axis=(1,2)))[0][0]
        optimal_landing_zone_information = [optimal_index, gdf_landing_zone_centers["longitude"][optimal_index], gdf_landing_zone_centers["latitude"][optimal_index]]
        optimal_landing_zone_writer.writerow(optimal_landing_zone_information)
    else:
        print("Houston, we have an EXTRAPOLATION problem")
        optimal_landing_zone_writer.writerow(["None", "None"])
    optimal_landing_zone_output_file.close()

    launch_area_ax.set_title(f"Trajectory & Landing Zone \n(Inclination: {round(test_flight.inclination, 2)} deg)") # add graph title
    plt.savefig(f"{figures_output_dir}/impact_area.png") # save the figure
    plt.show() # show the graph