# Libraries
import contextily as cx
import csv
import geopandas as gpd
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from rocketpy import Flight
import shutil
from skimage.measure import EllipseModel
import stat
from setup import DART_rocket, launch_site

figures_output_dir = "figures" # output directory for matplotlib figures

'''
Distance between whole-number longitudes at launch site
Source: https://gis.stackexchange.com/questions/251643/approx-distance-between-any-2-longitudes-at-a-given-latitude/251662#251662
'''
longitude_separation = math.radians(90 - launch_site.latitude) * 111321 # [m] distance between whole-number longitudes at launch site
landing_zone_radius = 5#/longitude_separation # [deg] radius of desired landing zone

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
    # CSV Output File
    trajectory_dataset_output_file_header = ["Inclination", "Heading", "longitude", "latitude"] # header of output CSV file containing trajectory information
    trajectory_dataset_output_file = open("trajectory_dataset.csv", 'w', newline="") # output CSV file containing optimal trajectory information
    trajectory_dataset_writer = csv.writer(trajectory_dataset_output_file) # CSV writer for output file containing optimal trajectory information
    trajectory_dataset_writer.writerow(trajectory_dataset_output_file_header) # write header row of output CSV file containing optimal trajectory information

    impact_longs = np.array([]) # [m] list to track impact longitudes
    impact_lats = np.array([]) # [m] list to track impact latitudes

    success_bool = False # boolean to track primary algorithm success (w.r.t largest best-fit ellipse encompassing landing zone coordinates)
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
            ellipse = EllipseModel() # create best-fit ellipse model
            if (ellipse.estimate(impact_coords)): # fit the best-fit model to the impact coordinates
                ellipse_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor=inclination_color, facecolor="None")
                launch_area_ax.add_patch(ellipse_patch) # for some reason, this has to be here for the following if statement to work properly
                for perimeter_coords in all_landing_zone_perimeters:
                    if (all(ellipse_patch.contains_points(points=launch_area_ax.transData.transform(values=perimeter_coords)))):
                        success_bool = True # largest best-fit ellipse encompasses landing zone coordinates
                        break
            impact_longs = np.array([]) # reset list of x-impact coordinates
            impact_lats = np.array([]) # reset list of ys-impact coordinates
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

    # Function for `shutil.rmtree` to call on "Access is denied" error from read-only folder
    def remove_readonly(func, path, excinfo):
        os.chmod(path, stat.S_IWRITE)
        func(path)

    if os.path.exists(figures_output_dir):
        shutil.rmtree(figures_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(figures_output_dir) # Create folder for CSV files of DNT simulation data

    launch_area_ax.set_title(f"Trajectory & Landing Zone \n(Inclination: {round(test_flight.inclination, 2)} deg)") # add graph title
    plt.savefig(f"{figures_output_dir}/impact_ellipses.png") # save the figure
    plt.show() # show the graph