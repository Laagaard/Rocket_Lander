# Libraries
import csv
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import matplotlib.pyplot as plt
import numpy as np
import os
from rocketpy import Flight
from scipy.spatial import ConvexHull
import shutil
import sys
sys.path.append("../") # Tell Python where to look for the `setup.py` file
from setup import all_landing_zone_perimeters, automation_flag, DART_rocket_1, date_dir_date_only, date_dir_with_time, gdf_landing_zone_centers, launch_rail_length, launch_site, launch_area_ax, remove_readonly

figures_output_dir = f"{date_dir_with_time}/figures" # output directory for matplotlib figures

# Run if the script is executed directly (i.e., not as a module)
if __name__ == "__main__":
    if os.path.exists(date_dir_date_only):
        shutil.rmtree(date_dir_date_only, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(date_dir_date_only) # Create folder for all results for the given date

    if os.path.exists(date_dir_with_time):
        shutil.rmtree(date_dir_with_time, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(date_dir_with_time) # Create folder for all results for the given date/time

    if os.path.exists(figures_output_dir):
        shutil.rmtree(figures_output_dir, onerror=remove_readonly) # remove existing directory (and, thereby, all files in it)
    os.mkdir(figures_output_dir) # Create folder for all results for the given date/time

    # CSV Output File
    trajectory_dataset_output_file_header = ["Inclination", "Heading", "longitude", "latitude"] # header of output CSV file containing trajectory information
    trajectory_dataset_output_file = open(f"{date_dir_with_time}/trajectory_dataset.csv", 'w', newline="") # output CSV file containing optimal trajectory information
    trajectory_dataset_writer = csv.writer(trajectory_dataset_output_file) # CSV writer for output file containing optimal trajectory information
    trajectory_dataset_writer.writerow(trajectory_dataset_output_file_header) # write header row of output CSV file containing optimal trajectory information

    impact_longs = np.array([]) # [m] list to track impact longitudes
    impact_lats = np.array([]) # [m] list to track impact latitudes

    success_bool = False # boolean to track primary algorithm success (w.r.t largest Path encompassing an entire landing zone)
    launch_inclination = 90 # [deg]
    launch_heading = 0 # [deg]
    inclination_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a random plotting color

    try:
        while (not success_bool):
            test_flight = Flight(
                rocket=DART_rocket_1,
                environment=launch_site,
                rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
                inclination=launch_inclination, # [deg] rail inclination relative to the ground
                heading=launch_heading, # [deg] heading angle relative to North (East = 90)
                time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
            ) # run trajectory simulation

            solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution

            impact_longitude = test_flight.longitude(solution_time)[-1] # [deg] longitude of impact location
            impact_latitude = test_flight.latitude(solution_time)[-1] # [deg] latitude of impact location

            trajectory_information = [test_flight.inclination, test_flight.heading, impact_longitude, impact_latitude]
            trajectory_dataset_writer.writerow(trajectory_information) # write trajectory information to output CSV file

            if (not automation_flag): # if the script is being run manually (i.e., not by an automatic runner)
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
    except (IndexError, ValueError): # Error occurs when the inclination goes too low for RocketPy to handle
        print("Inclination Too Low - Stopping Search")

    # Close trajectory dataset output file
    trajectory_dataset_output_file.close()

    # Optimal Landing Zone Parameters Output File
    optimal_landing_zone_output_file = open(f"{date_dir_with_time}/optimal_landing_zone.csv", mode="w", newline="")
    optimal_landing_zone_writer = csv.writer(optimal_landing_zone_output_file) # CSV writer for output file containing optimal landing zone information
    optimal_landing_zone_writer.writerow(["index", "longitude", "latitude"]) # write header row of output CSV file containing optimal landing zone information

    # Record Latitude and Longitude of Optimal Landing Zone Center
    if (success_bool):
        if (not automation_flag): # if the script is being run manually
            print("Houston, we have an INTERPOLATION problem")
        optimal_index = np.where(np.all(all_landing_zone_perimeters == perimeter_coords, axis=(1,2)))[0][0]
        optimal_landing_zone_center_longitude = gdf_landing_zone_centers["longitude"][optimal_index] # [deg] longitude of the center of the optimal landing zone
        optimal_landing_zone_center_latitude = gdf_landing_zone_centers["latitude"][optimal_index] # [deg] latitude of the center of the optimal landing zone
        optimal_landing_zone_information = [optimal_index, optimal_landing_zone_center_longitude, optimal_landing_zone_center_latitude]
        optimal_landing_zone_writer.writerow(optimal_landing_zone_information)
    else:
        if (not automation_flag): # if the script is being run manually
            print("Houston, we have an EXTRAPOLATION problem")
        optimal_landing_zone_writer.writerow(["None", "None", "None"])
    optimal_landing_zone_output_file.close()

    launch_area_ax.set_title(f"Impact Area Ground Coverage \n(Minimum Inclination: {round(test_flight.inclination, 2)} deg)") # add graph title
    plt.tight_layout()
    plt.savefig(f"{figures_output_dir}/impact_area.png", transparent=True, dpi=1000) # save the figure with a transparent background

    if (not automation_flag): # if the script is being run manually
        plt.show() # show the graph