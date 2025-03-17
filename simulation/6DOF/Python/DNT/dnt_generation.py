# Libraries
import csv
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import os
import pandas as pd
from skimage.measure import EllipseModel
from skspatial.objects import Line, Point
import sys
sys.path.append("../")
# DART Modules
from dataset_generation import figures_output_dir, launch_area_ax
from dnt_trajectories import CSV_output_dir, optimal_perimeter_coords
from setup import automation_flag, date_dir_with_time, launch_site

dnt_temporal_resolution = 0.15 # [s] time-step of each DNT discretization, be careful setting too high or too low
timestep_current_lower_bound = dnt_temporal_resolution # [s] lower time bound of current discretization
dnt_left_boundary_xs = [launch_site.longitude] # [m] x-coordinates comprising the left boundary of the DNT
dnt_left_boundary_ys = [launch_site.latitude] # [m] y-coordinates comprising the left boundary of the DNT
dnt_right_boundary_xs = [launch_site.longitude] # [m] x-coordinates comprising the right boundary of the DNT
dnt_right_boundary_ys = [launch_site.latitude] # [m] y-coordinates comprising the right boundary of the DNT
dnt_upper_altitudes = [launch_site.elevation] # [m] z-coordinate comprising the upper boundary of each DNT discretization
dnt_lower_altitudes = [0] # [m] z-coordinate comprising the lower boundary of each DNT discretization
time_vector = [0] # [s] time array

optimal_df = pd.read_csv(f"{date_dir_with_time}/optimal_trajectory.csv") # df of optimal trajectory data

test_longitudes = [] # [m] list to track x-coordinates of test points
test_latitudes = [] # [m] list to track y-coordinates of test points
test_altitudes = [] # [m] list to track altitudes of test points

# DNT Plot
launch_area_ax.plot(optimal_df[" Longitude (°)"], optimal_df[" Latitude (°)"], 'b', label="Optimal Trajectory")
launch_area_ax.set_title(f"Domain of Nominal Trajectories \n(Time Resolution: {dnt_temporal_resolution} [s])")

optimal_idx = (optimal_df["# Time (s)"] - timestep_current_lower_bound).abs().idxmin() # index of solution step of optimal trajectory nearest to the desired time value
reference_longitude = optimal_df[" Longitude (°)"][optimal_idx] # [m] longitude coordinate of reference location
reference_latitude = optimal_df[" Latitude (°)"][optimal_idx] # [m] latitude coordinate of reference locations

ellipse = EllipseModel()
if (ellipse.estimate(optimal_perimeter_coords)): # fit the best-fit model to the optimal landing zone perimeter coordss
    landing_zone_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor='r', facecolor="None")
    launch_area_ax.add_patch(landing_zone_patch)
    # Loop until the optimal trajectory enters the landing zone
    while (not landing_zone_patch.contains_point(point=launch_area_ax.transData.transform(values=(reference_longitude, reference_latitude)))):
        if (not automation_flag):
            print(f"Current Time: {round(timestep_current_lower_bound, 2)}")
        optimal_idx = (optimal_df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step of optimal trajectory nearest to the desired time value
        reference_longitude = optimal_df[" Longitude (°)"][optimal_idx] # [m] longitude coordinate of reference location
        reference_latitude = optimal_df[" Latitude (°)"][optimal_idx] # [m] latitude coordinate of reference location
        optimal_vx = optimal_df[" Vx (m/s)"][optimal_idx] # [m/s] x_velocity of the optimal trajectory nearest the time step of interest
        optimal_vy = optimal_df[" Vy (m/s)"][optimal_idx] # [m/s] y_velocity of the optimal trajectory nearest the time step of interest

        try:
            optimal_traj_tangent_slope = optimal_vy/optimal_vx # slope of tangent line to the optimal trajectory
            optimal_traj_tangent_line = Line(point=[reference_longitude, reference_latitude], direction=[1, optimal_traj_tangent_slope]) # line tangent to the trajectory
            optimal_traj_normal_line = Line(point=[reference_longitude, reference_latitude], direction=[1, -1/optimal_traj_tangent_slope]) # line perpendicular to the trajectory
        except (ValueError): # ValueError thrown if `optimal_vx` or `optimal_vy` equals zero (results in infinite slope for one of the lines)
            timestep_current_lower_bound += dnt_temporal_resolution # increment current time step
            continue

        # Iterate over files in `dnt_trajectories` directory
        for filename in os.listdir(CSV_output_dir):
            df = pd.read_csv(f"{CSV_output_dir}/{filename}") # copy CSV data into pandas dataframe

            test_idx = (df["# Time (s)"] - timestep_current_lower_bound).abs().idxmin() # index of solution step nearest to the desired time value

            test_longitude = df[" Longitude (°)"][test_idx] # [m] longitude coordinate of test location
            test_latitude = df[" Latitude (°)"][test_idx] # [m] latitude coordinate of test location
            test_altitude = df[" Altitude AGL (m)"][test_idx] # [m] z-coordinate (altitude) of test location

            # Append to Tracking Lists
            test_longitudes.append(test_longitude)
            test_latitudes.append(test_latitude)
            test_altitudes.append(test_altitude)

        projected_test_points_longitudes = [] # list to track the longitude coordinate of each projected point
        projected_test_points_latitudes = [] # list to track the latitude coordinate of each projected point
        projected_test_points_distance = [] # list to track the distances of each projected point to the reference point
        projected_test_points_side = [] # list to track which side of the optimal trajectory each projected point is on
        for idx in range(len(test_longitudes)):
            test_point_object = Point([test_longitudes[idx], test_latitudes[idx]]) # create scipy point object for the test point
            test_point_object_projected = optimal_traj_normal_line.project_point(test_point_object) # project the test point onto the perpendicular-to-the-trajectory line

            projected_test_point_longitude = test_point_object_projected[0] # [m] longitude coordinate of the projected test point
            projected_test_point_latitude = test_point_object_projected[1] # [m] latitude coordinate of the projected test point
            projected_test_point_distance = math.sqrt((projected_test_point_longitude - reference_longitude)**2 + (projected_test_point_latitude - reference_latitude)**2) # [m] distance between the projected test point and the reference point

            projected_test_points_longitudes.append(projected_test_point_longitude)
            projected_test_points_latitudes.append(projected_test_point_latitude)
            projected_test_points_distance.append(projected_test_point_distance)

            # Determine the side of the optimal trajectory the projected test point lies on
            if (optimal_vx > 0): # the optimal trajectory x_velocity is positive
                if (projected_test_point_latitude > reference_latitude): # the projected test point is above the reference point
                    projected_test_points_side.append(0) # the projected test point lies to the LEFT of the optimal trajectory
                else:
                    projected_test_points_side.append(1) # the projected test point lies to the RIGHT of the optimal trajectory
            else: # the optimal trajectory x_velocity is negative
                if (projected_test_point_latitude > reference_latitude): # the projected test point is above the reference point
                    projected_test_points_side.append(1) # the projected test point lies to the RIGHT of the optimal trajectory
                else:
                    projected_test_points_side.append(0) # the projected test point lies to the LEFT of the optimal trajectory

        indices_of_left = [idx for idx, elem in enumerate(projected_test_points_side) if elem == 0] # get indices of projected test points on the LEFT of the optimal trajectory
        left_point_xs = [projected_test_points_longitudes[idx] for idx in indices_of_left] # [m] x-coordinatles of each projected point on the LEFT of the optimal trajectory
        left_point_ys = [projected_test_points_latitudes[idx] for idx in indices_of_left] # [m] y-coordinatles of each projected point on the LEFT of the optimal trajectory
        left_point_distances = [projected_test_points_distance[idx] for idx in indices_of_left] # [m] distance from each point on the LEFT of the optimal trajectory to the reference point
        left_bound_index = left_point_distances.index(max(left_point_distances)) # index of the point with the greatest distance
        dnt_left_boundary_xs.append(left_point_xs[left_bound_index])
        dnt_left_boundary_ys.append(left_point_ys[left_bound_index])

        indices_of_right = [idx for idx, elem in enumerate(projected_test_points_side) if elem == 1] # get indices of projected test points on the RIGHT of the optimal trajectory
        right_point_xs = [projected_test_points_longitudes[idx] for idx in indices_of_right] # [m] x-coordinatles of each projected point on the RIGHT of the optimal trajectory
        right_point_ys = [projected_test_points_latitudes[idx] for idx in indices_of_right] # [m] y-coordinatles of each projected point on the RIGHT of the optimal trajectory
        right_point_distances = [projected_test_points_distance[idx] for idx in indices_of_right] # [m] distance from each point on the RIGHT of the optimal trajectory to the reference point
        right_bound_index = right_point_distances.index(max(right_point_distances)) # index of the point with the greatest distance
        dnt_right_boundary_xs.append(right_point_xs[right_bound_index])
        dnt_right_boundary_ys.append(right_point_ys[right_bound_index])

        dnt_lower_altitudes.append(min(test_altitudes))
        dnt_upper_altitudes.append(max(test_altitudes))

        # Reset all tracking lists
        test_longitudes = []
        test_latitudes = []
        test_altitudes = []
        projected_test_points_longitudes = []
        projected_test_points_latitudes = []
        projected_test_points_distance = []
        projected_test_points_side = []

        time_vector.append(timestep_current_lower_bound) # append current lower time bound to the time array
        timestep_current_lower_bound += dnt_temporal_resolution # increment current time step

dnt_file = open(f"{date_dir_with_time}/DNT.csv", 'w', newline="") # output CSV file containing the points forming the DNT boundaries
dnt_writer = csv.writer(dnt_file) # CSV writer for output file containing the points of the DNT boundaries
dnt_writer.writerow(["t", "x_1", "y_1", "x_2", "y_2"]) # writer header row of output CSV file containing the points of the DNT boundaries

for idx in range(len(dnt_left_boundary_xs)):
    t = time_vector[idx]
    x_1 = dnt_left_boundary_xs[idx]
    y_1 = dnt_left_boundary_ys[idx]
    x_2 = dnt_right_boundary_xs[idx]
    y_2 = dnt_right_boundary_ys[idx]
    dnt_writer.writerow([round(t, 2), x_1, y_1, x_2, y_2])

launch_area_ax.plot(dnt_left_boundary_xs, dnt_left_boundary_ys, 'b.-', markersize=1)
launch_area_ax.plot(dnt_right_boundary_xs, dnt_right_boundary_ys, 'b.-', markersize=1)

dnt_file.close() # close the file

plt.tight_layout()
plt.savefig(f"{figures_output_dir}/DNT.png", transparent=True, dpi=1000) # save the figure with a transparent background

if (not automation_flag): # if the script is being run manually
    plt.show()