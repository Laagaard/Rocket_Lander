# Libraries
import csv
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from skimage.measure import EllipseModel
from skspatial.objects import Line, Point
from dataset_generation import date_dir, figures_output_dir, launch_area_ax
from dnt_trajectories import CSV_output_dir, optimal_perimeter_coords
from setup import launch_site_longitude, launch_site_latitude

dnt_temporal_resolution = 0.25 # [s] time-step of each DNT discretization
timestep_current_lower_bound = dnt_temporal_resolution # [s] lower time bound of current discretization
dnt_left_boundary_xs = [launch_site_longitude] # [m] x-coordinates comprising the left boundary of the DNT
dnt_left_boundary_ys = [launch_site_latitude] # [m] y-coordinates comprising the left boundary of the DNT
dnt_right_boundary_xs = [launch_site_longitude] # [m] x-coordinates comprising the right boundary of the DNT
dnt_right_boundary_ys = [launch_site_latitude] # [m] y-coordinates comprising the right boundary of the DNT
dnt_upper_altitudes = [] # [m] z-coordinate comprising the upper boundary of each DNT discretization
dnt_lower_altitudes = [] # [m] z-coordinate comprising the lower boundary of each DNT discretization
time_vector = [0] # [s] time array

optimal_df = pd.read_csv("optimal_trajectory.csv") # df of optimal trajectory data

test_longitudes = [] # [m] list to track x-coordinates of test points
test_latitudes = [] # [m] list to track y-coordinates of test points
test_altitudes = [] # [m] list to track altitudes of test points

# DNT Plot
launch_area_ax.plot(optimal_df["longitude"], optimal_df["latitude"], 'b', label="Optimal Trajectory")
launch_area_ax.set_title(f"Domain of Nominal Trajectories \n(Time Resolution: {dnt_temporal_resolution} [s])")

optimal_idx = (optimal_df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step of optimal trajectory nearest to the desired time value
reference_longitude = optimal_df["longitude"][optimal_idx] # [m] longitude coordinate of reference location
reference_latitude = optimal_df["latitude"][optimal_idx] # [m] latitude coordinate of reference locations

ellipse = EllipseModel()
if (ellipse.estimate(optimal_perimeter_coords)): # fit the best-fit model to the optimal landing zone perimeter coordss
    landing_zone_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor='r', facecolor="None")
    launch_area_ax.add_patch(landing_zone_patch)
    # Loop until the optimal trajectory enters the landing zone
    while (not landing_zone_patch.contains_point(point=launch_area_ax.transData.transform(values=(reference_longitude, reference_latitude)))):
        print(f"Current Time: {timestep_current_lower_bound}")
        time_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a new plotting color
        optimal_idx = (optimal_df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step of optimal trajectory nearest to the desired time value
        reference_longitude = optimal_df["longitude"][optimal_idx] # [m] longitude coordinate of reference location
        reference_latitude = optimal_df["latitude"][optimal_idx] # [m] latitude coordinate of reference locations
        optimal_vx = optimal_df["x_vel"][optimal_idx] # [m/s] x_velocity of the optimal trajectory nearest the time step of interest
        optimal_vy = optimal_df["y_vel"][optimal_idx] # [m/s] y_velocity of the optimal trajectory nearest the time step of interest
        optimal_traj_tangent_slope = optimal_vy/optimal_vx # slope of tangent line to the optimal trajectory
        optimal_traj_tangent_line = Line(point=[reference_longitude, reference_latitude], direction=[1, optimal_traj_tangent_slope]) # line tangent to the trajectory
        optimal_traj_normal_line = Line(point=[reference_longitude, reference_latitude], direction=[1, -1/optimal_traj_tangent_slope]) # line perpendicular to the trajectory

        # Iterate over files in `dnt_trajectories` directory
        for filename in os.listdir(CSV_output_dir):
            df = pd.read_csv(f"{CSV_output_dir}/{filename}") # copy CSV data into pandas dataframe

            test_idx = (df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step nearest to the desired time value

            test_longitude = df["longitude"][test_idx] # [m] longitude coordinate of test location
            test_latitude = df["latitude"][test_idx] # [m] latitude coordinate of test location
            test_altitude = df["altitude"][test_idx] # [m] z-coordinate (altitude) of test location

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

output_file = open(f"{date_dir}/DNT.csv", 'w', newline="") # output CSV file containing DNT information
writer = csv.writer(output_file) # CSV writer for output file containing DNT information
writer.writerow(["#","t_i","t_f","m_1","b_1","m_2","b_2","h_high","h_low"]) # write header row of output CSV file containing DNT information

# Loop to Output the DNT Information CSV
for idx in range(len(dnt_left_boundary_xs) - 1): # for each point along the left edge of the DNT
    discretization_number = idx + 1 # count of the discretization
    t_i = time_vector[idx] # lower time bound to which this discretization applies
    t_f = time_vector[idx + 1] # upper time bound to which this discretization applies
    m_1 = (dnt_left_boundary_ys[idx + 1] - dnt_left_boundary_ys[idx])/(dnt_left_boundary_xs[idx + 1] - dnt_left_boundary_xs[idx]) # slope of the line bounding the left edge of the DNT for this discretization ("left" defined as the launch operator standing at the inertial origin and looking downrange)
    b_1 = dnt_left_boundary_ys[idx] - m_1*dnt_left_boundary_xs[idx] # y-intercept of the line bounding the left edge of the DNT for this discretization
    m_2 = (dnt_right_boundary_ys[idx + 1] - dnt_right_boundary_ys[idx])/(dnt_right_boundary_xs[idx + 1] - dnt_right_boundary_xs[idx]) # slope of the line bounding the right edge of the DNT for this discretization ("right" defined the same as "left" previously)
    b_2 = dnt_left_boundary_ys[idx] - m_2*dnt_left_boundary_xs[idx] # y-intercept of the line bounding the right edge of the DNT for this discretization ("right" defined the same as "left" 
    h_high = dnt_upper_altitudes[idx] # upper alitutde limit of the current discretization
    h_low = dnt_lower_altitudes[idx] # lower alitutde limit of the current discretization
    writer.writerow([discretization_number, round(t_i, 1), round(t_f, 1), round(m_1, 3), round(b_1, 3), round(m_2, 3), round(b_2, 3), round(h_high, 3), round(h_low, 3)]) # write header row of output CSV file containing DNT information

output_file.close() # close the file

launch_area_ax.plot(dnt_left_boundary_xs, dnt_left_boundary_ys, 'b.-')
launch_area_ax.plot(dnt_right_boundary_xs, dnt_right_boundary_ys, 'g.-')
launch_area_ax.legend(loc="best")
plt.savefig(f"{figures_output_dir}/DNT.png")
plt.show()