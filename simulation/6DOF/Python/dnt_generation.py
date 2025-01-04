# Libraries
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from skspatial.objects import Line, Point
from dataset_generation import landing_zone_patch, landing_zone_x, landing_zone_y
from dnt_trajectories import CSV_output_dir

dnt_temporal_resolution = 0.1 # [s] time-step of each DNT discretization
timestep_current_lower_bound = dnt_temporal_resolution # [s] lower time bound of current discretization
dnt_left_boundary_xs = [0] # [m] x-coordinates comprising the left boundary of the DNT
dnt_left_boundary_ys = [0] # [m] y-coordinates comprising the left boundary of the DNT
dnt_right_boundary_xs = [0] # [m] x-coordinates comprising the right boundary of the DNT
dnt_right_boundary_ys = [0] # [m] y-coordinates comprising the right boundary of the DNT
dnt_upper_altitudes = [] # [m] z-coordinate comprising the upper boundary of each DNT discretization
dnt_lower_altitudes = [] # [m] z-coordinate comprising the lower boundary of each DNT discretization
time_vector = [0] # [s] time array

optimal_df = pd.read_csv("optimal_trajectory.csv") # df of optimal trajectory data

test_xs = [] # [m] list to track x-coordinates of test points
test_ys = [] # [m] list to track y-coordinates of test points
test_altitudes = [] # [m] list to track altitudes of test points

# DNT Plot
dnt_fig = plt.figure()
dnt_ax = dnt_fig.add_subplot()
dnt_ax.plot(0, 0, 'k.', label="Launch Site") # plot launch site (i.e., inertial CS origin)
dnt_ax.plot(landing_zone_x, landing_zone_y, color='r', marker="+") # plot center of the landing zone
dnt_ax.add_patch(landing_zone_patch) # plot landing zone patch
dnt_ax.plot(optimal_df["x_pos"], optimal_df["y_pos"], 'b', label="Optimal Trajectory")
dnt_ax.set_xlabel("X - East [m]")
dnt_ax.set_ylabel("Y - North [m]")
dnt_ax.axis('equal') # set axis limits equivalent
dnt_ax.grid(which="major", axis="both")
dnt_ax.set_title(f"Domain of Nominal Trajectories \n(Time Resolution: {dnt_temporal_resolution} [s])")

# Loop until the lower time bound is greater than or equal to the optimal trajectory flight time
while (timestep_current_lower_bound <= optimal_df["Time"].iloc[-1]):
    print(f"Current Time: {timestep_current_lower_bound}")
    time_color = (np.random.uniform(0, 1), np.random.uniform(0, 1), np.random.uniform(0, 1)) # generate a new plotting color
    optimal_idx = (optimal_df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step of optimal trajectory nearest to the desired time value
    reference_x = optimal_df["x_pos"][optimal_idx] # [m] x-coordinate of reference location
    reference_y = optimal_df["y_pos"][optimal_idx] # [m] y-coordinate of reference locations
    optimal_vx = optimal_df["x_vel"][optimal_idx] # [m/s] x_velocity of the optimal trajectory nearest the time step of interest
    optimal_vy = optimal_df["y_vel"][optimal_idx] # [m/s] y_velocity of the optimal trajectory nearest the time step of interest
    optimal_traj_tangent_slope = optimal_vy/optimal_vx # slope of tangent line to the optimal trajectory
    optimal_traj_tangent_line = Line(point=[reference_x, reference_y], direction=[1, optimal_traj_tangent_slope]) # line tangent to the trajectory
    optimal_traj_normal_line = Line(point=[reference_x, reference_y], direction=[1, -1/optimal_traj_tangent_slope]) # line perpendicular to the trajectory

    # Iterate over files in Monte Carlo simulations directory
    for filename in os.listdir(CSV_output_dir):
        df = pd.read_csv(f"{CSV_output_dir}/{filename}") # copy CSV data into pandas dataframe

        test_idx = (df["Time"] - timestep_current_lower_bound).abs().idxmin() # index of solution step nearest to the desired time value

        test_x = df["x_pos"][test_idx] # [m] x-coordinate of test location
        test_y = df["y_pos"][test_idx] # [m] y-coordinate of test location
        test_altitude = df["z_pos"][test_idx] # [m] z-coordinate (altitude) of test location

        # Append to Tracking Lists
        test_xs.append(test_x)
        test_ys.append(test_y)
        test_altitudes.append(test_altitude)

    projected_test_points_x = [] # list to track the x-coordinate of each projected point
    projected_test_points_y = [] # list to track the y-coordinate of each projected point
    projected_test_points_distance = [] # list to track the distances of each projected point to the reference point
    projected_test_points_side = [] # list to track which side of the optimal trajectory each projected point is on
    for idx in range(len(test_xs)):
        test_point_object = Point([test_xs[idx], test_ys[idx]]) # create scipy point object for the test point
        test_point_object_projected = optimal_traj_normal_line.project_point(test_point_object) # project the test point onto the perpendicular-to-the-trajectory line

        projected_test_point_x = test_point_object_projected[0] # [m] x-coordinate of the projected test point
        projected_test_point_y = test_point_object_projected[1] # [m] y-coordinate of the projected test point
        projected_test_point_distance = math.sqrt((projected_test_point_x - reference_x)**2 + (projected_test_point_y - reference_y)**2) # [m] distance between the projected test point and the reference point

        projected_test_points_x.append(projected_test_point_x)
        projected_test_points_y.append(projected_test_point_y)
        projected_test_points_distance.append(projected_test_point_distance)

        # Determine the side of the optimal trajectory the projected test point lies on
        if (optimal_vx > 0): # the optimal trajectory x_velocity is positive
            if (projected_test_point_y > reference_y): # the projected test point is above the reference point
                projected_test_points_side.append(0) # the projected test point lies to the LEFT of the optimal trajectory
            else:
                projected_test_points_side.append(1) # the projected test point lies to the RIGHT of the optimal trajectory
        else: # the optimal trajectory x_velocity is negative
            if (projected_test_point_y > reference_y): # the projected test point is above the reference point
                projected_test_points_side.append(1) # the projected test point lies to the RIGHT of the optimal trajectory
            else:
                projected_test_points_side.append(0) # the projected test point lies to the LEFT of the optimal trajectory

    indices_of_left = [idx for idx, elem in enumerate(projected_test_points_side) if elem == 0] # get indices of projected test points on the LEFT of the optimal trajectory
    left_point_xs = [projected_test_points_x[idx] for idx in indices_of_left] # [m] x-coordinatles of each projected point on the LEFT of the optimal trajectory
    left_point_ys = [projected_test_points_y[idx] for idx in indices_of_left] # [m] y-coordinatles of each projected point on the LEFT of the optimal trajectory
    left_point_distances = [projected_test_points_distance[idx] for idx in indices_of_left] # [m] distance from each point on the LEFT of the optimal trajectory to the reference point
    left_bound_index = left_point_distances.index(max(left_point_distances)) # index of the point with the greatest distance
    dnt_left_boundary_xs.append(left_point_xs[left_bound_index])
    dnt_left_boundary_ys.append(left_point_ys[left_bound_index])

    indices_of_right = [idx for idx, elem in enumerate(projected_test_points_side) if elem == 1] # get indices of projected test points on the RIGHT of the optimal trajectory
    right_point_xs = [projected_test_points_x[idx] for idx in indices_of_right] # [m] x-coordinatles of each projected point on the RIGHT of the optimal trajectory
    right_point_ys = [projected_test_points_y[idx] for idx in indices_of_right] # [m] y-coordinatles of each projected point on the RIGHT of the optimal trajectory
    right_point_distances = [projected_test_points_distance[idx] for idx in indices_of_right] # [m] distance from each point on the RIGHT of the optimal trajectory to the reference point
    right_bound_index = right_point_distances.index(max(right_point_distances)) # index of the point with the greatest distance
    dnt_right_boundary_xs.append(right_point_xs[right_bound_index])
    dnt_right_boundary_ys.append(right_point_ys[right_bound_index])

    dnt_lower_altitudes.append(min(test_altitudes))
    dnt_upper_altitudes.append(max(test_altitudes))

    # Reset all tracking lists
    test_xs = []
    test_ys = []
    test_altitudes = []
    projected_test_points_x = []
    projected_test_points_y = []
    projected_test_points_distance = []
    projected_test_points_side = []

    time_vector.append(timestep_current_lower_bound) # append current lower time bound to the time array
    timestep_current_lower_bound += dnt_temporal_resolution # increment current time step

output_file = open("DNT.csv", 'w', newline="") # output CSV file containing DNT information
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

dnt_ax.plot(dnt_left_boundary_xs, dnt_left_boundary_ys, 'b.-')
dnt_ax.plot(dnt_right_boundary_xs, dnt_right_boundary_ys, 'g.-')
dnt_ax.legend(loc="best")
plt.show()