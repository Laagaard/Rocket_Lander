# Libraries
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import launch_area_ax, launch_rail_length, launch_site, DART_rocket

try:
    launch_date = sys.argv[1] # MM-DD-YYYY format required
    launch_hour = sys.argv[2] # HH format required (24-hour)
except (IndexError):
    print("NEED DATE AND TIME SPECIFIED ON THE COMMAND LINE: MM-DD-YYYY HH")

date_path = f"../DNT/Results/{launch_date}" # path to the folder with the DNT information for the date of interest
date_time_path = f"../DNT/Results/{launch_date}/{launch_hour}" # path to the folder with the DNT information for the date/time of interest
dnt_file_path = f"{date_time_path}/DNT.csv" # path to the DNT CSV file
dnt_df = pd.read_csv(dnt_file_path) # DNT CSV as a pandas df

dnt_points_file_path = f"../DNT/Results/{launch_date}/{launch_hour}/DNT_points.csv" # path to the CSV file containing the points of the DNT boundaries
dnt_points_df = pd.read_csv(dnt_points_file_path) # Points of the DNT boundaries as a pandas df
# launch_area_ax.plot(dnt_points_df["x_1"], dnt_points_df["y_1"], 'b.-') # plot left edge of the DNT
# launch_area_ax.plot(dnt_points_df["x_2"], dnt_points_df["y_2"], 'g.-') # plot right edge of the DNT

optimal_launch_information_file_path = f"{date_path}/{launch_date}.csv" # path to CSV file with optimal launch information for the date of interest
optimal_launch_information_df = pd.read_csv(optimal_launch_information_file_path) # optimal launch information for the date of interest as a pandas df
optimal_inclination = optimal_launch_information_df["Inclination"][0] # optimal launch inclination, TBR, not robust for multiple times per day
optimal_heading = optimal_launch_information_df["Heading"][0] # optimal launch heading, TBR, not robust for multiple times per day

num_trajectories = 1 # number of trajectories to simulate
success_counter = 0 # counter for the number of trajectories that stay within the DNT
for elem in range(num_trajectories):
    launch_inclination = np.random.normal(loc=optimal_inclination, scale=0.25, size=1)[0] # [deg] draw launch inclination from normal distribution centered at optimal inclination
    launch_heading = np.random.normal(loc=optimal_heading, scale=0.25, size=1)[0] # [deg] draw launch heading from normal distribution centered at optimal heading
    print(f"Iteration: {elem}, Inclination: {round(launch_inclination, 2)} deg, Heading: {round(launch_heading, 2)} deg")

    test_flight = Flight(
        rocket=DART_rocket,
        environment=launch_site,
        rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
        inclination=optimal_inclination, # [deg] rail inclination relative to the ground
        heading=optimal_heading, # [deg] heading angle relative to North (East = 90)
        time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
    ) # run trajectory simulation

    # abort_color = 'g' # plot in green if the trajectory remains within the DNT
    # Iterate through Solution Steps in the Simulated Trajectory
    for solution_step in test_flight.solution:
        time_step_time = solution_step[0] # [s] time of the current solution step
        time_step_long = test_flight.longitude(time_step_time) # [deg] longitude position of the trajectory at the current time step
        time_step_lat = test_flight.latitude(time_step_time) # [deg] latitude position of the trajectory at the current time step

        if all(time_step_time > dnt_df["t_f"]): # if the current time step exceeds the DNT limits
            current_dnt_df_row = dnt_df.tail(1) # use the last row of the DNT dataframe
            current_dnt_points_df_rows = dnt_points_df.tail(2) # use the last rows of the DNT points dataframe
        else:
            current_dnt_df_row = dnt_df.loc[(dnt_df["t_i"] <= time_step_time) & (dnt_df["t_f"] > time_step_time)] # row of the DNT dataframe relevant to the current time step
            first_dnt_points_df_row_index = dnt_points_df.iloc[(dnt_points_df["t"] - time_step_time).abs().argsort()[:1]].index.tolist()[0] # index of the DNT points df most relevant row to the current time step
            current_dnt_points_df_rows = dnt_points_df.iloc[first_dnt_points_df_row_index:first_dnt_points_df_row_index+2] # rows of the DNT points dataframe relevant to the current time step

        # m_1 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("m_1")] # slope of the left boundary of the DNT
        # b_1 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("b_1")] # y-intercept of the line making the left boundary of the DNT
        # m_2 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("m_2")] # slope of the right boundary of the DNT
        # b_2 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("b_2")] # y-intercept of the line making the right boundary of the DNT

        left_x_1 = current_dnt_points_df_rows.iloc[0, dnt_points_df.columns.get_loc("x_1")] # closet left DNT boundary point x-coordinate
        left_y_1 = current_dnt_points_df_rows.iloc[0, dnt_points_df.columns.get_loc("y_1")] # closet left DNT boundary point y-coordinate
        left_x_2 = current_dnt_points_df_rows.iloc[1, dnt_points_df.columns.get_loc("x_1")] # second-closet left DNT boundary point x-coordinate
        left_y_2 = current_dnt_points_df_rows.iloc[1, dnt_points_df.columns.get_loc("y_1")] # second-closet left DNT boundary point y-coordinate
        right_x_1 = current_dnt_points_df_rows.iloc[0, dnt_points_df.columns.get_loc("x_2")] # closet right DNT boundary point x-coordinate
        right_y_1 = current_dnt_points_df_rows.iloc[0, dnt_points_df.columns.get_loc("y_2")] # closet right DNT boundary point y-coordinate
        right_x_2 = current_dnt_points_df_rows.iloc[1, dnt_points_df.columns.get_loc("x_2")] # second-closet right DNT boundary point x-coordinate
        right_y_2 = current_dnt_points_df_rows.iloc[1, dnt_points_df.columns.get_loc("y_2")] # second-closet right DNT boundary point y-coordinate

        m_1 = (left_y_2 - left_y_1)/(left_x_2 - left_x_1) # slope of the DNT left boundary at the current time step
        b_1 = left_y_1 - m_1*left_x_1 # y-intercept of the line bounding the left edge of the DNT at the current time step
        m_2 = (right_y_2 - right_y_1)/(right_x_2 - right_x_1) # slope of the DNT right boundary at the current time step
        b_2 = right_y_1 - m_2*right_x_1 # y-intercept of the line bounding the right edge of the DNT at the current time step

        # launch_area_ax.plot([left_x_1, left_x_2], [m_1*left_x_1 + b_1, m_1*left_x_2 + b_1], 'b.-') # plot left boundary line segments
        launch_area_ax.plot([left_x_1, left_x_2], [left_y_1, left_y_2], 'b.-') # plot left boundary line segments
        # launch_area_ax.plot([right_x_1, right_x_2], [m_2*right_x_1 + b_2, m_2*right_x_2 + b_2], 'g.-') # plot right boundary line segments

        # Check Left Boundary
        comparable_left_time_step_lat = m_1 * time_step_long + b_1
        # if (m_1 > 0): # the DNT left boundary has positive slope
        if (left_x_1 < left_x_2): # the DNT left boundary is North of the right boundary
            left_bool = bool(time_step_lat < comparable_left_time_step_lat) # desire to be BELOW the left boundary
            flag1 = True
        else: # the DNT left boundary is South of the right boundary
            left_bool = bool(time_step_lat > comparable_left_time_step_lat) # desire to be ABOVE the left boundary
            flag1 = False
        # elif (m_1 < 0): # the DNT left boundary has negative slope
        #     if (left_x_1 < left_x_2): # the DNT left boundary is North of the right boundary
        #         left_bool = bool(time_step_lat < comparable_left_time_step_lat) # desire to be BELOW the left boundary
        #     elif (left_x_1 > left_x_2): # the DNT left boundary is South of the right boundary
        #         left_bool = bool(time_step_lat > comparable_left_time_step_lat) # desire to be ABOVE the left boundary
        # Check Right Boundary
        comparable_right_time_step_lat = m_2 * time_step_long + b_2
        if (right_x_1 < right_x_2): # the DNT right boundary is South of the left boundary
            right_bool = bool(time_step_lat > comparable_right_time_step_lat) # desire to be ABOVE the right boundary
        elif (right_x_1 > right_x_2): # the DNT right boundary is North of the left boundary
            right_bool = bool(time_step_lat < comparable_right_time_step_lat) # desire to be BELOW the right boundary
        '''
        The logic does not require knowledge of which direction the rocket is flying (i.e., it does not need to know whether the left boundary is North of the right boundary, or vice versa).
        - If the bools are the same value: the rocket must be within the DNT
            - If the correct orientation was assumed, both bools will be `True` when the rocket is within the DNT
            - If the incorrect orientation was assumed, both bools will be `False` when the rocket is within the DNT
        - If the bools are not the same value: the rocket must have exited the DNT
            - If the correct orientation was assumed, one bool will be `False` when the rocket exits the DNT, while the other will remain `True`
            - If the incorrect orientation was assumed, one bool will be `True` when the rocket exits the DNT, while the other will remain `False`
        '''
        if (left_bool): # the rocket is within the DNT
            abort_color = 'g'
            success_counter += 1
        else: # the rocket has exited the DNT
            print(time_step_time, left_bool)
            # launch_area_ax.plot([time_step_long, time_step_long], [time_step_lat, comparable_left_time_step_lat], 'k.-')
            # launch_area_ax.plot([time_step_long, time_step_long], [time_step_lat, comparable_right_time_step_lat], 'm.-')
            # print(f"left_bool: {time_step_lat}, {comparable_left_time_step_lat}")
            # print(f"right_bool: {time_step_lat}, {comparable_right_time_step_lat}")
            abort_color = 'r' # plot in red if the trajectory exits the DNT

        # if (abs(time_step_time - 1) < 0.1):
        launch_area_ax.plot(time_step_long, time_step_lat, '.', color=abort_color)
        # solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
        # launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), color=abort_color)

success_rate = success_counter/num_trajectories # percentage of trajectories that stayed within the DNT
launch_area_ax.set_title(f"SYS.09 Verification")

plt.show()