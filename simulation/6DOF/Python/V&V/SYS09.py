# Libraries
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rocketpy import Flight
import sys
sys.path.append("../")
from setup import launch_area_ax, launch_rail_length, launch_site, DART_rocket

def check_dnt(current_long: int, current_lat: int, DNT_FILE_PATH: str, abort_counter: int, abort_threshold: int):
    '''
    Function to Check the Current Rocket Position Against the Domain of Nominal Trajectories and Determine the Necessity of an In-Flight Abort

    Inputs
    ------
    `current_long`: Current longitude of the rocket's position
    `current_lat`: Current latitude of the rocket's position
    `DNT_FILE_PATH`: Path to the CSV file containing the DNT information
    `abort_counter`: Counter to track the number of consecutive trajectory positions that have exited the DNT
    `abort_threshold`: Number of consecutive trajectory positions that must exit the DNT to trigger an abort

    Outputs
    -------
    `abort`: Boolean indicating the necessity of an abort (`True` if abort necessary, `False` otherwise)
    '''

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

dnt_ts = dnt_points_df["t"].tolist() # list of timesteps defining the DNT
dnt_x_1s = dnt_points_df["x_1"].tolist() # list of longitude coordinates of the DNT left boundary
dnt_y_1s = dnt_points_df["y_1"].tolist() # list of latitude coordinates of the DNT left boundary
dnt_x_2s = dnt_points_df["x_2"].tolist() # list of longitude coordinates of the DNT right boundary
dnt_y_2s = dnt_points_df["y_2"].tolist() # list of latitude coordinates of the DNT right boundary

dnt_average_longs = [] # list to track the average longitude coordinate of each DNT discretization
dnt_average_lats = [] # list to track the average latitude coordinate of each DNT discretization
for idx in range(len(dnt_x_1s) - 1):
    new_longitude_average = (dnt_x_1s[idx] + dnt_x_1s[idx + 1] + dnt_x_2s[idx] + dnt_x_2s[idx + 1])/4 # average longitude coordinate of all boundary coordinates applicable to the discretization
    dnt_average_longs.append(new_longitude_average)
    new_latitude_average = (dnt_y_1s[idx] + dnt_y_1s[idx + 1] + dnt_y_2s[idx] + dnt_y_2s[idx + 1])/4 # average latitude coordinate of all boundary coordinates applicable to the discretization
    dnt_average_lats.append(new_latitude_average)

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
    abort_counts = 0 # counter to track the number of trajectory positions that exited the DNT
    abort_threshold = 3 # number of trajectory positions that must exit the DNT to trigger an abort
    # Iterate through Solution Steps in the Simulated Trajectory
    for solution_step in test_flight.solution:
        time_step_time = solution_step[0] # [s] time of the current solution step
        time_step_long = test_flight.longitude(time_step_time) # [deg] longitude position of the trajectory at the current time step
        time_step_lat = test_flight.latitude(time_step_time) # [deg] latitude position of the trajectory at the current time step

        if (time_step_time > dnt_ts[-1]): # if the current time step exceeds the DNT limits
            current_dnt_df_row = dnt_df.tail(1) # use the last row of the DNT dataframe
            dnt_points_indices = [len(dnt_ts) - 2, len(dnt_ts) - 1] # use the last two indices
        else:
            # current_dnt_df_row = dnt_df.loc[(dnt_df["t_i"] <= time_step_time) & (dnt_df["t_f"] > time_step_time)] # row of the DNT dataframe relevant to the current time step
            # dnt_points_df_row_indices = dnt_points_df.iloc[(dnt_points_df["t"] - time_step_time).abs().argsort()[:2]].index.tolist() # indices of the DNT points DataFrame 2 most relevant row to the current time step
            # current_position_distances = np.array([math.sqrt((time_step_long - dnt_average_longs[idx])**2 + (time_step_lat - dnt_average_lats[idx])**2) for idx in range(len(dnt_average_longs))]) # [deg] Pythagorean distance between current position and each DNT discretization average coordinate
            current_position_times = [(time_step_time - dnt_t_test) for dnt_t_test in dnt_ts]
            current_position_lower_times = list(filter(lambda time_since: time_since >= 0, current_position_times))
            dnt_optimal_index = len(current_position_lower_times) - 1 # get the index of the most recent DNT lower time bound
            dnt_points_indices = [dnt_optimal_index, dnt_optimal_index + 1] # indices to use for DNT boundary points
            # dnt_longs_points_indices = np.array(([abs(time_step_long - dnt_long_test) for dnt_long_test in dnt_average_longs])).argsort()
            # dnt_lats_points_indices = np.array(([abs(time_step_lat - dnt_lat_test) for dnt_lat_test in dnt_average_lats])).argsort()
            # current_dnt_points_df_rows = dnt_points_df.iloc[min(dnt_points_df_row_indices):max(dnt_points_df_row_indices) + 1] # rows of the DNT points dataframe relevant to the current time step

        # m_1 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("m_1")] # slope of the left boundary of the DNT
        # b_1 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("b_1")] # y-intercept of the line making the left boundary of the DNT
        # m_2 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("m_2")] # slope of the right boundary of the DNT
        # b_2 = current_dnt_df_row.iloc[0, dnt_df.columns.get_loc("b_2")] # y-intercept of the line making the right boundary of the DNT

        left_x_1 = dnt_x_1s[min(dnt_points_indices)] # closet left DNT boundary point x-coordinate
        left_y_1 = dnt_y_1s[min(dnt_points_indices)] # closet left DNT boundary point y-coordinate
        left_x_2 = dnt_x_1s[max(dnt_points_indices)] # second-closet left DNT boundary point x-coordinate
        left_y_2 = dnt_y_1s[max(dnt_points_indices)] # second-closet left DNT boundary point y-coordinate
        # print(f"Left: {left_x_1, left_x_2, left_y_1, left_y_2}")
        right_x_1 = dnt_x_2s[min(dnt_points_indices)] # closet right DNT boundary point x-coordinate
        right_y_1 = dnt_y_2s[min(dnt_points_indices)] # closet right DNT boundary point y-coordinate
        right_x_2 = dnt_x_2s[max(dnt_points_indices)] # second-closet right DNT boundary point x-coordinate
        right_y_2 = dnt_y_2s[max(dnt_points_indices)] # second-closet right DNT boundary point y-coordinate
        # print(f"Right: {right_x_1, right_x_2, right_y_1, right_y_2}")

        if (left_x_1 == right_x_1 and left_y_1 == right_y_1): # the DNT begins at a point, so the left and right boundaries collide (results in divide by zero error)
            print("Special Case 1")
            continue # continue to the next time step
        elif (left_x_1 == left_x_2 and left_y_1 == left_y_2): # TBR, sometimes the DNT has consecutive points with the same coordinates, still unsure why
            print("Special Case 2")
            continue # continue to the next time step
        elif (right_x_1 == right_x_2 and right_y_1 == right_y_2): # TBR, sometimes the DNT has consecutive points with the same coordinates, still unsure why
            print("Special Case 3")
            continue # continue to the next time step

        '''
        Angle Check 1
        -------------
        Vector 1: Vector from (left_x_1, left_y_1) to (right_x_1, right_y_1)
        Vector 2: Vector from (left_x_1, left_y_1) to (left_x_2, left_y_2)
        Vector 3: Vector from (left_x_1, left_y_1) to the current rocket position
        '''
        vec_1 = [right_x_1 - left_x_1, right_y_1 - left_y_1] # vector from the first relevant left boundary point to the first relevent right boundary point
        vec_1_mag = math.sqrt(sum(elem**2 for elem in vec_1)) # magnitude of the above vector
        vec_2 = [left_x_2 - left_x_1, left_y_2 - left_y_1] # vector from the first relevant left boundary point to the second relevant left boundary point
        vec_2_mag = math.sqrt(sum(elem**2 for elem in vec_2)) # magnitude of the above vector
        vec_3 = [time_step_long - left_x_1, time_step_lat - left_y_1] # vector from the first relevent left boundary point to the current postion
        vec_3_mag = math.sqrt(sum(elem**2 for elem in vec_3)) # magnitude of the above vector

        vec1_vec2_dot = (vec_1[0] * vec_2[0]) + (vec_1[1] * vec_2[1]) # dot product of vec_1 and vec_2
        try:
            angle_vec1_vec2 = math.acos(vec1_vec2_dot/(vec_1_mag * vec_2_mag)) # [rad] angle between vec_1 and vec_2 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec1_vec2 = math.acos(round(vec1_vec2_dot/(vec_1_mag * vec_2_mag))) # [rad] angle between vec_1 and vec_2 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        vec1_vec3_dot = (vec_1[0] * vec_3[0]) + (vec_1[1] * vec_3[1]) # dot product of vec_1 and vec_3
        try:
            angle_vec1_vec3 = math.acos(vec1_vec3_dot/(vec_1_mag * vec_3_mag)) # [rad] angle between vec_1 and vec_3 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec1_vec3 = math.acos(vec1_vec3_dot/(vec_1_mag * vec_3_mag)) # [rad] angle between vec_1 and vec_3 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        check_1 = bool(angle_vec1_vec2 > angle_vec1_vec3) # desire the angle between vec_1 and vec_2 to be greater than the angle between vec_1 and vec_3

        '''
        Angle Check 2
        -------------
        Vector 4: Vector from (left_x_2, left_y_2) to (left_x_1, left_y_1)
        Vector 5: Vector from (left_x_2, left_y_2) to (right_x_2, right_y_2)
        Vector 6: Vector from (left_x_2, left_y_2) to the current rocket position
        '''
        vec_4 = [left_x_1 - left_x_2, left_y_1 - left_y_2] # vector from the second relevant left boundary point to the first relevent left boundary point
        vec_4_mag = math.sqrt(sum(elem**2 for elem in vec_4)) # magnitude of the above vector
        vec_5 = [right_x_2 - left_x_2, right_y_2 - left_y_2] # vector from the second relevant left boundary point to the second relevant right boundary point
        vec_5_mag = math.sqrt(sum(elem**2 for elem in vec_5)) # magnitude of the above vector
        vec_6 = [time_step_long - left_x_2, time_step_lat - left_y_2] # vector from the second relevent left boundary point to the current postion
        vec_6_mag = math.sqrt(sum(elem**2 for elem in vec_6)) # magnitude of the above vector

        vec4_vec5_dot = (vec_4[0] * vec_5[0]) + (vec_4[1] * vec_5[1]) # dot product of vec_4 and vec_5
        try:
            angle_vec4_vec5 = math.acos(vec4_vec5_dot/(vec_4_mag * vec_5_mag)) # [rad] angle between vec_4 and vec_5 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec4_vec5 = math.acos(round(vec4_vec5_dot/(vec_4_mag * vec_5_mag))) # [rad] angle between vec_4 and vec_5 (tail-to-tail), round to either 1 or -1 if roundoff error occurs
        
        vec4_vec6_dot = (vec_4[0] * vec_6[0]) + (vec_4[1] * vec_6[1]) # dot product of vec_4 and vec_6
        try:
            angle_vec4_vec6 = math.acos(vec4_vec6_dot/(vec_4_mag * vec_6_mag)) # [rad] angle between vec_4 and vec_6 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec4_vec6 = math.acos(round(vec4_vec6_dot/(vec_4_mag * vec_6_mag))) # [rad] angle between vec_4 and vec_6 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        check_2 = bool(angle_vec4_vec5 > angle_vec4_vec6) # desire the angle between vec_4 and vec_5 to be greater than the angle between vec_4 and vec_6

        '''
        Angle Check 3
        -------------
        Vector 7: Vector from (right_x_2, right_y_2) to (left_x_2, left_y_2)
        Vector 8: Vector from (right_x_2, right_y_2) to (right_x_1, right_y_1)
        Vector 9: Vector from (right_x_2, right_y_2) to the current rocket position
        '''
        vec_7 = [left_x_2 - right_x_2, left_y_2 - right_y_2] # vector from the second relevant right boundary point to the second relevent left boundary point
        vec_7_mag = math.sqrt(sum(elem**2 for elem in vec_7)) # magnitude of the above vector
        vec_8 = [right_x_1 - right_x_2, right_y_1 - right_y_2] # vector from the second relevant right boundary point to the first relevant right boundary point
        vec_8_mag = math.sqrt(sum(elem**2 for elem in vec_8)) # magnitude of the above vector
        vec_9 = [time_step_long - right_x_2, time_step_lat - right_y_2] # vector from the second relevent right boundary point to the current postion
        vec_9_mag = math.sqrt(sum(elem**2 for elem in vec_9)) # magnitude of the above vector

        vec7_vec8_dot = (vec_7[0] * vec_8[0]) + (vec_7[1] * vec_8[1]) # dot product of vec_7 and vec_8
        try:
            angle_vec7_vec8 = math.acos(vec7_vec8_dot/(vec_7_mag * vec_8_mag)) # [rad] angle between vec_7 and vec_8 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec7_vec8 = math.acos(round(vec7_vec8_dot/(vec_7_mag * vec_8_mag))) # [rad] angle between vec_7 and vec_8 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        vec7_vec9_dot = (vec_7[0] * vec_9[0]) + (vec_7[1] * vec_9[1]) # dot product of vec_7 and vec_9
        try:
            angle_vec7_vec9 = math.acos(vec7_vec9_dot/(vec_7_mag * vec_9_mag)) # [rad] angle between vec_7 and vec_9 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec7_vec9 = math.acos(round(vec7_vec9_dot/(vec_7_mag * vec_9_mag))) # [rad] angle between vec_7 and vec_9 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        check_3 = bool(angle_vec7_vec8 > angle_vec7_vec9) # desire the angle between vec_7 and vec_8 to be greater than the angle between vec_7 and vec_9

        '''
        Angle Check 4
        -------------
        Vector 10: Vector from (right_x_1, right_y_1) to (right_x_2, right_y_2)
        Vector 11: Vector from (right_x_1, right_y_1) to (left_x_1, left_y_1)
        Vector 12: Vector from (right_x_1, right_y_1) to the current rocket position
        '''
        vec_10 = [right_x_2 - right_x_1, right_y_2 - right_y_1] # vector from the first relevant right boundary point to the second relevent right boundary point
        vec_10_mag = math.sqrt(sum(elem**2 for elem in vec_10)) # magnitude of the above vector
        vec_11 = [left_x_1 - right_x_1, left_y_1 - right_y_1] # vector from the first relevant right boundary point to the first relevant left boundary point
        vec_11_mag = math.sqrt(sum(elem**2 for elem in vec_11)) # magnitude of the above vector
        vec_12 = [time_step_long - right_x_1, time_step_lat - right_y_1] # vector from the first relevent right boundary point to the current postion
        vec_12_mag = math.sqrt(sum(elem**2 for elem in vec_12)) # magnitude of the above vector

        vec10_vec11_dot = (vec_10[0] * vec_11[0]) + (vec_10[1] * vec_11[1]) # dot product of vec_10 and vec_11
        try:
            angle_vec10_vec11 = math.acos(vec10_vec11_dot/(vec_10_mag * vec_11_mag)) # [rad] angle between vec_10 and vec_11 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec10_vec11 = math.acos(round(vec10_vec11_dot/(vec_10_mag * vec_11_mag))) # [rad] angle between vec_10 and vec_11 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        vec10_vec12_dot = (vec_10[0] * vec_12[0]) + (vec_10[1] * vec_12[1]) # dot product of vec_10 and vec_12
        try:
            angle_vec10_vec12 = math.acos(vec10_vec12_dot/(vec_10_mag * vec_12_mag)) # [rad] angle between vec_10 and vec_11 (tail-to-tail)
        except (ValueError): # "math domain error" caused by roundoff error (i.e., -1.0000000000000002)
            angle_vec10_vec12 = math.acos(round(vec10_vec12_dot/(vec_10_mag * vec_12_mag))) # [rad] angle between vec_10 and vec_11 (tail-to-tail), round to either 1 or -1 if roundoff error occurs

        check_4 = bool(angle_vec10_vec11 > angle_vec10_vec12) # desire the angle between vec_10 and vec_11 to be greater than the angle between vec_10 and vec_12

        # print(time_step_time)
        # print(vec_2)
        # print(f"{vec_1_mag, vec_2_mag, vec_3_mag}\n")

        # print(f"Angle_1: {vec1_vec2_dot/(vec_1_mag * vec_2_mag)}")

        # m_1 = (left_y_2 - left_y_1)/(left_x_2 - left_x_1) # slope of the DNT left boundary at the current time step
        # b_1 = left_y_1 - m_1*left_x_1 # y-intercept of the line bounding the left edge of the DNT at the current time step
        # m_2 = (right_y_2 - right_y_1)/(right_x_2 - right_x_1) # slope of the DNT right boundary at the current time step
        # b_2 = right_y_1 - m_2*right_x_1 # y-intercept of the line bounding the right edge of the DNT at the current time step

        # launch_area_ax.plot([left_x_1, left_x_2], [m_1*left_x_1 + b_1, m_1*left_x_2 + b_1], 'b.-') # plot left boundary line segments
        launch_area_ax.plot([left_x_1, left_x_2], [left_y_1, left_y_2], 'b.-') # plot left boundary line segments
        launch_area_ax.plot([right_x_1, right_x_2], [right_y_1, right_y_2], 'g.-') # plot right boundary line segments

        # Check Left Boundary
        # comparable_left_time_step_lat = m_1 * time_step_long + b_1
        # if (m_1 > 0): # the DNT left boundary has positive slope
        # if (left_x_1 < left_x_2): # the DNT left boundary is North of the right boundary
        #     left_bool = bool(time_step_lat < comparable_left_time_step_lat) # desire to be BELOW the left boundary
        #     flag1 = True
        # else: # the DNT left boundary is South of the right boundary
        #     left_bool = bool(time_step_lat > comparable_left_time_step_lat) # desire to be ABOVE the left boundary
        #     flag1 = False
        # elif (m_1 < 0): # the DNT left boundary has negative slope
        #     if (left_x_1 < left_x_2): # the DNT left boundary is North of the right boundary
        #         left_bool = bool(time_step_lat < comparable_left_time_step_lat) # desire to be BELOW the left boundary
        #     elif (left_x_1 > left_x_2): # the DNT left boundary is South of the right boundary
        #         left_bool = bool(time_step_lat > comparable_left_time_step_lat) # desire to be ABOVE the left boundary
        # Check Right Boundary
        # comparable_right_time_step_lat = m_2 * time_step_long + b_2
        # if (right_x_1 < right_x_2): # the DNT right boundary is South of the left boundary
        #     right_bool = bool(time_step_lat > comparable_right_time_step_lat) # desire to be ABOVE the right boundary
        # elif (right_x_1 > right_x_2): # the DNT right boundary is North of the left boundary
        #     right_bool = bool(time_step_lat < comparable_right_time_step_lat) # desire to be BELOW the right boundary
        # '''
        # The logic does not require knowledge of which direction the rocket is flying (i.e., it does not need to know whether the left boundary is North of the right boundary, or vice versa).
        # - If the bools are the same value: the rocket must be within the DNT
        #     - If the correct orientation was assumed, both bools will be `True` when the rocket is within the DNT
        #     - If the incorrect orientation was assumed, both bools will be `False` when the rocket is within the DNT
        # - If the bools are not the same value: the rocket must have exited the DNT
        #     - If the correct orientation was assumed, one bool will be `False` when the rocket exits the DNT, while the other will remain `True`
        #     - If the incorrect orientation was assumed, one bool will be `True` when the rocket exits the DNT, while the other will remain `False`
        # '''
        if (check_1 and check_2 and check_3 and check_4): # the rocket is within the DNT
            abort_counts = 0 # reset the abort counter
            abort_color = 'g'
            success_counter += 1
        else: # the rocket has exited the DNT
            abort_counts += 1 # increment the abort counter
            # print(time_step_time, flag1, left_bool) # `left_bool` registering as False, but points stil showing green?
            # launch_area_ax.plot([time_step_long, time_step_long], [time_step_lat, comparable_left_time_step_lat], 'k.-')
            # launch_area_ax.plot([time_step_long, time_step_long], [time_step_lat, comparable_right_time_step_lat], 'm.-')
            # print(f"left_bool: {time_step_lat}, {comparable_left_time_step_lat}")
            # print(f"right_bool: {time_step_lat}, {comparable_right_time_step_lat}")
            if (abort_counts >= abort_threshold):
                abort_color = 'r' # plot in red if the trajectory exits the DNT
                print("ABORT TRIGGERED")

        # if (abs(time_step_time - 1) < 0.1):
        launch_area_ax.plot(time_step_long, time_step_lat, '.', color=abort_color)
        # solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
        # launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), color=abort_color)

success_rate = success_counter/num_trajectories # percentage of trajectories that stayed within the DNT
launch_area_ax.set_title(f"SYS.09 Verification")

plt.show()