# Libraries
import math
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rocketpy import Flight
from skimage.measure import EllipseModel
import sys
sys.path.append("../")
from setup import command_line_args, DART_rocket_1, date_dir_with_time, launch_area_ax, launch_rail_length, launch_site, all_landing_zone_perimeters

def check_dnt(DNT_FILE_PATH: str, current_time: float, current_long: float, current_lat: float, abort_counts: int, abort_count_threshold = 3):
    '''
    Method to Check the Current Rocket Position Against the Domain of Nominal Trajectories and Determine the Necessity of an In-Flight Abort

    Parameters
    ------
    `DNT_FILE_PATH`: Path to the CSV file containing the DNT information
    `current_time`: Current flight time [s]
    `current_long`: Current longitude of the rocket's position [deg]
    `current_lat`: Current latitude of the rocket's position [deg]
    `abort_counts`: Counter to track the number of consecutive trajectory positions that have exited the DNT, increments by 1 if an abort seems desired
    `abort_count_threshold`: Number of consecutive trajectory positions that must exit the DNT to trigger an abort (default = 3)

    Returns
    -------
    `abort`: Boolean indicating the necessity of an abort (`True` if abort necessary, `False` otherwise)
    `abort_counts`: Updated value of the `abort_counts` input
    '''

    dnt_points_df = pd.read_csv(DNT_FILE_PATH) # Points of the DNT boundaries as a pandas df
    dnt_ts = dnt_points_df["t"].tolist() # list of timesteps defining the DNT
    dnt_x_1s = dnt_points_df["x_1"].tolist() # list of longitude coordinates of the DNT left boundary
    dnt_y_1s = dnt_points_df["y_1"].tolist() # list of latitude coordinates of the DNT left boundary
    dnt_x_2s = dnt_points_df["x_2"].tolist() # list of longitude coordinates of the DNT right boundary
    dnt_y_2s = dnt_points_df["y_2"].tolist() # list of latitude coordinates of the DNT right boundary

    dnt_average_longs = [] # list to track the average longitude coordinate of each DNT discretization
    dnt_average_lats = [] # list to track the average latitude coordinate of each DNT discretization

    positive_time = list(filter(lambda time: time > 0, dnt_ts)) # list of DNT time steps greater than 0
    if (not current_time >= min(positive_time)): # if the current time is not greater than or equal to the first positive DNT time step
        abort_counts = 0 # reset the abort counter
        return False, abort_counts # too early to reasonably determine if abort is required

    check_1s = [] # list to track all `check_1` booleans
    check_2s = [] # list to track all `check_2` booleans
    check_3s = [] # list to track all `check_3` booleans
    check_4s = [] # list to track all `check_4` booleans

    for idx in range(len(dnt_x_1s) - 1):
        new_longitude_average = (dnt_x_1s[idx] + dnt_x_1s[idx + 1] + dnt_x_2s[idx] + dnt_x_2s[idx + 1])/4 # average longitude coordinate of all boundary coordinates applicable to the discretization
        dnt_average_longs.append(new_longitude_average)
        new_latitude_average = (dnt_y_1s[idx] + dnt_y_1s[idx + 1] + dnt_y_2s[idx] + dnt_y_2s[idx + 1])/4 # average latitude coordinate of all boundary coordinates applicable to the discretization
        dnt_average_lats.append(new_latitude_average)

        left_x_1 = dnt_x_1s[idx] # first left DNT boundary point x-coordinate
        left_y_1 = dnt_y_1s[idx] # first left DNT boundary point y-coordinate
        left_x_2 = dnt_x_1s[idx + 1] # second left DNT boundary point x-coordinate
        left_y_2 = dnt_y_1s[idx + 1] # second left DNT boundary point y-coordinate

        right_x_1 = dnt_x_2s[idx] # right DNT boundary point x-coordinate
        right_y_1 = dnt_y_2s[idx] # right DNT boundary point y-coordinate
        right_x_2 = dnt_x_2s[idx + 1] # second right DNT boundary point x-coordinate
        right_y_2 = dnt_y_2s[idx + 1] # second right DNT boundary point y-coordinate

        special_case_1_flag = False # flag to indicate whether Special Case 1 is active
        if (left_x_1 == right_x_1 and left_y_1 == right_y_1): # Special Case 1: the DNT begins at a point, so the left and right boundaries collide (results in divide by zero error)
            special_case_1_flag = True
        elif (left_x_1 == left_x_2 and left_y_1 == left_y_2): # Sometimes the DNT has consecutive points with the same coordinates, still unsure why, might be caused by time steps too small
            continue # continue to the next loop iteration
        elif (right_x_1 == right_x_2 and right_y_1 == right_y_2): # Sometimes the DNT has consecutive points with the same coordinates, still unsure why, might be caused by time steps too small
            continue # continue to the next loop iteration

        if (special_case_1_flag): # only run Angle Check 1 if special case 1 is not active
            check_1 = True
        else:
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
            vec_3 = [current_long - left_x_1, current_lat - left_y_1] # vector from the first relevent left boundary point to the current postion
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
        vec_6 = [current_long - left_x_2, current_lat - left_y_2] # vector from the second relevent left boundary point to the current postion
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
        vec_9 = [current_long - right_x_2, current_lat - right_y_2] # vector from the second relevent right boundary point to the current postion
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

        if (special_case_1_flag): # only run Angle Check 4 if special case 1 is not active
            check_4 = True
        else:
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
            vec_12 = [current_long - right_x_1, current_lat - right_y_1] # vector from the first relevent right boundary point to the current postion
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

        check_1s.append(check_1) # append `check_1` to the list of tracked check_1s
        check_2s.append(check_2) # append `check_2` to the list of tracked check_2s
        check_3s.append(check_3) # append `check_3` to the list of tracked check_3s
        check_4s.append(check_4) # append `check_4` to the list of tracked check_4s

    # Iterate Through Lists of "Checks"
    for idx in range(len(check_1s)):
        if (check_1s[idx] and check_2s[idx] and check_3s[idx] and check_4s[idx]): # if the rocket's position satifies all four checks for any DNT discretization
            # TBR, may eroneously trigger abort when the rocket exceeds the limit of the DNT (but by then we probably won't be checking the DNT anymore in the flight software)
            abort_counts = 0 # reset the abort counter
            return False, abort_counts # return no abort

    # Below code will only run if the rocket's position does not satisfy all four checks for any DNT discretization
    abort_counts += 1 # increment the abort counter
    if (abort_counts >= abort_count_threshold): # can be sufficiently confident the rocket has exited the DNT
        return True, abort_counts # return abort
    else:
        return False, abort_counts # return no abort until `about_count_threshold` reached/surpassed

try:
    launch_date = command_line_args["date"] # MM-DD-YYYY format required
    launch_hour = command_line_args["time"] # HH format required (24-hour)
except (IndexError):
    print("---------------------------------------------------------------")
    print("NEED DATE AND TIME SPECIFIED ON THE COMMAND LINE: MM-DD-YYYY HH")
    print("---------------------------------------------------------------")

optimal_landing_zone_df = pd.read_csv(f"../DNT/{date_dir_with_time}/optimal_landing_zone.csv") # df of optimal landing zone information
optimal_perimeter_coords = all_landing_zone_perimeters[optimal_landing_zone_df["index"][0]] # coordinates of optimal landing zone perimeter

ellipse = EllipseModel()
if (ellipse.estimate(optimal_perimeter_coords)): # fit the best-fit model to the optimal landing zone perimeter coordss
    landing_zone_patch = Ellipse(xy=(ellipse.params[0], ellipse.params[1]), width=2*ellipse.params[2], height=2*ellipse.params[3], angle=math.degrees(ellipse.params[4]), edgecolor='r', facecolor="None")
    launch_area_ax.add_patch(landing_zone_patch)

dnt_points_file_path = f"../DNT/Results/{launch_date}/{launch_hour}/DNT.csv" # path to the CSV file containing the points of the DNT boundaries
dnt_points_df = pd.read_csv(dnt_points_file_path) # Points of the DNT boundaries as a pandas df
dnt_ts = dnt_points_df["t"].tolist() # list of timesteps defining the DNT
dnt_x_1s = dnt_points_df["x_1"].tolist() # list of longitude coordinates of the DNT left boundary
dnt_y_1s = dnt_points_df["y_1"].tolist() # list of latitude coordinates of the DNT left boundary
dnt_x_2s = dnt_points_df["x_2"].tolist() # list of longitude coordinates of the DNT right boundary
dnt_y_2s = dnt_points_df["y_2"].tolist() # list of latitude coordinates of the DNT right boundary

date_path = f"../DNT/Results/{launch_date}" # path to the folder with the DNT information for the date of interest

optimal_launch_information_file_path = f"{date_path}/{launch_date}.csv" # path to CSV file with optimal launch information for the date of interest
optimal_launch_information_df = pd.read_csv(optimal_launch_information_file_path) # optimal launch information for the date of interest as a pandas df
optimal_inclination = optimal_launch_information_df["Inclination"][0] # optimal launch inclination, TBR, not robust for multiple times per day
optimal_heading = optimal_launch_information_df["Heading"][0] # optimal launch heading, TBR, not robust for multiple times per day

if __name__ == "__main__":
    num_trajectories = 50 # number of trajectories to simulate
    for elem in range(num_trajectories):
        launch_inclination = np.random.uniform(low=optimal_inclination - 1, high=optimal_inclination + 1) # [deg] randomly draw launch inclination
        launch_heading = np.random.uniform(low=optimal_heading - 1, high=optimal_heading + 1) # [deg] randomly draw launch heading
        print(f"Iteration: {elem}, Inclination: {round(launch_inclination, 2)} deg, Heading: {round(launch_heading, 2)} deg")

        test_flight = Flight(
            rocket=DART_rocket_1,
            environment=launch_site,
            rail_length=launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
            inclination=launch_inclination, # [deg] rail inclination relative to the ground
            heading=launch_heading, # [deg] heading angle relative to North (East = 90)
            time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
        ) # run trajectory simulation

        abort_bools = [] # list to track the `abort_bool` values returned from `check_dnt()`
        abort_color = 'g' # plot in green if the trajectory remains within the DNT
        abort_counts = 0 # counter to track the number of trajectory positions that exited the DNT
        abort_count_threshold = 3 # number of trajectory positions that must exit the DNT to trigger an abort

        # Iterate through Solution Steps in the Simulated Trajectory
        for solution_step in test_flight.solution:
            time_step_time = solution_step[0] # [s] time of the current solution step
            time_step_long = test_flight.longitude(time_step_time) # [deg] longitude position of the trajectory at the current time step
            time_step_lat = test_flight.latitude(time_step_time) # [deg] latitude position of the trajectory at the current time step

            if (landing_zone_patch.contains_point(point=launch_area_ax.transData.transform(values=(time_step_long, time_step_lat)))): # if the trajectory is within the landing zone
                abort_bool = False # don't trigger an abort (but don't reset the abort counter)
            else: # check the rocket's position against the DNT
                abort_bool, abort_counts = check_dnt(DNT_FILE_PATH=dnt_points_file_path,
                                                    current_time=time_step_time,
                                                    current_long=time_step_long,
                                                    current_lat=time_step_lat,
                                                    abort_counts=abort_counts,
                                                    abort_count_threshold=abort_count_threshold)
            abort_bools.append(abort_bool)

        if (any(abort_bools)): # if an abort was ever triggered
            print("ABORT TRIGGERED")
            abort_color = 'r' # plot in red if the trajectory exits the DNT

        solution_time = [solution_step[0] for solution_step in test_flight.solution] # [s] time array of solution
        launch_area_ax.plot(test_flight.longitude(solution_time), test_flight.latitude(solution_time), '-', color=abort_color, markersize=1)

    launch_area_ax.plot(dnt_x_1s, dnt_y_1s, 'b.-', markersize=1) # plot left boundary line segments
    launch_area_ax.plot(dnt_x_2s, dnt_y_2s, 'b.-', markersize=1) # plot right boundary line segments
    launch_area_ax.set_title(f"SYS.09 Verification")

    plt.tight_layout()
    plt.savefig(f"SYS09_Verification.png", transparent=True, dpi=1000) # save the figure with a transparent background
    plt.show()