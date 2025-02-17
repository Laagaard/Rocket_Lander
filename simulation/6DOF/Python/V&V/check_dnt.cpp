#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>

/**
 * @brief Method to Check the Current Rocket Position Against the Domain of Nominal Trajectories and Determine the Necessity of an In-Flight Abort
 * 
 * @param DNT_FILE_PATH Path to the CSV file containing the DNT information
 * @param current_time Current flight time [s]
 * @param current_long Current longitude of the rocket's position [deg]
 * @param current_lat Current latitude of the rocket's position [deg]
 * @param abort_counts Counter to track the number of consecutive trajectory positions that have exited the DNT, increments by 1 if an abort seems desired
 * @return Updated value of the `abort_counts` input
*/
int check_dnt(std::string DNT_FILE_PATH, float_t current_time, float_t current_long, float_t current_lat, int8_t abort_counts)
{
    std::ifstream dnt_file(DNT_FILE_PATH); // input file

    if (!dnt_file.is_open()) { // if the file (erroneously) is not open
        std::cerr << "Failed to open DNT file: " << DNT_FILE_PATH << std::endl;
        return abort_counts; // return the `abort_counts` parameter without changing it
    }

    const int array_length = 150;
    float dnt_ts[array_length] = {0}; // array of timesteps defining the DNT
    float dnt_x_1s[array_length] = {0}; // array of longitude coordinates of the DNT left boundary
    float dnt_y_1s[array_length] = {0}; // array of latitude coordinates of the DNT left boundary
    float dnt_x_2s[array_length] = {0}; // array of longitude coordinates of the DNT right boundary
    float dnt_y_2s[array_length] = {0}; // array of latitude coordinates of the DNT right boundary

    std::string line; // string to store the current DNT file line
    int line_ctr = 0; // counter to track the current line number
    while (std::getline(dnt_file, line)) { // iterate lines of the DNT CSV file
        std::stringstream ss(line); // stringstream to iterate the elements of the current file line
        std::string element; // string to hold the current elememt of the file line

        // Iterate through elements of the DNT file row
        int row_idx = 0; // index counting the DNT CSV column
        while (std::getline(ss, element, ',')) {
            if (element == "t") { // if currently on the header row
                break;
            }
            float current_value = std::stof(element); // TODO, will this limit the precision?
            switch (row_idx)
            {
                case (0): // first column of the DNT CSV file: time
                    dnt_ts[line_ctr] = current_value;
                    break;
                case (1): // second column of the DNT CSV file: x_1 (left boundary longitude coordinate)
                    dnt_x_1s[line_ctr] = current_value;
                    break;
                case (2): // third column of the DNT CSV file: y_1 (left boundary latitude coordinate)
                    dnt_y_1s[line_ctr] = current_value;
                    break;
                case (3): // fourth column of the DNT CSV file: x_2 (right boundary longitude coordinate)
                    dnt_x_2s[line_ctr] = current_value;
                    break;
                case (4): // fifth column of the DNT CSV file: y_2 (right boundary latitude coordinate)
                    dnt_y_2s[line_ctr] = current_value;
                    break;
            }
            row_idx++; // increment the row index
        }

        if (element == "t") { // if currently on the header row
            continue;
        }
        std::cout << dnt_ts[line_ctr] << ", " << dnt_x_1s[line_ctr] << ", " << dnt_y_1s[line_ctr] << ", " << dnt_x_2s[line_ctr] << ", " << dnt_y_2s[line_ctr] << std::endl;
        line_ctr++; // increment the line counter
    }

    // TODO, need to perform time check (lambda function)

    bool check_1s[array_length] = {0}; // array to track all `check_1` booleans
    bool check_2s[array_length] = {0}; // array to track all `check_2` booleans
    bool check_3s[array_length] = {0}; // array to track all `check_3` booleans
    bool check_4s[array_length] = {0}; // array to track all `check_4` booleans

    int lower_boundary_idx = 0; // counter to track the lower boundary of the current DNT discretization

    while (dnt_x_1s[lower_boundary_idx] != 0) {

        float left_x_1 = dnt_x_1s[lower_boundary_idx]; // first left DNT boundary point x-coordinate
        float left_y_1 = dnt_y_1s[lower_boundary_idx]; // first left DNT boundary point y-coordinate
        float left_x_2 = dnt_x_1s[lower_boundary_idx + 1]; // second left DNT boundary point x-coordinate
        float left_y_2 = dnt_y_1s[lower_boundary_idx + 1]; // second left DNT boundary point y-coordinate

        float right_x_1 = dnt_x_2s[lower_boundary_idx]; // right DNT boundary point x-coordinate
        float right_y_1 = dnt_y_2s[lower_boundary_idx]; // right DNT boundary point y-coordinate
        float right_x_2 = dnt_x_2s[lower_boundary_idx + 1]; // second right DNT boundary point x-coordinate
        float right_y_2 = dnt_y_2s[lower_boundary_idx + 1]; // second right DNT boundary point y-coordinate

        bool special_case_1_flag = false; // flag to indicate whether Special Case 1 is active
        if (left_x_1 == right_x_1 && left_y_1 == right_y_1) { // Special Case 1: the DNT begins at a point, so the left and right boundaries collide (results in divide by zero error)
            special_case_1_flag = true;
        } else if (left_x_1 == left_x_2 && left_y_1 == left_y_2) { // Sometimes the DNT has consecutive points with the same coordinates, still unsure why, might be caused by time steps too small
            lower_boundary_idx++; // increment the lower boundary counter
            continue; // continue to the next loop iteration
        } else if (right_x_1 == right_x_2 && right_y_1 == right_y_2) { // Sometimes the DNT has consecutive points with the same coordinates, still unsure why, might be caused by time steps too small
            lower_boundary_idx++; // increment the lower boundary counter
            continue; // continue to the next loop iteration
        }

        if (special_case_1_flag) { // only run Angle Check 1 if special case 1 is not active
            bool check_1 = true;
        } else { // special case 1 is not active
            /*
            Angle Check 1
            -------------
            Vector 1: Vector from (left_x_1, left_y_1) to (right_x_1, right_y_1)
            Vector 2: Vector from (left_x_1, left_y_1) to (left_x_2, left_y_2)
            Vector 3: Vector from (left_x_1, left_y_1) to the current rocket position
            */

            // Compute Vector 1 & Magnitude
            float vec_1[2] = {right_x_1 - left_x_1, right_y_1 - left_y_1}; // vector from the first relevant left boundary point to the first relevent right boundary point
            float vec_1_mag = 0; // magnitude of the above vector
            for (int i = 0; i < sizeof(vec_1)/sizeof(vec_1[0]); i++) {
                vec_1_mag += pow(vec_1[i], 2);
            }
            vec_1_mag = sqrt(vec_1_mag);

            // Compute Vector 2 & Magnitude
            float vec_2[2] = {left_x_2 - left_x_1, left_y_2 - left_y_1}; // vector from the first relevant left boundary point to the second relevant left boundary point
            float vec_2_mag = 0; // magnitude of the above vector
            for (int i = 0; i < sizeof(vec_2)/sizeof(vec_2[0]); i++) {
                vec_2_mag += pow(vec_2[i], 2);
            }
            vec_2_mag = sqrt(vec_2_mag);

            // Compute Vector 3 & Magnitude
            float vec_3[2] = {current_long - left_x_1, current_lat - left_y_1}; // vector from the first relevent left boundary point to the current postion
            float vec_3_mag = 0; // magnitude of the above vector
            for (int i = 0; i < sizeof(vec_3)/sizeof(vec_3[0]); i++) {
                vec_3_mag += pow(vec_3[i], 2);
            }
            vec_3_mag = sqrt(vec_3_mag);

            // Compute Dot (Scalar) Product of Vector 1 & Vector 2
            float vec1_vec2_dot = (vec_1[0] * vec_2[0]) + (vec_1[1] * vec_2[1]); // dot product of vec_1 and vec_2
            float vec1_vec2_dot_normalized = vec1_vec2_dot/(vec_1_mag * vec_2_mag);
            if (vec1_vec2_dot_normalized < -1 || vec1_vec2_dot_normalized > 1) {
                float angle_vec1_vec2 = std::round(std::acos(vec1_vec2_dot_normalized)); // [rad] angle between vec_1 and vec_2 (tail-to-tail), round to either 1 or -1 if roundoff error occurs
            } else {
                float angle_vec1_vec2 = std::acos(vec1_vec2_dot_normalized); // [rad] angle between vec_1 and vec_2 (tail-to-tail)
            }

            // Compute Dot (Scalar) Product of Vector 1 & Vector 3
            float vec1_vec3_dot = (vec_1[0] * vec_3[0]) + (vec_1[1] * vec_3[1]); // dot product of vec_1 and vec_3
            float vec1_vec3_dot_normalized = vec1_vec3_dot/(vec_1_mag * vec_3_mag);
            if (vec1_vec3_dot_normalized < -1 || vec1_vec3_dot_normalized > 1) {
                float angle_vec1_vec3 = std::round(std::acos(vec1_vec3_dot_normalized)); // [rad] angle between vec_1 and vec_3 (tail-to-tail), round to either 1 or -1 if roundoff error occurs
            } else {
                float angle_vec1_vec3 = std::acos(vec1_vec3_dot_normalized); // [rad] angle between vec_1 and vec_3 (tail-to-tail)
            }

            // bool check_1 = (angle_vec1_vec2 > angle_vec1_vec3); // desire the angle between vec_1 and vec_2 to be greater than the angle between vec_1 and vec_3
        }

        // std::cout << lower_boundary_idx << std::endl;
        lower_boundary_idx++; // increment the lower boundary counter
    }

    return abort_counts; // return the updated value of `abort_counts`
}

int main()
{
    int8_t abort_counts = 0; // intialize abort counter

    abort_counts = check_dnt("../DNT/Results/02-23-2025/10/DNT.csv", abort_counts, 0, 0, 0);
}