import sys
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import quaternion

sys.path.append('../')
import setup
from rocketpy import Flight, _Controller, Environment
from TVC import TVC

# --- First Simulation (Ascent Phase) ---
test_flight = Flight(
    rocket=setup.DART_rocket_1,  # Use Rocket 1 (Ascent)
    environment=setup.launch_site,
    rail_length=1.5,  # TBR (m)
    inclination=86,  # TBR (deg)
    heading=0,  # (deg)
    time_overshoot=True
)

# --- Extract Time, Velocity, and Altitude ---
time = [row[0] for row in test_flight.solution]  
altitude = [test_flight.z(t) for t in time] 
vx = [test_flight.vx(t) for t in time] 
vy = [test_flight.vy(t) for t in time] 
vz = [test_flight.vz(t) for t in time]  # Fixed: Use vz instead of vx
velocity = [test_flight.speed(t) for t in time]  

# --- Calculate Momentum (p = m * v) ---
mass = [setup.DART_rocket_1.total_mass(t) for t in time]

# Convert lists to NumPy arrays
mass = np.array(mass)
velocity = np.array(velocity)

# Perform element-wise multiplication
momentum = mass * velocity

# --- Find Motor Burnout Time ---
t_burnout = setup.AeroTechG25W.burn_out_time  
burnout_index = min(range(len(time)), key=lambda i: abs(time[i] - t_burnout))
motor_impulse = momentum[burnout_index]

altitude = test_flight.z(time[burnout_index])
print("Altitude when burnout:", altitude)

# Initialize index_fire
index_fire = None

# Iterate over indices of velocity
for x in range(len(velocity)):
    # Getting magnitude of velocity (TVC will focus on limiting components)
    rocket_impulse = rocket_impulse = setup.DART_rocket_2.total_mass(time[x]) * velocity[x]

    # Momentum & Impulse logic
    if rocket_impulse > motor_impulse and vz[x] < 0:
        index_fire = x + 10
        break  # Exit loop once the condition is met

print(index_fire)

# Check if index_fire was found
if index_fire is not None:
    print(f"Descent motor ignition index: {index_fire}")

    # --- Adjust Initial Conditions for Descent Phase ---
    speed = test_flight.speed(time[index_fire])
    altitude = test_flight.z(time[index_fire])

    print("Speed when descent fire:", speed)
    print("Altitude when descent fire:", altitude)
    # Compute quaternion conjugate to flip orientation
    q0 = test_flight.e0(time[index_fire])
    q1 = test_flight.e1(time[index_fire])
    q2 = test_flight.e2(time[index_fire])
    q3 = test_flight.e3(time[index_fire])

    # print('Original Quat:',[q0, q1, q2, q3])

    # dcm2 = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    
    # dcm1 = R.from_quat([q0, q1, q2, q3])
    
    # dcm3 = dcm2 * dcm1.as_matrix()

    # q_flipped = R.from_matrix(dcm3).as_quat() # Flipped quaternion

    # print('New Quat:',q_flipped)

    q_flipped = [-q1, q0, -q3, q2] # Long rotation

    # --- Enable TVC for Descent Rocket ---
    # ENABLE_TVC = True  # Enable TVC for second rocket body

    # --- Second Simulation (Descent Phase) ---
    # Construct initial_solution as a 1D array
    initial_solution = [
        0.0,  # Time (scalar)
        test_flight.x(time[index_fire]),  # Position x
        test_flight.y(time[index_fire]),  # Position y
        test_flight.z(time[index_fire]),  # Position z
        test_flight.vx(time[index_fire]),  # Velocity x
        test_flight.vy(time[index_fire]),  # Velocity y
        test_flight.vz(time[index_fire]),  # Velocity z
        q_flipped[0],  # Quaternion e0
        q_flipped[1],  # Quaternion e1
        q_flipped[2],  # Quaternion e2
        q_flipped[3],  # Quaternion e3
        test_flight.w1(time[index_fire]),  # Angular velocity w1
        test_flight.w2(time[index_fire]),  # Angular velocity w2
        test_flight.w3(time[index_fire])   # Angular velocity w3
    ]

    # Attach TVC system using the add_TVC method
    tvc_system, tvc_controller = setup.DART_rocket_2.add_TVC(
        TVC(max_gimbal=0, servo_rate=375),
        _Controller.tvc_lqr_controller,
        sampling_rate=10,
        initial_observed_variables=None,
        name="TVC LQR Controller"
    )

    # Create the descent flight using Rocket 2
    descent_flight = Flight(
        rocket= setup.DART_rocket_2,
        environment=setup.launch_site,
        rail_length=0.1,
        initial_solution=initial_solution,
        time_overshoot=True
    )

    # Plot the 3D trajectory
    descent_flight.plots.trajectory_3d()
    descent_flight.plots.linear_kinematics_data()
    descent_flight.plots.angular_kinematics_data()

    
    descent_flight.export_data('Lukey_pukey.csv','x','y','altitude','e0','e1','e2','e3')

    # --- Extract Time and Flight Path Angle ---
    time_descent = [t for t in descent_flight.time]  # Extract time points
    path_angle = [descent_flight.attitude_angle(t) for t in time_descent]  # Compute path angle

    # --- Plot Flight Path Angle Over Time ---
    plt.figure(figsize=(10, 5))
    plt.plot(time_descent, path_angle, label="Flight Path Angle", color='g')
    plt.xlabel("Time (s)")
    plt.ylabel("Flight Path Angle (degrees)")
    plt.title("Rocket Flight Path Angle During Descent")
    plt.legend()
    plt.grid()
    plt.show()


    # --- Plot TVC Gimbal Angles Over Time ---
    history = tvc_system.get_gimbal_history()
    if history:
        time_data, pitch_data, yaw_data = zip(*history)
        plt.figure(figsize=(10, 5))
        plt.plot(time_data, np.degrees(pitch_data), label="Pitch Gimbal Angle", color='b')
        plt.plot(time_data, np.degrees(yaw_data), label="Yaw Gimbal Angle", color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("Gimbal Angle (degrees)")
        plt.title("Commanded Gimbal Angles Over Time")
        plt.legend()
        plt.grid()
        plt.show()
    else:
        print("No gimbal commands were recorded.")

