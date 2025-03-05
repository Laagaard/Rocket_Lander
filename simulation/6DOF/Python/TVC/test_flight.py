import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../')
import setup
from rocketpy import Flight, _Controller, Environment
from TVC import TVC

# --- First Simulation (Ascent Phase) ---
test_flight = Flight(
    rocket=setup.DART_rocket_1,  # Use Rocket 1 (Ascent)
    environment=setup.launch_site,
    rail_length=1.5,  # TBR (m)
    inclination=80,  # TBR (deg)
    heading=0,  # (deg)
    time_overshoot=True
)

test_flight.plots.trajectory_3d()
test_flight.plots.linear_kinematics_data()
test_flight.plots.attitude_data()
#print(f"Shape of initial_solution: {test_flight.initial_solution}")

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

# Initialize index_fire
index_fire = None

# Iterate over indices of velocity
for x in range(len(velocity)):
    # Getting magnitude of velocity (TVC will focus on limiting components)
    rocket_impulse = rocket_impulse = setup.DART_rocket_2.total_mass(time[x]) * velocity[x]

    # Momentum & Impulse logic
    if rocket_impulse > motor_impulse and vz[x] < 0:
        index_fire = x
        break  # Exit loop once the condition is met

print(index_fire)

# Check if index_fire was found
if index_fire is not None:
    print(f"Descent motor ignition index: {index_fire}")

    # --- Adjust Initial Conditions for Descent Phase ---
    speed = test_flight.speed(time[index_fire])

    print("Speed when descent fire:", speed)
    # Compute quaternion conjugate to flip orientation
    q0 = test_flight.e0(time[index_fire])
    q1 = test_flight.e1(time[index_fire])
    q2 = test_flight.e2(time[index_fire])
    q3 = test_flight.e3(time[index_fire])

    q = [q0, q1, q2, q3]  # Initial Descent Quaternion
    q_rot = [0, 0, 1, 0]  # Flip Quaternion

    q_flipped = [-q2, -q3, q0, q1]  # Flipped quaternion

    # --- Enable TVC for Descent Rocket ---
    ENABLE_TVC = True  # Enable TVC for second rocket body

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

    DART_rocket_2 = setup.DART_rocket_2

    # Initialize TVC System
    if ENABLE_TVC:
        tvc_system = TVC(max_gimbal=10, servo_rate=375)
        DART_rocket_2.TVC = tvc_system  # Attach TVC to descent rocket
    else:
        DART_rocket_2.TVC = None  # Disable TVC if not used

    # --- LQR Controller Setup ---
    if ENABLE_TVC is True:
        sampling_rate = 10  # Hz
        controller = _Controller(
        interactive_objects=[DART_rocket_2.TVC, DART_rocket_2],
        controller_function=_Controller.tvc_lqr_controller,  # Bind to instance
        sampling_rate=sampling_rate,
        initial_observed_variables=None,
        name="LQR Controller"
    )

    DART_rocket_2.controller = controller

    # Create the descent flight
    descent_flight = Flight(
        rocket=setup.DART_rocket_2,  # Use Rocket 2 (Descent)
        environment=setup.launch_site,
        inclination=0,  # Use inclination from ascent phase
        rail_length=0.1,
        heading=0,
        initial_solution=initial_solution,  # Use complete state vector
        time_overshoot=True
    )

    # Plot the 3D trajectory
    descent_flight.plots.trajectory_3d()
    descent_flight.plots.linear_kinematics_data()
    descent_flight.plots.attitude_data()
    setup.AeroTechG25W.all_info()

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