from inspect import signature
import numpy as np
from scipy.linalg import solve_continuous_are
from ..prints.controller_prints import _ControllerPrints


class _Controller:
    """A class for storing and running controllers on a rocket. Controllers
    have a controller function that is called at a specified sampling rate
    during the simulation. The controller function can access and modify
    the objects that are passed to it. The controller function also stores the
    variables of interest in the objects that are passed to it."""

    def __init__(
        self,
        interactive_objects,
        controller_function,
        sampling_rate,
        initial_observed_variables=None,
        name="Controller",
    ):
        """Initialize the class with the controller function and the objects to
        be observed.

        Parameters
        ----------
        interactive_objects : list or object
            A collection of objects that the controller function can access and
            potentially modify. This can be either a list of objects or a single
            object. The objects listed here are provided to the controller function
            as the last argument, maintaining the order specified in this list if
            it's a list. The controller function gains the ability to interact with
            and make adjustments to these objects during its execution.
        controller_function : function, callable
            An user-defined function responsible for controlling the simulation.
            This function is expected to take the following arguments, in order:

            1. `time` (float): The current simulation time in seconds.
            2. `sampling_rate` (float): The rate at which the controller
               function is called, measured in Hertz (Hz).
            3. `state` (list): The state vector of the simulation, structured as
               `[x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]`.
            4. `state_history` (list): A record of the rocket's state at each
               step throughout the simulation. The state_history is organized as
               a list of lists, with each sublist containing a state vector. The
               last item in the list always corresponds to the previous state
               vector, providing a chronological sequence of the rocket's
               evolving states.
            5. `observed_variables` (list): A list containing the variables that
               the controller function returns. The return of each controller
               function call is appended to the observed_variables list. The
               initial value in the first step of the simulation of this list is
               provided by the `initial_observed_variables` argument.
            6. `interactive_objects` (list): A list containing the objects that
               the controller function can interact with. The objects are
               listed in the same order as they are provided in the
               `interactive_objects`.
            7. `sensors` (list): A list of sensors that are attached to the
                rocket. The most recent measurements of the sensors are provided
                with the ``sensor.measurement`` attribute. The sensors are
                listed in the same order as they are added to the rocket

            This function will be called during the simulation at the specified
            sampling rate. The function should evaluate and change the interactive
            objects as needed. The function return statement can be used to save
            relevant information in the `observed_variables` list.

            .. note:: The function will be called according to the sampling rate
            specified.
        sampling_rate : float
            The sampling rate of the controller function in Hertz (Hz). This
            means that the controller function will be called every
            `1/sampling_rate` seconds.
        initial_observed_variables : list, optional
            A list of the initial values of the variables that the controller
            function returns. This list is used to initialize the
            `observed_variables` argument of the controller function. The
            default value is None, which initializes the list as an empty list.
        name : str
            The name of the controller. This will be used for printing and
            plotting.

        Returns
        -------
        None
        """
        self.interactive_objects = interactive_objects
        self.base_controller_function = controller_function
        self.controller_function = self.__init_controller_function(controller_function)
        self.sampling_rate = sampling_rate
        self.initial_observed_variables = initial_observed_variables
        self.name = name
        self.prints = _ControllerPrints(self)

        if initial_observed_variables is not None:
            self.observed_variables = [initial_observed_variables]
        else:
            self.observed_variables = []

    def __init_controller_function(self, controller_function):
        """Checks number of arguments of the controller function and initializes
        it with the correct number of arguments. This is a workaround to allow
        the controller function to receive sensors without breaking changes"""

        print("Controller Initialized")


        sig = signature(controller_function)
        if len(sig.parameters) == 6:

            # pylint: disable=unused-argument
            def new_controller_function(
                time,
                sampling_rate,
                state_vector,
                state_history,
                observed_variables,
                interactive_objects,
                sensors,
            ):
                return controller_function(
                    time,
                    sampling_rate,
                    state_vector,
                    state_history,
                    observed_variables,
                    interactive_objects,
                )

        elif len(sig.parameters) == 7:
            new_controller_function = controller_function
        else:
            raise ValueError(
                "The controller function must have 6 or 7 arguments. "
                "The arguments must be in the following order: "
                "(time, sampling_rate, state_vector, state_history, "
                "observed_variables, interactive_objects, sensors)."
                "Sensors argument is optional."
            )
        return new_controller_function

    def __call__(self, time, state_vector, state_history, sensors):
        """Call the controller function. This is used by the simulation class.

        Parameters
        ----------
        time : float
            The time of the simulation in seconds.
        state_vector : list
            The state vector of the simulation, which is defined as:

            `[x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]`.
        state_history : list
            A list containing the state history of the simulation. The state
            history is a list of every state vector of every step of the
            simulation. The state history is a list of lists, where each
            sublist is a state vector and is ordered from oldest to newest.
        sensors : list
            A list of sensors that are attached to the rocket. The most recent
            measurements of the sensors are provided with the
            ``sensor.measurement`` attribute. The sensors are listed in the same
            order as they are added to the rocket.

        Returns
        -------
        None
        """
        observed_variables = self.controller_function(
            time,
            self.sampling_rate,
            state_vector,
            state_history,
            self.observed_variables,
            self.interactive_objects,
            sensors,
        )
        if observed_variables is not None:
            self.observed_variables.append(observed_variables)

        print(f"Controller call at {time:.2f}s | Observed Vars: {observed_variables}")


    # --- LQR Controller Class ---
    def get_state_matrices(self, Ix, Iy, Iz, mass, altitude, dt):
        """
        Compute system matrices A and B for LQR control.
        
        Parameters
        ----------
        Ix, Iy, Iz : float
            Principal moments of inertia (kg·m²).
        mass : float
            Rocket mass (kg).
        altitude : float
            Rocket altitude (m).
        dt : float
            Sampling time step (s).
        
        Returns
        -------
        A, B : np.array
            State-space matrices.
        """

        A = np.array([
            [0, 0, 0],  # Vertical velocity dynamics
            [0, 0, 0],  # Horizontal velocity dynamics
            [0, 0, 0]   # Attitude dynamics
        ])

        B = np.array([
            [1/mass, 0],    # Control input affects vertical acceleration
            [0, 1/mass],    # Control input affects horizontal acceleration
            [1/Ix, 1/Iy]    # Control input affects attitude (simplified model)
        ])

        return A, B

    def lqr(self, A, B, Q, R):
        """
        Solve the Algebraic Riccati Equation (ARE) for optimal control gain matrix K.

        Parameters
        ----------
        A : np.array
            System state matrix.
        B : np.array
            Control input matrix.
        Q : np.array
            State cost matrix.
        R : np.array
            Control cost matrix.

        Returns
        -------
        K : np.array
            Optimal LQR gain matrix.
        """

        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K

    def get_attitude(self, state_vector):
        """
        Compute tilt angle from quaternions.

        Parameters
        ----------
        state_vector : list
            Rocket state vector containing quaternions.

        Returns
        -------
        tilt_angle : float
            Tilt angle (rad) relative to vertical.
        """

        e0, e1, e2, e3 = state_vector[6:10]  # Extract quaternions
        tilt_angle = np.arccos(2 * (e0**2 + e3**2) - 1)  # Compute tilt from quaternion
        return tilt_angle


    def tvc_lqr_controller(time, sampling_rate, state_vector, state_history, observed_variables, interactive_objects, sensors=None):
        """
        LQR Controller for Thrust Vector Control (TVC) system.

        Parameters
        ----------
        time : float
            Current simulation time (s).
        sampling_rate : float
            Sampling rate of the controller (Hz).
        state_vector : list
            Rocket state [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz].
        state_history : list
            History of previous state vectors.
        observed_variables : list
            Controller-observed variables for logging.
        interactive_objects : list
            List of objects that the controller can modify.
        sensors : list
            List of sensors providing real-time data.

        Returns
        -------
        list
            Updated observed variables (pitch_command, yaw_command).
        """
        print("Controller activated")
        dt = 1.0 / sampling_rate  # Time step (s)

        # Extract TVC system and Rocket objects
        tvc_system = interactive_objects[0]
        rocket = interactive_objects[1]

        # Extract relevant state variables
        vz = state_vector[5]  # Vertical velocity
        vh = np.sqrt(state_vector[3]**2 + state_vector[4]**2)  # Horizontal velocity
        attitude = self.get_attitude(state_vector)  # Compute rocket tilt angle

        # Define current state vector
        x = np.array([vz, vh, attitude])

        # Define reference state (Target: zero velocity & upright)
        x_ref = np.array([0.0, 0.0, 0.0])  # [vz_target, vh_target, attitude_target]

        # Compute state error
        x_error = x - x_ref

        # Retrieve Rocket properties
        inertia_tensor = rocket.get_inertia_tensor_at_time(time)
        Ix, Iy, Iz = inertia_tensor.diagonal()
        mass = rocket.total_mass.get_value_opt(time)
        altitude = state_vector[2]

        # Compute system matrices A and B
        A, B = self.get_state_matrices(Ix, Iy, Iz, mass, altitude, dt)

        # Define LQR cost matrices
        Q = np.diag([100, 100, 10])  # Penalize velocity and attitude errors
        R = np.diag([10, 10])  # Penalize gimbal movement

        # Compute optimal control gains
        K = self.lqr(A, B, Q, R)
        u = -K @ x_error  # Compute optimal control input (based on error)

        # Apply TVC constraints (limit gimbal angles)
        pitch_command = np.clip(u[0], -tvc_system.max_gimbal, tvc_system.max_gimbal)
        yaw_command = np.clip(u[1], -tvc_system.max_gimbal, tvc_system.max_gimbal)

        # Update TVC system with new gimbal angles
        tvc_system.update_gimbal(pitch_command, yaw_command, dt)

        # Return updated observed variables
        return [pitch_command, yaw_command]

    def __str__(self):
        return f"Controller '{self.name}' with sampling rate {self.sampling_rate} Hz."

    def info(self):
        """Prints out summarized information about the controller."""
        self.prints.all()

    def all_info(self):
        """Prints out all information about the controller."""
        self.info()


