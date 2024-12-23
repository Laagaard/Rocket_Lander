# `simulation`
## Forward Work
- [ ] Investigate [Generic Surfaces and Custom Aerodynamic Coefficients](https://docs.rocketpy.org/en/latest/user/rocket/generic_surface.html#generic-surfaces-and-custom-aerodynamic-coefficients)

### TVC Implementation (TOP PRIORITY)
- See `RocketPy/rocketpy/simulation/flight.py` for state vector derivative code (line 1668)
- See `RocketPy/rocketpy/control/controller.py` for the Controller class
- Airbrakes controller example: https://docs.rocketpy.org/en/latest/user/airbrakes.html
- STRONGLY consider branching from `master` before making any changes for TVC (https://docs.rocketpy.org/en/latest/development/style_guide.html)

Planned Implementation Approach:

1) Create new `u_dot_generalized` (line 1668 of `RocketPy/rocketpy/simulation/flight.py`) for powered descent flight phase

    1) Will require implementing thrust vector resolution along different body axes
    2) Will require implementing thrust vector controller
2) Create new [FlightPhase](https://docs.rocketpy.org/en/latest/reference/classes/Flight.html#rocketpy.Flight.FlightPhases.FlightPhase) for powered descent
3) Determine how to implement new flight phase in sequence with existing RocketPy flight phases

### Abort System
#### Domain of Nominal Trajectories (DNT) Development
1. Determine and implement method for finding optimal launch inclination and heading to achieve desired lateral displacment coordinates

    - [ ] Deliverable: Algorithm that takes desired `(x,y)` (in inertial coordinates) as input and returns the required trajectory (and, thereby, the required launch inclination and heading)

2. Centered around the optimal trajectory, use Monte Carlo analyses (with launch inclination and heading uncertainties, and <u>varying wind velocities/directions</u>) to generate the DNT

    - [ ] Deliverable: "Spaghetti" plot illustrating the DNT

        - May be desired to generate confidence levels based on confidence of landing within landing zone <u>as well as</u> confidence of landing within safe region

3. Output full state history information for all trajectories in a format suitable for further analysis

    - Option(s) to Consider: CSV file of state history for each trajectory (located in a unique `.gitignore` directory)

4. Implement method for determining left & right bounds (in 2D inertial coordinates) of the DNT during each discretized time interval

5. Implement method for determining upper and lower altitude limits (in 3D inertial coordinates) during each discretized time interval

6. Output all results to a CSV file

    - Format: `#, t_i, t_f, m_1, b_1, m_2, b_2, h_high, h_low` (but don't include the spaces)

        - `#`: discretization number
        - `t_i`: lower time bound to which this discretization applies
        - `t_f`: upper time bound to which this discretization applies
        - `m_1`: slope of the line bounding the left edge of the DNT for this discretization ("left" defined as the launch operator standing at the inertial origin and looking downrange)
        - `b_1`: y-intercept of the line bounding the left edge of the DNT for this discretization
        - `m_2`: slope of the line bounding the right edge of the DNT for this discretization ("right" defined the same as "left" previously)
        - `b_2`: y-intercept of the line bounding the right edge of the DNT for this discretization ("right" defined the same as "left" previously)
        - `h_high`: upper altitude limit for this discretization
        - `h_low`: lower altitude limit for this discretization

#### Parachute Deployment Trigger Function
- [ ] Investigate trigger function development (https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html#parachute-class)

    - Goal: Determine method of implementation

- [ ] Identify abort system trigger criteria (will likely be based on the DNT)
- [ ] Implement abort system trigger function with identified criteria to trigger parachute deployment upon departure from the DNT