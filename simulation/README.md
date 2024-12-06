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
- [ ] Investigate Monte Carlo analyses with launch inclination and heading uncertainties, and <u>varying wind velocities/directions</u>

    - Goal: Determine method for generation of dispersion analysis within landing zone (i.e., determine method to obtain tolerable wind uncertainties for remaining within landing zone with some confidence level)

- [ ] Implement Monte Carlo analyses with launch inclination and heading uncertainties, and <u>varying wind velocities/directions</u>

    - Goal: Generate dispersion analysis within landing zone (with confidence levels) based on wind velocity uncertainties

        - May be desired to generate confidence levels based on confidence of landing within landing zone <u>as well as</u> confidence of landing within safe region

- [ ] Investigate/implement generation of DNT from Monte Carlo results via time-based ellipse (or other shape) cross-sections of flight envelope

    - Goal: Determine method for generation of the time-based plane defined by the acceptable positions
    - Possible approach:

        1) Amalgamate Monte Carlo results

            1) Need each time step, 3D (inertial) positions of ALL trajectories at each time step, and 3D (inertial) velocities of ALL trajectories at each time step

        2) At each time step, apply a best-fit ellipse to the edge[^1] points of the generated trajectories in 3D inertial space
        
            1) Define the center of the ellipse as the position of the center-most trajectory in the plane (or average of all trajectory positions)
            2)  Define the normal vector (orientation) of the ellipse as the average unit velocity vector of all trajectories at the time index defining the plane

                1) Other orientation possibilites:

                    1) "Average" quaternion of all trajectories at the given time index?
            3) End results:

                1) Parameters $a$ and $b$ defining the ellipse shape according to $\frac{x^2}{a} + \frac{y^2}{b} \leq 1$
                2) 3D inertial position describing the center of the ellipse
                3) Parametrization describing the orientation of the ellipse in 3D inertial space
        3) Create a callable function that takes in the complete geometry of the ellipse (in 3D inertial space) and a point in 3D inertial space, and returns (boolean?) whether or not the point projected onto the plane defined by the ellipse lies within the ellipse

[^1]: "edge" may be defined with respect to landing within the landing zone <u>OR</u> just within a safe area

#### Parachute Deployment Trigger Function
- [ ] Investigate trigger function development (https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html#parachute-class)

    - Goal: Determine method of implementation

- [ ] Identify abort system trigger criteria (will likely be based on the DNT)
- [ ] Implement abort system trigger function with identified criteria to trigger parachute deployment upon departure from the DNT