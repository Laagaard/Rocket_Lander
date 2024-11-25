# `simulation`
## Forward Work
### TVC Implementation (TOP PRIORITY)
- See `RocketPy/rocketpy/simulation/flight.py` for state vector derivative code (line 1668)
- See `RocketPy/rocketpy/control/controller.py` for the Controller class
- Airbrakes controller example: https://docs.rocketpy.org/en/latest/user/airbrakes.html
- STRONGLY consider branching from `master` before making any changes for TVC (https://docs.rocketpy.org/en/latest/development/style_guide.html)

### Abort System
#### Domain of Nominal Trajectories (DNT) Development
- [ ] Investigate Monte Carlo analyses with launch inclination and heading uncertainties, and <u>varying wind velocities/directions</u>
#### Parachute Deployment Trigger Function
- [ ] Investigate trigger function development (https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html#parachute-class)

    - Goal: Determine method of implementation

- [ ] Identify abort system trigger criteria (will be based on the DNT?)
- [ ] Implement abort system trigger function with identified criteria to trigger parachute deployment upon departure from the DNT