# Python Libraries
from rocketpy import Flight, MonteCarlo, StochasticEnvironment, StochasticFlight, StochasticRocket
import sys
sys.path.append("../../")
# DART Modules
import setup
import ascent_test_2

test_flight = Flight(
    rocket=setup.DART_rocket_1,
    environment=setup.launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=ascent_test_2.optimal_inclination, # [deg] rail inclination relative to the ground
    heading=ascent_test_2.optimal_heading, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
) # run trajectory simulation

stochastic_launch_site = StochasticEnvironment(
    environment=setup.launch_site,
    wind_velocity_x_factor=(0, 0.5), # (scaling factor, standard deviation) normal distribution
    wind_velocity_y_factor=(0, 0.5) # (scaling factor, standard deviation) normal distribution
)

stochastic_launch_site.visualize_attributes()

stochastic_DART_rocket_1 = StochasticRocket(
    rocket=setup.DART_rocket_1
)

stochastic_DART_rocket_1.add_motor(setup.motors.AeroTechG25W)
stochastic_DART_rocket_1.add_nose(setup.rockets.DART_nose)
stochastic_DART_rocket_1.add_trapezoidal_fins(setup.rockets.DART_fins_rocket_1)
stochastic_DART_rocket_1.set_rail_buttons(setup.rockets.DART_rail_buttons)
# stochastic_DART_rocket_1.add_parachute(setup.)

stochastic_DART_rocket_1.visualize_attributes()

stochastic_test_flight = StochasticFlight(
    flight=test_flight
)

test_flight_dispersion = MonteCarlo(
    filename="monte_carlo_dispersion",
    environment=stochastic_launch_site,
    rocket=stochastic_DART_rocket_1,
    flight=stochastic_test_flight,
)

test_flight_dispersion.simulate(number_of_simulations=1000, append=False)

test_flight_dispersion.prints.all()