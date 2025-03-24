# Python Libraries
from rocketpy import Flight, MonteCarlo, StochasticEnvironment, StochasticFlight, StochasticRocket, StochasticSolidMotor
# DART Modules
import launch_site
import motors
import rockets
import setup

# Define custom callback functions
def get_impact_longitude(flight):
    '''
    @brief Callable Function to Return the Impact Longitude

    @param flight: RocketPy `flight` object

    @return Longitude of the rocket's impact location
    '''
    return flight.longitude(flight.time[-1])

def get_impact_latitude(flight):
    '''
    @brief Callable Function to Return the Impact Latitude

    @param flight: RocketPy `flight` object

    @return Latitude of the rocket's impact location
    '''
    return flight.latitude(flight.time[-1])

def get_wind_u(flight):
    '''
    @brief Callable Function to Return the Wind U (East) Component at Ground Level

    @param flight: RocketPy `flight` object

    @return Wind u (east) component at ground level
    '''
    return flight.env.wind_velocity_x(flight.env.elevation)

def get_wind_v(flight):
    '''
    @brief Callable Function to Return the Wind V (North) Component at Ground Level

    @param flight: RocketPy `flight` object

    @return Wind v (north) component at ground level
    '''
    return flight.env.wind_velocity_y(flight.env.elevation)

custom_data_collector = {
    "impact_longitude": get_impact_longitude,
    "impact_latitude": get_impact_latitude,
    "wind_u": get_wind_u,
    "wind_v": get_wind_v
}

'''
Construct Stochastic Launch Site Environment
- Anemometer uncertainty is +-2% of readings
'''
nominal_wind_u_surface = launch_site.launch_site.wind_velocity_x(launch_site.launch_site.elevation) # [m/s] nominal value of wind U (east) component at launch site ground level
nominal_wind_v_surface = launch_site.launch_site.wind_velocity_y(launch_site.launch_site.elevation) # [m/s] nominal value of wind V (North) component at launch site ground level
stochastic_launch_site = StochasticEnvironment(
    environment=launch_site.launch_site, # deterministic `Environment`
    wind_velocity_x_factor=(1, 0.02, "uniform"), # (nominal scaling factor, standard deviation, uniform distribution)
    wind_velocity_y_factor=(1, 0.02, "uniform") # (nominal scaling factor, standard deviation, uniform distribution)
)

stochastic_launch_site.visualize_attributes()

# Construct Stochastic AeroTechG79W Solid Rocket Motor
stochastic_AeroTechG79W = StochasticSolidMotor(
    solid_motor=motors.AeroTechG79W, # deterministic `SolidMotor`
    total_impulse=0.1*motors.AeroTechG79W.total_impulse # standard deviation (normal distribution)
)

stochastic_AeroTechG79W.visualize_attributes()

# Construct Stochastic Rocket 1 (for ascent and unpowered descent flight phases)
stochastic_DART_rocket_1 = StochasticRocket(
    rocket=rockets.DART_rocket_1 # deterministic `Rocket`
)

# Add Motor to Stochastic Rocket 1
stochastic_DART_rocket_1.add_motor(motor=stochastic_AeroTechG79W)

# Construct Flight
test_flight = Flight(
    rocket=rockets.DART_rocket_1,
    environment=launch_site.launch_site,
    rail_length=setup.launch_rail_length, # [m] length in which the rocket will be attached to the launch rail
    inclination=86, # [deg] rail inclination relative to the ground
    heading=0, # [deg] heading angle relative to North (East = 90)
    time_overshoot=True # decouples ODE time step from parachute trigger functions sampling rate
)

# Construct Stochastic Flight
stochastic_test_flight = StochasticFlight(
    flight=test_flight # deterministic `Flight`
)

flight_dispersion = MonteCarlo(
    filename="monte_carlo_dispersion",
    environment=stochastic_launch_site,
    rocket=stochastic_DART_rocket_1,
    flight=stochastic_test_flight,
    data_collector=custom_data_collector
)

flight_dispersion.simulate(number_of_simulations=100, append=False)
# flight_dispersion.import_results()
flight_dispersion.prints.all()
flight_dispersion.export_ellipses_to_kml(filename="monte_carlo_dispersion.kml",
                                         origin_lat=launch_site.launch_site.latitude,
                                         origin_lon=launch_site.launch_site.longitude,
                                         type="all", # export both apogee and impact ellipses
                                         resolution=100 # number of points to be used to draw the ellipses
)