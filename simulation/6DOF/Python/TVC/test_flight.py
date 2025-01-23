import sys
sys.path.append('../')
import setup

from rocketpy import Flight

test_flight = Flight(
    rocket = setup.DART_rocket, 
    environment = setup.launch_site,
    rail_length = 1.5, #TBR (m)
    inclination = 90, #TBR (deg)
    heading = 0, #(deg)
    time_overshoot = True
)

test_flight.plots.trajectory_3d()