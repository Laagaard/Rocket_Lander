'''
DART Mission Planner

Sources:
- Equations of Motion: simulation\Missile Aerodynamics for Ascent and Reentry.pdf
- FBD: simulation\Missile Aerodynamics for Ascent and Reentry.pdf
'''

import numpy as np
import openmdao.api as om
import dymos as dm
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

class RocketMotor:
    def __init__(self, thrust_curve):
        self.thrust_curve = thrust_curve

AeroTech_G25W = RocketMotor(np.array([[0.0000, 00.0000], [0.0100, 38.0460],
                                     [0.0290, 21.7680], [0.1100, 30.0460],
                                     [0.2390, 35.7750], [0.4960, 40.6960],
                                     [0.5680, 42.1160], [0.5920, 45.4290],
                                     [0.7490, 46.1860], [0.9970, 45.7130],
                                     [1.2550, 44.1980], [1.4980, 39.9390],
                                     [1.9990, 31.4210], [2.5010, 23.0930],
                                     [3.0060, 14.1960], [3.5030, 07.2880],
                                     [3.9990, 04.0700], [4.3710, 01.9880],
                                     [4.4140, 00.0000]]))

print(AeroTech_G25W.thrust_curve.shape)

class RocketODE(om.ExplicitComponent):
    def initialize(self):
        self.options.declare('num_nodes', types=int)

    def setup(self):
        nn = self.options['num_nodes']

        # Outputs
        self.add_output('Psi_dot', shape=(nn,), desc="Yaw Euler angle rate", units="rad/s", tags=['dymos.state_rate_source:Psi', 'dymos.state_units:rad'])
        self.add_output('Theta_dot', shape=(nn,), desc="Pitch Euler angle rate", units="rad/s", tags=['dymos.state_rate_source:Theta', 'dymos.state_units:rad'])
        self.add_output("Phi_dot", shape=(nn,), desc="Roll Euler angle rate", units="rad/s", tags=['dymos.state_rate_source:Phi', 'dymos.state_units:rad'])

        self.add_output('P_dot', shape=(nn,), desc="Roll acceleration", units="rad/s**2", tags=['dymos.state_rate_source:P', 'dymos.state_units:rad/s'])
        self.add_output('Q_dot', shape=(nn,), desc="Pitch acceleration", units="rad/s**2", tags=['dymos.state_rate_source:G', 'dymos.state_units:rad/s'])
        self.add_output("R_dot", shape=(nn,), desc="Yaw acceleration", units="rad/s**2", tags=['dymos.state_rate_source:R', 'dymos.state_units:rad/s'])

        self.add_output("Inertial Acceleration", shape=(nn,3), desc="Inertial Acceleration", units="m/s**2", tags=['dymos.state_rate_source:Inertial Velocity', 'dymos.state_units:m/s'])

        # Inputs
        self.add_input('time', shape=(nn,), desc="Current simulation Time", units="s")
        self.add_input("Inertial Displacement", shape=(nn,3), desc="Inertial Displacement", units="m")
        self.add_input("Inertial Velocity", shape=(nn,3), desc="Inertial Velocity", units="m/s")

        self.add_input('M_x', shape=(nn,), desc="Roll moment", units="N*m")
        self.add_input('M_y', shape=(nn,), desc="Pitch moment", units="N*m")
        self.add_input('M_z', shape=(nn,), desc="Yaw moment", units="N*m")

        self.add_input('Mass', shape=(1,1), desc="Vehicle Mass", units="kg", tags=['dymos.static_target']) # TODO Mass is not constant
        self.add_input('CG', shape=(1,1), desc="Center of Gravity (relative to nose tip)", units="m", tags=['dymos.static_target']) # TODO CG changes position
        self.add_input('CP', shape=(nn,), desc="Center of Pressure (relative to nose tip)", units="m", tags=['dymos.static_target'])
        self.add_input('I_xx', shape=(nn,), desc="X-axis MMOI", units="kg*m**2", tags=['dymos.static_target'])
        self.add_input('I_yy', shape=(nn,), desc="Y-axis MMOI", units="kg*m**2", tags=['dymos.static_target'])
        self.add_input('I_zz', shape=(nn,), desc="Z-axis MMOI", units="kg*m**2", tags=['dymos.static_target'])

        self.add_input('P', shape=(nn,), desc="Roll rate", units="rad/s")
        self.add_input('Q', shape=(nn,), desc="Pitch rate", units="rad/s")
        self.add_input('R', shape=(nn,), desc="Yaw rate", units="rad/s")

        self.add_input('Psi', shape=(nn,), desc="Yaw Euler angle", units="rad")
        self.add_input('Theta', shape=(nn,), desc="Pitch Euler angle", units="rad")
        self.add_input('Phi', shape=(nn,), desc="Roll Euler angle", units="rad")

        self.add_input("Thrust Curve", shape=(19,2), desc="Ascent Motor Thrust Curve", units="N", tags=['dymos.static_target']) # TODO Determine shape dynamically
        self.add_input("Drag", shape=(nn,), desc="Aerodynamic Drag Force", units="N")
        self.add_input("Lift", shape=(nn,), desc="Aerodynamic Lift Force", units="N")

        self.add_input("Rho", shape=(1,1), desc="Atmospheric Density", units="kg/m**3", tags=['dymos.static_target'])

        self.add_input("Body Velocity", shape=(nn,3), desc="Velocity in Body Frame", units="m/s")
        self.add_input("Wind Velocity", shape=(1,1), desc="Wind Velocity", units="m/s", tags=['dymos.static_target']) # TODO Wind should vary at each node

        self.add_input("Body Force", shape=(nn,3), desc="Net Force in Body Frame", units="N")

        self.add_input("S", shape=(1,1), desc="Cross-Sectional Area", units="m**2", tags=['dymos.static_target'])

        self.add_input("C_L", shape=(1,1), desc="Lift Coefficient", tags=['dymos.static_target'])
        self.add_input("C_D", shape=(1,1), desc="Drag Coefficient", tags=['dymos.static_target'])

        self.declare_partials(of='*', wrt="*", method="fd")

    def compute(self, inputs, outputs):
        # Define inputs
        CG = inputs['CG'] # Center of gravity location (relative to nose tip) [m]
        CP = inputs['CP'] # Center of pressure location (relative to nose tip) [m]
        time = inputs['time'] # Simulation time [s]
        mass = inputs["Mass"] # Vehicle mass [s]
        M_x = inputs['M_x'] # Rolling moment [N*m]
        M_y = inputs['M_y'] # Pitching moment [N*m]
        M_z = inputs['M_z'] # Yawing moment [N*m]
        I_xx = inputs['I_xx'] # Mass moment of inetia w.r.t body x-axis
        I_yy = inputs['I_yy'] # Mass moment of inetia w.r.t body y-axis
        I_zz = inputs['I_zz'] # Mass moment of inetia w.r.t body z-axis
        P = inputs['P'] # Roll rate [rad/s]
        Q = inputs['Q'] # Pitch rate [rad/s]
        R = inputs['R'] # Yaw rate [rad/s]
        Psi = inputs['Psi'] # Yaw Euler angle [rad]
        Theta = inputs['Theta'] # Pitch Euler angle [rad]
        sin_theta = np.sin(Theta)
        cos_theta = np.cos(Theta)
        Phi = inputs['Phi'] # Roll Euler angle [rad]
        sin_phi = np.sin(Phi)
        cos_phi = np.cos(Phi)

        thrust_curve = inputs['Thrust Curve'] # Rocket motor thrust curve as `np.array` [[s], [N]]

        thrust = np.array([])
        for time_node in time:
            index = np.argmin(np.abs(time_node - thrust_curve[:, 0]))
            thrust = np.append(thrust, thrust_curve[index,1])

        rho =  inputs["Rho"]
        velocity = inputs["Body Velocity"] # shape=(nn,3)
        S = inputs["S"]
        C_D = inputs["C_D"]
        C_L = inputs["C_L"]
        drag = (0.5*rho*velocity[:,0]**2)*S*C_D # FIXME is C_D a constant value?

        wind_velocity = inputs["Wind Velocity"] # wind velocity [m/s]

        alpha = np.sin(wind_velocity/np.linalg.norm(velocity, axis=1)) - Theta # angle of attack [rad] (:= 0 when perfectly upright)

        lift = (0.5*rho*np.linalg.norm(velocity, axis=1)**2)*S*(2*np.pi*alpha) # FIXME how bad of an approximation is thin airfoil theory (i.e. `2*np.pi*alpha`)

        # TODO Why body force have weird shape despite being Dymos input?
        net_force_body = inputs['Body Force']
        net_force_body = np.array([thrust - drag, 0, lift]) # net force in BODY FRAME [N]

        net_acceleration_body = net_force_body/mass # net acceleration in BODY FRAME [m/s**2]

        static_margin = CP - CG # static margin [m] (i.e. lift force moment arm)

        M_y = net_force_body[2]*static_margin # pitching moment [N*m]

        outputs['P_dot'] = (1/I_xx)*(M_x - (I_zz - I_yy)*Q*R)
        outputs['Q_dot'] = (1/I_yy)*(M_y - (I_xx - I_zz)*P*R)
        outputs['R_dot'] = (1/I_zz)*(M_z - (I_yy - I_xx)*P*Q)

        outputs['Psi_dot'] = (Q*sin_phi + R*cos_phi)/cos_theta
        outputs['Theta_dot'] = Q*cos_phi - R*sin_phi
        outputs['Phi_dot'] = P + ((Q*sin_phi + R*cos_phi)/cos_theta)*sin_theta

        r = Rotation.from_euler('ZYX', np.array([Phi, Theta, Psi]).reshape(16,3), degrees=False)
        r = r.as_matrix()

        # Make ALL variable either inputs or outputs for Dymos

        outputs["Inertial Acceleration"] = net_acceleration_body*r

prob = om.Problem()

traj = dm.Trajectory()
prob.model.add_subsystem('traj', traj)

phase = dm.Phase(ode_class = RocketODE, transcription=dm.Radau(num_segments=4, solve_segments='forward'))
traj.add_phase('phase0', phase)

phase.add_state('P', rate_source='P_dot', targets=['P'], units="rad/s", input_initial=True, val=0)
phase.add_state('Q', rate_source='Q_dot', targets=['Q'], units="rad/s", input_initial=True, val=0)
phase.add_state('R', rate_source='R_dot', targets=['R'], units="rad/s", input_initial=True, val=0)

phase.add_state('Psi', rate_source='Psi_dot', targets=['Psi'], units="rad", input_initial=True, val=0)
phase.add_state('Theta', rate_source='Theta_dot', targets=['Theta'], units="rad", input_initial=True, val=0)
phase.add_state('Phi', rate_source='Phi_dot', targets=['Phi'], units="rad", input_initial=True, val=0)

phase.add_state("Inertial Displacement", rate_source="Inertial Velocity", targets=["Inertial Displacement"], units="m", input_initial=True, val=np.array([0,0,0]))
phase.add_state("Inertial Velocity", rate_source="Inertial Acceleration", targets=["Inertial Velocity"], units="m/s", input_initial=True, val=np.array([0,0,0]))

phase.add_parameter('Mass', units="kg", targets=['Mass'], static_target=True) # TODO Mass will not be constant
phase.add_parameter('CG', units="m", targets=['CG'], static_target=True) # TODO CG will change location
phase.add_parameter('CP', units="m", targets=['CP'], static_target=True)
phase.add_parameter('I_xx', units="kg*m**2", targets=['I_xx'], static_target=True)
phase.add_parameter('I_yy', units="kg*m**2", targets=['I_yy'], static_target=True)
phase.add_parameter('I_zz', units="kg*m**2", targets=['I_zz'], static_target=True)

phase.add_parameter("Rho", units="kg/m**3", targets=["Rho"], static_target=True)
phase.add_parameter("Wind Velocity", units="m/s", targets=["Wind Velocity"], static_target=True)

phase.add_parameter("S", units="m**2", targets=["S"], static_target=True)
phase.add_parameter("C_L", targets=["C_L"], static_target=True)
phase.add_parameter("C_D", targets=["C_D"], static_target=True)

phase.add_parameter("Thrust Curve", units="N", targets=["Thrust Curve"], static_target=True)

prob.setup()

phase.set_time_options('time', targets=['time'], initial_scaler=0.0, duration_scaler=15.0)

prob.set_val('traj.phase0.t_initial', 0.0)
prob.set_val('traj.phase0.t_duration', 15.0)

prob.set_val('traj.phase0.parameters:Mass', 1.5)
prob.set_val('traj.phase0.parameters:CG', 0.3048)
prob.set_val('traj.phase0.parameters:CP', 0.6096)
prob.set_val('traj.phase0.parameters:I_xx', 2.0)
prob.set_val('traj.phase0.parameters:I_yy', 1.0)
prob.set_val('traj.phase0.parameters:I_zz', 0.5)

prob.set_val('traj.phase0.parameters:Rho', 1.225)

'''
TODO either 1) randomize with `np.array([np.arange(0, 100, 1).reshape(100, 1), np.random.rand(100, 1)])`
         or 2) obtain real wind speeds from NOAA or some other source
'''
prob.set_val('traj.phase0.parameters:Wind Velocity', 0.0)

prob.set_val('traj.phase0.parameters:S', 0.00456) # [m**2] (circle area w/ radius = 3 in)
prob.set_val('traj.phase0.parameters:C_L', 0.25)
prob.set_val('traj.phase0.parameters:C_D', 0.025)

prob.set_val('traj.phase0.parameters:Thrust Curve', AeroTech_G25W.thrust_curve)

prob.run_model()

sim_out = traj.simulate(times_per_seg=50)

t_sol = prob.get_val('traj.phase0.timeseries.time')
t_sim = sim_out.get_val('traj.phase0.timeseries.time')

states = ['Inertial Displacement']
fig, axes = plt.subplots(len(states), 1)
for i, state in enumerate(states):
    sol = axes[i].plot(t_sol, prob.get_val(f'traj.phase0.timeseries.{state}'), 'o')
    sim = axes[i].plot(t_sim, sim_out.get_val(f'traj.phase0.timeseries.{state}'), '-')
    axes[i].set_ylabel(state)
axes[-1].set_xlabel('time (s)')
fig.legend((sol[0], sim[0]), ('solution', 'simulation'), loc='lower right', ncol=2)
plt.tight_layout()
plt.show()