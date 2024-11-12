%{
6DOF Equations of Motion

------------ Inertia Tensor ------------------------
I = [I_xx I_xy I_xz;
     I_xy I_yy I_yz;
     I_xz I_yz I_zz]

------------- Angular Velocity ---------------------
omega = [P; Q; R]

----------------- 1 ----------------

Input: Moments in body C.S
M_x = I_xx*P_dot + (I_zz - I_yy)*Q*R
M_y = I_yy*Q_dot + (I_xx - I_zz)*P*R
M_z = I_zz*R_dot + (I_yy - I_xx)*P*Q
Output: P (angular rate about body x axis),
        Q (angular rate about body y axis),
        R (angular rate about body z axis)

----------------- 2 ----------------

Input: P, Q, R (angular rates)
psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta)
theta_dot = Q*cos(phi) - R*sin(phi)
phi_dot = P + (Q*sin(phi) + R*cos(phi))*tan(theta)
Output: psi_dot (heading/yaw angle rate, rotation about body Z axis),
        theta_dot (attitude angle rate, rotation about body y axis),
        phi_dot (bank/roll angle rate, rotation about body x axis)
Rotation Order: 3-2-1 (i.e., rotation about body z axis by psi,
                             rotation about body y axis by theta,
                             rotation about body x axis by phi)
%}

function stateDot = sixDOF_v2(t,state)

% =========================================
% Import global varialbes
global thrustCurve ratesOptions rho S C_D C_P0 C_L_alpha g_accel t_fire burnTime
global propellantMass

% =========================================
% import states from integration
x                       =   state(1);   % (m) x position - inertial frame
xdot_inertial           =   state(2);   % (m/s) x velocity - inertial frame
y                       =   state(3);   % (m) y position - inertial frame
ydot_inertial           =   state(4);   % (m/s) y velocity - inertial frame
z                       =   state(5);   % (m) z position - inertial frame
zdot_inertial           =   state(6);   % (m/s) z velocity - inertial frame
psi                     =   state(7);  % (rad) heading (yaw) Euler angle
theta                   =   state(8);  % (rad) pitch Euler angle
phi                     =   state(9);  % (rad) roll Euler angle
P                       =   state(10);  % (rad/s) x-axis (roll) angular rate
Q                       =   state(11);  % (rad/s) y-axis (pitch) angular rate
R                       =   state(12);  % (rad/s) z-axis (yaw) angular rate
m                       =   state(13);  % (kg)
I_xx                    =   state(14);  % (kg)
I_yy                    =   state(15);  % (kg)
I_zz                    =   state(16);  % (kg)
C_G                     =   state(17);  % (m)

% Determine which engines are firing and evaluate thrust curve
ascentThrust = interp1(thrustCurve(:,1),thrustCurve(:,2),t,'linear',0); % (N)
descentThrust = 0; % (N)
if ratesOptions(1) == true
    % descent thrust is pointing opposite direction of ascent thrust
    % (relative to rocket), so it is negative
    descentThrust = - interp1(thrustCurve(:,1),thrustCurve(:,2),t-t_fire,'linear',0); % (N)
    Thrust = ascentThrust + descentThrust; % (N)
else
    Thrust = ascentThrust;
end

% Mass flow from motor
if ratesOptions(1)==true % if firing descent motor
    if (t<=burnTime)
        mdot = - (propellantMass/burnTime);        % (kg/s)
    elseif (t>burnTime)||(t>=t_fire)||((t-t_fire)<=burnTime)
        mdot = - (propellantMass/burnTime);        % (kg/s)
    else
        mdot = 0;                                % (kg/s)
    end
else % if not firing descent motor
    if (t<=burnTime)
        mdot = - (propellantMass/burnTime);        % (kg/s)
    else
        mdot = 0;                                % (kg/s)
    end
end

% Body CS to Inertial CS Transformation
q_inertial_2_body = quaternion(eul2quat([psi theta phi], "ZYX")); % quaternion representing inertial-to-body coordinate transformation
q_body_2_inertial = conj(q_inertial_2_body); % quaternion representing body-to-inertial coordinate transformation

% Velocities in body frame
linear_velocities_body = quatrotate(q_inertial_2_body, [xdot_inertial, ydot_inertial, zdot_inertial]);
xdot_body = linear_velocities_body(1);
ydot_body = linear_velocities_body(2);
zdot_body = linear_velocities_body(3);

% Evaluate Lift
alpha = atan2(zdot_body, xdot_body); % (rad) angle of attack (DOES NOT ACCOUNT FOR YDOT_BODY)
Lift = 0;%1/2 * rho * norm(linear_velocities_body)^2 * S * C_L_alpha*abs(alpha); % (N)

% Evaluate Drag
Drag = 0;%1/2 * rho * norm(linear_velocities_body)^2 * S * C_D; % (N)

% =========================================
% Sum forces to evaluate moments
lift_body = [Lift*sin(alpha); 0; Lift*cos(alpha)]; % (N) lift expressed in body coordinates
drag_body = [Drag*(-cos(alpha)); 0; Drag*(-sin(alpha))]; % (N) drag expressed in body coordinates
f_aero_body = [lift_body(1) + drag_body(1);
               0;
               lift_body(3) + drag_body(3)]; % (N) net aerodynamic force expressed in body coordinates

ascentThrust_body = [ascentThrust 0 0]'; % (N) ascent thrust vector in body axes

inertial_pitch_angle = pi/2 - abs(pi/2 - abs(theta)); % [rad]
gravity_body = [-m*g_accel*sin(inertial_pitch_angle), 0, m*g_accel*cosd(rad2deg(inertial_pitch_angle))]'; % (N) gravitational force vector in body axes

f_CG_body = ascentThrust_body + gravity_body; % (N) net force acting on rocket CG in body coordinates

% Calculate normal force from launch pad
if (norm(ascentThrust_body*sin(inertial_pitch_angle)) < norm(gravity_body) && z == 0)
    launch_pad_normal = -f_CG_body; % (N) normal force of the launch pad on the rocket in body frame
else
    launch_pad_normal = zeros(size(f_CG_body)); % (N) normal force of the launch pad on the rocket in body frame
end

% Sum forces to evaluate accelerations
xddot_body = (f_aero_body(1) + f_CG_body(1) + launch_pad_normal(1))/m; % (m/s^2)
yddot_body = 0; % NEEDS TO BE POPULATED (AEROYNAMIC FORCES) (m/s^2)
zddot_body = (f_aero_body(3) + f_CG_body(3) + launch_pad_normal(3))/m; % (m/s^2)

% Accelerations in Body Frame
linear_accelerations_body = [xddot_body yddot_body zddot_body];

x_bar = [C_G - C_P0; 0; 0]; % (m) static margin (distance between CP and CG) in body coordinates)
M = cross(x_bar, f_aero_body); % (N*m) net moment on the rocket in body coordinates

psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta); % (rad/s) heading/yaw angle rate
theta_dot = Q*cos(phi) - R*sin(phi); % (rad/s) attitude angle rate
phi_dot = P + (Q*sin(phi) + R*cos(phi))*tan(theta); % (rad/s) bank/roll angle rate

P_dot = (M(1) - (I_zz - I_yy)*Q*R)/I_xx; % (rad/s)
Q_dot = (M(2) - (I_xx - I_zz)*P*R)/I_yy; % (rad/s)
R_dot = (M(3) - (I_yy - I_xx)*P*Q)/I_zz; % (rad/s)

% animate_stl([x y z], [xdot_body ydot_body zdot_body], q_body_2_inertial)

% Accelerations in Inertial Frame
linear_accelerations_inertial = quatrotate(q_body_2_inertial, linear_accelerations_body);

fprintf("t = %.4f s, a_N = [%.15f %.15f %.15f], inertial_pos = [%.15f %.15f %.15f] m\n", t, linear_accelerations_inertial, x, y, z)

xddot_inertial = linear_accelerations_inertial(1);
yddot_inertial = linear_accelerations_inertial(2);
zddot_inertial = linear_accelerations_inertial(3);

% =========================================
% Final state derivative vector
stateDot = [xdot_inertial, xddot_inertial, ydot_inertial, yddot_inertial, zdot_inertial, zddot_inertial, psi_dot, theta_dot, phi_dot, P_dot, Q_dot, R_dot, mdot, 0, 0, 0, 0]';