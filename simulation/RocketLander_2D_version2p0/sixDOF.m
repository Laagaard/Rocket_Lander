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

function stateDot = sixDOF(t,state)
    
    % state(1)  -   x (m)
    % state(2)  -   xdot_body (m/s)
    % state(3)  -   xdot_inertial (m/s)
    % state(4)  -   y (m)
    % state(5)  -   ydot_body (m/s)
    % state(6)  -   ydot_inertial (m/s)
    % state(7)  -   z (m)
    % state(8)  -   zdot_body (m/s)
    % state(9)  -   zdot_inertial (m/s)
    % state(10)  -   psi (rad)
    % state(11)  -   theta (rad)
    % state(12)  -   phi (rad)
    % state(13)  -   P (rad/s)
    % state(14)  -   Q (rad/s)
    % state(15)  -   R (rad/s)
    % state(16)  -   m (kg)
    % state(17)  -   I_xx (kg*m^2)
    % state(18)  -   I_yy (kg*m^2)
    % state(19)  -   I_zz (kg*m^2)
    % state(20)  -   C_G (m)

    % ratesOptions(1)   -   are we firing descent motor?
    % ratesOptions(2)   -   are we using TVC?

% =========================================
% Import global varialbes
global thrustCurve ratesOptions rho S C_D C_P0 C_L_alpha g_accel t_fire burnTime
global propellantMass

% =========================================
% import states from integration
x               =   state(1);   % (m) x position - inertial frame
xdot_body       =   state(2);   % (m/s) x velocity - body frame
xdot_inertial   =   state(3);   % (m/s) x velocity - inertial frame
y               =   state(4);   % (m) y position - inertial frame
ydot_body       =   state(5);   % (m/s) y velocity - body frame
ydot_inertial   =   state(6);   % (m/s) y velocity - inertial frame
z               =   state(7);   % (m) z position - inertial frame
zdot_body       =   state(8);   % (m/s) z velocity - body frame
zdot_inertial   =   state(9);   % (m/s) z velocity - inertial frame
psi             =   state(10);  % (rad) heading (yaw) Euler angle
theta           =   state(11);  % (rad) pitch Euler angle
phi             =   state(12);  % (rad) roll Euler angle
launch_angle    =   state(13);  % (rad) launch angle from horizontal - inertial frame
P               =   state(14);  % (rad/s) x-axis (roll) angular rate
Q               =   state(15);  % (rad/s) y-axis (pitch) angular rate
R               =   state(16);  % (rad/s) z-axis (yaw) angular rate
m               =   state(17);  % (kg)
I_xx            =   state(18);  % (kg)
I_yy            =   state(19);  % (kg)
I_zz            =   state(20);  % (kg)
C_G             =   state(21);  % (m)

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

% Evaluate Lift
alpha = atan2(sqrt(ydot_body^2 + zdot_body^2), xdot_body); % (rad) angle of attack
Lift = 1/2 * rho * (xdot_body^2 + ydot_body^2 + zdot_body^2) * S * C_L_alpha*alpha; % (N)

% Evaluate Drag
Drag = 1/2 * rho * (xdot_body^2 + ydot_body^2 + zdot_body^2) * S * C_D; % (N)

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

% =========================================
% Sum forces to evaluate moments
lift_body = [Lift*sin(alpha); 0; -Lift*cos(alpha)]; % (N) lift expressed in body coordinates
drag_body = [-Drag*cos(alpha); 0; -Drag*sin(alpha)]; % (N) drag expressed in body coordinates
f_aero_body = [lift_body(1) + drag_body(1);
               0;
               lift_body(3) + drag_body(3)]; % (N) net aerodynamic force expressed in body coordinates
% F_aero_X_body = lift_body(1) + drag_body(1); % (N) net force acting on rocket CP along x-body axis
% F_aero_Y_body = 0; % NEEDS TO BE POPULATED (AEROYNAMIC FORCES) (N) net force acting on rocket CP along z-body axis
% F_aero_Z_body = lift_body(3) + drag_body(3); % (N) net force acting on rocket CP along z-body axis

linear_velocities_inertial = [xdot_inertial ydot_inertial zdot_inertial]'; % (m/s) linear velocities in inertial frame
if (norm(linear_velocities_inertial) == 0)
    inertial_pitch_angle = launch_angle; % [rad] pitch angle in inertial frame
else
    inertial_pitch_angle = atan2(zdot_inertial, sqrt(xdot_inertial^2 + ydot_inertial^2)); % [rad] pitch angle in inertial frame
end

ascentThrust_body = [ascentThrust 0 0]'; % (N) ascent thrust vector in body axes
gravity_body = [-m*g_accel*sin(inertial_pitch_angle) 0 m*g_accel*cos(inertial_pitch_angle)]'; % (N) gravitational force vector in body axes

f_CG_body = ascentThrust_body + gravity_body; % (N) net force acting on rocket CG in body coordinates

% Calculate normal force from launch pad
if (t < burnTime && norm(ascentThrust_body*sin(inertial_pitch_angle)) < norm(gravity_body))
    launch_pad_normal = -f_CG_body; % (N) normal force of the launch pad on the rocket in body frame
else
    launch_pad_normal = zeros(size(f_CG_body)); % (N) normal force of the launch pad on the rocket in body frame
end

% Sum forces to evaluate accelerations
xddot_body = (f_aero_body(1) + f_CG_body(1) + launch_pad_normal(1))/m; % (m/s^2)
yddot_body = 0; % NEEDS TO BE POPULATED (AEROYNAMIC FORCES) (m/s^2)
zddot_body = (f_aero_body(3) + f_CG_body(3) + launch_pad_normal(3))/m;

% Accelerations in Body Frame
linear_accelerations_body = [xddot_body yddot_body zddot_body];

% F_CG_X = ascentThrust - m*g_accel*cos(theta); % (N) net force acting on rocket CG along x-body axis
% F_CG_Z = m*g_accel*sin(theta); % (N) net force acting on rocket CG along z-body axis

x_bar = [C_G - C_P0; 0; 0]; % (m) static margin (distance between CP and CG) in body coordinates
% M_x = 0; % (N*m) moment along body x axis
M = cross(x_bar, f_aero_body); % (N*m) net moment on the rocket in body coordinates
% M_z = 0; % (N*m) moment along body z axis

psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta); % (rad/s) heading/yaw angle rate
theta_dot = Q*cos(phi) - R*sin(phi); % (rad/s) attitude angle rate
phi_dot = P + (Q*sin(phi) + R*cos(phi))*tan(theta); % (rad/s) bank/roll angle rate

% fprintf("t = %.2f s, alpha = %.4f rad, psi = %.4f rad, theta = %.2f rad, phi = %.4f rad, P = %.2f rad/s, Q = %.2f rad/s, R = %.2f rad/s\n", t, alpha, psi, theta, phi, P, Q, R)
% fprintf("xdot_body = %.4f m/s, ydot_body = %.4f m/s, zdot_body = %.4f m/s\n", xdot_body, ydot_body, zdot_body)
% fprintf("xdot_inertial = %.4f m/s, ydot_inertial = %.4f m/s, zdot_inertial = %.4f m/s\n", xdot_inertial, ydot_inertial, zdot_inertial)
% fprintf("psi_dot = %.4f rad/s, theta_dot = %.4f rad/s, phi_dot = %.4f rad/s\n\n", psi_dot, theta_dot, phi_dot)

P_dot = (M(1) - (I_zz - I_yy)*Q*R)/I_xx; % (rad/s)
Q_dot = (M(2) - (I_xx - I_zz)*P*R)/I_yy; % (rad/s)
R_dot = (M(3) - (I_yy - I_xx)*P*Q)/I_zz; % (rad/s)

% Body CS to Inertial CS Transformation
q_inertial_2_body = quaternion(eul2quat([psi theta phi], "ZYX")); % quaternion representing inertial-to-body coordinate transformation
q_body_2_inertial = conj(q_inertial_2_body); % quaternion representing body-to-inertial coordinate transformation

% Accelerations in Inertial Frame
linear_accelerations_inertial = quatrotate(q_body_2_inertial, linear_accelerations_body);

xddot_inertial = linear_accelerations_inertial(1);
yddot_inertial = linear_accelerations_inertial(2);
zddot_inertial = linear_accelerations_inertial(3);

pause(0.01);
fprintf("t = %.4f s, eulAngs = [%.7f %.7f %.7f] rad, omega = [%.7f %.7f %.7f] rad/s, inertial_accels = [%.7f %.7f %.7f] m/s^2, body_accels = [%.7f %.7f %.7f] m/s^2 \n\n", t, psi, theta, phi, P, Q, R, linear_accelerations_inertial, linear_accelerations_body)
% fprintf("t = %.4f s, alpha = %.7f rad, inertial pitch angle = %.7f rad, ",  t, alpha, inertial_pitch_angle)
% fprintf("f_CG_body = [%.4f %.4f %.4f] N, inertial accels = [%.7f %.7f %.7f] m/s^2, inertial vels = [%.4f %.4f %.4f] m/s, \n\n", f_CG_body, linear_accelerations_inertial, xdot_inertial, ydot_inertial, zdot_inertial)
% fprintf("body accels = [%.4f %.4f %.4f] m/s^2, body vels = [%.7f %.7f %.7f] m/s, position = [%.4f %.4f %.4f] m\n\n", linear_accelerations_body, xdot_body, ydot_body, zdot_body, x, y, z)
% fprintf("q = [%.4f %.4f %.4f %.4f], eulAngs = [%.7f %.7f %.7f] rad, M = [%.7f %.7f %.7f] Nm, omega = [%.4f %.4f %.4f] rad/s\n\n", q, psi, theta, phi, M, P, Q, R)

% =========================================
% Final state derivative vector
% fprintf("state = [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]\n", state)
stateDot = [xdot_inertial xddot_body xddot_inertial ydot_inertial yddot_body yddot_inertial zdot_inertial zddot_body zddot_inertial psi_dot theta_dot phi_dot 0 P_dot Q_dot R_dot mdot 0 0 0 0]';
% fprintf("stateDot = [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]\n\n", stateDot)