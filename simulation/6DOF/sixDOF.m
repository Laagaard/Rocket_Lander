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

% ratesOptions(1)   -   are we firing descent motor?
% ratesOptions(2)   -   are we using TVC?

% =========================================
% Import global varialbes
global thrustCurve ratesOptions rho S C_D C_P0 C_L_alpha g_accel t_fire burnTime
global propellantMass

% =========================================
% import states from integration
x                       =   state(1);   % (m) x position - inertial frame
xdot_body               =   state(2);   % (m/s) x velocity - body frame
xdot_inertial           =   state(3);   % (m/s) x velocity - inertial frame
y                       =   state(4);   % (m) y position - inertial frame
ydot_body               =   state(5);   % (m/s) y velocity - body frame
ydot_inertial           =   state(6);   % (m/s) y velocity - inertial frame
z                       =   state(7);   % (m) z position - inertial frame
zdot_body               =   state(8);   % (m/s) z velocity - body frame
zdot_inertial           =   state(9);   % (m/s) z velocity - inertial frame
psi                     =   state(10);  % (rad) heading (yaw) Euler angle
theta                   =   state(11);  % (rad) pitch Euler angle
phi                     =   state(12);  % (rad) roll Euler angle
inertial_pitch_angle    =   state(13);  % (rad) launch angle from horizontal - inertial frame
alpha                   =   state(14);  % (rad) angle of attack (angle between rocket velocity vector and rocket centerline)
beta                    =   state(15);  % (rad) angle of sideslip (yaw equivalent to angle of attack)
P                       =   state(16);  % (rad/s) x-axis (roll) angular rate
Q                       =   state(17);  % (rad/s) y-axis (pitch) angular rate
R                       =   state(18);  % (rad/s) z-axis (yaw) angular rate
m                       =   state(19);  % (kg)
I_xx                    =   state(20);  % (kg)
I_yy                    =   state(21);  % (kg)
I_zz                    =   state(22);  % (kg)
C_G                     =   state(23);  % (m)

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

linear_velocities_body = [xdot_body, ydot_body, zdot_body]; % (m/s)

% Evaluate Lift
% alpha = atan2(zdot_body, xdot_body); % (rad) angle of attack (DOES NOT ACCOUNT FOR YDOT_BODY)
Lift = 1/2 * rho * norm(linear_velocities_body)^2 * S * C_L_alpha*abs(alpha); % (N)

% Evaluate Drag
Drag = 1/2 * rho * norm(linear_velocities_body)^2 * S * C_D; % (N)

% =========================================
% Sum forces to evaluate moments
lift_body = [Lift*abs(sin(alpha)); 0; Lift*-sign(alpha)*cos(alpha)]; % (N) lift expressed in body coordinates
drag_body = [Drag*(-cos(alpha)); 0; Drag*(-sin(alpha))]; % (N) drag expressed in body coordinates
f_aero_body = [lift_body(1) + drag_body(1);
               0;
               lift_body(3) + drag_body(3)]; % (N) net aerodynamic force expressed in body coordinates

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
zddot_body = (f_aero_body(3) + f_CG_body(3) + launch_pad_normal(3))/m; % (m/s^2)

% Accelerations in Body Frame
linear_accelerations_body = [xddot_body yddot_body zddot_body];

x_bar = [C_G - C_P0; 0; 0]; % (m) static margin (distance between CP and CG) in body coordinates)
M = cross(x_bar, f_aero_body); % (N*m) net moment on the rocket in body coordinates

psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta); % (rad/s) heading/yaw angle rate
theta_dot = Q*cos(phi) - R*sin(phi); % (rad/s) attitude angle rate
phi_dot = P + (Q*sin(phi) + R*cos(phi))*tan(theta); % (rad/s) bank/roll angle rate

if (linear_velocities_body(1) ~= 0)
    % Rate of change of angle of attack
    alpha_dot = (1/(norm(linear_velocities_body))*cos(beta))*(-sin(alpha)*(gravity_body(1) + f_aero_body(1) + ascentThrust_body(1)) + cos(alpha)*(gravity_body(3) + f_aero_body(3) + ascentThrust_body(3))) + Q - (P*cos(alpha) + R*sin(alpha))*tan(beta);
    % alpha_dot = cos(alpha)^2*(linear_accelerations_body(3)/linear_velocities_body(1) - P*tan(beta) + Q*sec(alpha)^2 - (linear_accelerations_body(1)*tan(alpha))/linear_velocities_body(1) - R*tan(alpha)*tan(beta)); % (rad/s)

    % Rate of change of angle of sidesip
    beta_dot = (1/norm(linear_velocities_body))*(-cos(alpha)*sin(beta)*(gravity_body(1) + f_aero_body(1) + ascentThrust_body(1)) + cos(beta)*(gravity_body(2) + f_aero_body(2) + ascentThrust_body(2)) - sin(alpha)*sin(beta)*(gravity_body(3) + f_aero_body(3) + ascentThrust_body(3))) + P*sin(alpha) - R*cos(alpha);
    % beta_dot = cos(beta)^2*(linear_accelerations_body(2)/linear_velocities_body(1) - R*sec(beta)^2 + P*tan(alpha) - (linear_accelerations_body(1)*tan(beta))/linear_velocities_body(1) + Q*tan(alpha)*tan(beta)); % (rad/s)
else
    alpha_dot = 0;
    beta_dot = 0;
end

P_dot = (M(1) - (I_zz - I_yy)*Q*R)/I_xx; % (rad/s^2)
Q_dot = (M(2) - (I_xx - I_zz)*P*R)/I_yy; % (rad/s^2)
R_dot = (M(3) - (I_yy - I_xx)*P*Q)/I_zz; % (rad/s^2)

fprintf("t = %.4f s, alpha_dot = %.4f rad/s, v_b = [%.7f %.7f %.7f] m/s \n", t, alpha_dot, linear_velocities_body)

% Body CS to Inertial CS Transformation
q_inertial_2_body = quaternion(eul2quat([psi theta phi], "ZYX")); % quaternion representing inertial-to-body coordinate transformation
q_body_2_inertial = conj(q_inertial_2_body); % quaternion representing body-to-inertial coordinate transformation

% animate_stl([x y z], [xdot_body ydot_body zdot_body], q_body_2_inertial)

% Accelerations in Inertial Frame
linear_accelerations_inertial = quatrotate(q_body_2_inertial, linear_accelerations_body);

xddot_inertial = linear_accelerations_inertial(1);
yddot_inertial = linear_accelerations_inertial(2);
zddot_inertial = linear_accelerations_inertial(3);

% =========================================
% Final state derivative vector
stateDot = [xdot_inertial, xddot_body, xddot_inertial, ydot_inertial, yddot_body, ...
            yddot_inertial, zdot_inertial, zddot_body, zddot_inertial, psi_dot, ...
            theta_dot, phi_dot, theta_dot, alpha_dot, beta_dot, ...
            P_dot, Q_dot, R_dot, mdot, 0, 0, 0, 0]';
