%{
6DOF Equations of Motion
Project: DART

------------ Inertia Tensor ------------------------
I = [I_xx I_xy I_xz; I_xy I_yy I_yz; I_xz I_yz I_zz]

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
Output: psi (heading/yaw angle, rotation about body Z axis),
        theta (attitude angle, rotation about body y axis),
        phi (bank/roll angle, rotation about body x axis)
Rotation Order: 3-2-1 (i.e., rotation about body z axis by psi,
                             rotation about body y axis by theta,
                             rotation about body x axis by phi)
%}

function stateDot = sixDOF(t,state)
    
    % state(1)  -   x (m)
    % state(2)  -   xdot (m/s)
    % state(3)  -   y (m)
    % state(4)  -   ydot (m/s)
    % state(5)  -   z (m)
    % state(6)  -   zdot (m/s)
    % state(7)  -   psi (rad)
    % state(8)  -   theta (rad)
    % state(9)  -   phi (rad)
    % state(10) -   P (rad/s)
    % state(11) -   Q (rad/s)
    % state(12) -   R (rad/s)
    % state(13) -   m (kg)
    % state(14) -   I_xx (kg*m^2)
    % state(15) -   I_yy (kg*m^2)
    % state(16) -   I_zz (kg*m^2)

    % ratesOptions(1)   -   are we firing descent motor?
    % ratesOptions(2)   -   are we using TVC?

% =========================================
% Import global varialbes
global thrustCurve ratesOptions rho S C_D C_P0 C_L_alpha g_accel t_fire burnTime
global propellantMass

% =========================================
% import states from integration
x       =   state(1);   % (m)
xdot    =   state(2);   % (m/s)
y       =   state(3);   % (m)
ydot    =   state(4);   % (m/s)
z       =   state(5);   % (m)
zdot    =   state(6);   % (m/s)
psi     =   state(7);   % (rad)
theta   =   state(8);   % (rad)
phi     =   state(9);   % (rad)
P       =   state(10);  % (rad/s)
Q       =   state(11);  % (rad/s)
R       =   state(12);  % (rad/s)
m       =   state(13);  % (kg)
I_xx    =   state(14);  % (kg)
I_yy    =   state(15);  % (kg)
I_zz    =   state(16);  % (kg)
C_G     =   state(17);  % (m)

% Determine which engines are firing and evaluate thrust curve
ascentThrust = interp1(thrustCurve(:,1),thrustCurve(:,2),t,'linear',0); % (N)
descentThrust = zeros(size(ascentThrust)); % (N)
if ratesOptions(1) == true
    % descent thrust is pointing opposite direction of ascent thrust
    % (relative to rocket), so it is negative
    descentThrust = - interp1(thrustCurve(:,1),thrustCurve(:,2),t-t_fire,'linear',0); % (N)
    Thrust = ascentThrust + descentThrust; % (N)
else
    Thrust = ascentThrust;
end

% Evaluate Lift
alpha = theta - atan2(zdot, sqrt(xdot^2 + ydot^2)); % (rad) angle of attack
Lift = 1/2 * rho * (xdot^2 + ydot^2) * S * C_L_alpha*alpha; % (N)

% Evaluate Drag
Drag = 1/2 * rho * (xdot^2 + ydot^2) * S * C_D; % (N)

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

f_CG_body = [ascentThrust - m*g_accel*sin(theta);
             0;
             m*g_accel*cos(theta)]; % (N) net force acting on rocket CG in body coordinates
% F_CG_X = ascentThrust - m*g_accel*cos(theta); % (N) net force acting on rocket CG along x-body axis
% F_CG_Z = m*g_accel*sin(theta); % (N) net force acting on rocket CG along z-body axis

x_bar = [C_P0 - C_G; 0; 0]; % (m) static margin (distance between CP and CG) in body coordinates
% M_x = 0; % (N*m) moment along body x axis
M = cross(f_aero_body, x_bar); % (N*m) net moment on the rocket in body coordinates
% M_z = 0; % (N*m) moment along body z axis

psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta); % (rad/s)
theta_dot = Q*cos(phi) - R*sin(phi); % (rad/s)
phi_dot = P + (Q*sin(phi) + R*cos(phi))*tan(theta); % (rad/s)
fprintf("alpha = %.2f\n", alpha)

P_dot = (M(1) - (I_zz - I_yy)*Q*R)/I_xx; % (rad/s)
Q_dot = (M(2) - (I_xx - I_zz)*P*R)/I_yy; % (rad/s)
R_dot = (M(3) - (I_yy - I_xx)*P*Q)/I_zz; % (rad/s)

% Body CS to Inertial CS Transformation
q = eul2quat([psi theta phi], "ZYX"); % quaternion representing inertial-to-body coordinate transformation

% Sum forces to evaluate accelerations
xddot_body = (f_aero_body(1) + f_CG_body(1))/m; % (m/s^2)
yddot_body = 0; % NEEDS TO BE POPULATED (AEROYNAMIC FORCES) (m/s^2)
zddot_body = (f_aero_body(3) + f_CG_body(3))/m;

% Transform accelerations from body CS to inertial CS
linear_accelerations_body = [xddot_body yddot_body zddot_body];
linear_accelerations_inertial = quatrotate(q, linear_accelerations_body);

xddot_inertial = linear_accelerations_inertial(1);
yddot_inertial = linear_accelerations_inertial(2);
zddot_inertial = linear_accelerations_inertial(3);

% =========================================
% Final state derivative vector
stateDot = [xdot xddot_inertial ydot yddot_inertial zdot zddot_inertial psi_dot theta_dot phi_dot P_dot Q_dot R_dot mdot 0 0 0 0]';