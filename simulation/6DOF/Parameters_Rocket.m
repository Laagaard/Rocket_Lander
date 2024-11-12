function Parameters_Rocket
    % Specifies rocket dimensions, mass, and other relevant constants
    
    % =================================
    % 3 inch by 3 ft cardboard tube rocket

    global x0 xdot0_body xdot0_inertial y0 ydot0_body ydot0_inertial z0 zdot0_body zdot0_inertial 
    global psi0 theta0 phi0 alpha0 beta0 P0 Q0 R0 m0 I_xx0 I_yy0 I_zz0 C_G0 C_P0 C_L_alpha thetaDot0 C_D S 
    
    x0 = 0; % (m)
    xdot0_body = 0; % (m/s)
    xdot0_inertial = 0; % (m/s)
    y0 = 0; % (m)
    ydot0_body = 0; % (m/s)
    ydot0_inertial = 0; % (m/s)
    z0 = 0; % (m)
    zdot0_body = 0; % (m/s)
    zdot0_inertial = 0; % (m/s)
    eulAngles = deg2rad([-90 -90.01 0]); % (rad) yaw angle, pitch angle, roll angle
    psi0 = eulAngles(1); % (rad) yaw angle
    theta0 = eulAngles(2); % (rad) pitch angle
    phi0 = eulAngles(3); % (rad) roll angle
    alpha0 = 0; % (rad) angle of attack
    beta0 = 0; % (rad) angle of sideslip
    P0 = 0; % (rad/s)
    Q0 = 0; % (rad/s)
    R0 = 0; % (rad/s)
    m0 = 1.50; % (kg)
    I_xx0 = 0.004784; % (kg*m^2) longitudinal axis
    I_yy0 = 0.273645; % (kg*m^2) 
    I_zz0 = 0.273654; % (kg*m^2)
    C_G0 = 611.908E-3; % [m] initial CG location (measured from tip of nose)
    C_P0 = 0.656; % [m] initial CP location (measured from tip of nose)
    C_L_alpha = 10; % (1/rad) TOTALLY WRONG AND NEEDS TO BE CHANGED (JUST A PLACEHOLDER)

    thetaDot0 = 0; % (degrees/s)
    
    m_tube = 0.1135 * (3.00); % (kg)
    C_D = 0.75; % (unitless)
    S = pi * ((3.1*25.4/1000)/2)^2; % (m^2)
end