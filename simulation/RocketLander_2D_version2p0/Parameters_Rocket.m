function Parameters_Rocket
    % Specifies rocket dimensions, mass, and other relevant constants
    
    % =================================
    % 3 inch by 3 ft cardboard tube rocket

    global x0 xdot0 y0 ydot0 z0 zdot0 psi0 theta0 phi0 P0 Q0 R0 m0 I_xx0 I_yy0 I_zz0 C_G0 C_P0 C_L_alpha thetaDot0 C_D S 
    
    x0 = 0; % (m)
    xdot0 = 0; % (m/s)
    y0 = 0; % (m)
    ydot0 = 0; % (m/s)
    z0 = 0; % (m)
    zdot0 = 0; % (m/s)
    psi0 = 0; % (rad)
    theta0 = pi/2; % (rad)
    phi0 = 0; % (rad)
    P0 = 0; % (rad/s)
    Q0 = 0; % (rad/s)
    R0 = 0; % (rad/s)
    m0 = 1.50; % (kg)
    I_xx0 = 0.004784; % (kg*m^2) longitudinal axis
    I_yy0 = 0.273645; % (kg*m^2) 
    I_zz0 = 0.273654; % (kg*m^2)
    C_G0 = 0.4; % [m] initial CG location (measured from tip of nose) TOTALLY WRONG AND NEEDS TO BE CHANGED (JUST A PLACEHOLDER)
    C_P0 = 0.656; % [m] initial CP location (measured from tip of nose)
    C_L_alpha = 0.01; % (1/rad) TOTALLY WRONG AND NEEDS TO BE CHANGED (JUST A PLACEHOLDER)
    thetaDot0 = 0; % (degrees/s)
    
    m_tube = 0.1135 * (3.00); % (kg)
    C_D = 0.75; % (unitless)
    S = pi * ((3.1*25.4/1000)/2)^2; % (m^2)
end