function Parameters_Rocket
    % Specifies rocket dimensions, mass, and other relevant constants
    
    % =================================
    % 3 inch by 3 ft cardboard tube rocket

    global x0 xdot0 y0 ydot0 theta0 thetaDot0 m0 C_D S
    
    x0 = 0; % (m)
    xdot0 = 0; % (m/s)
    y0 = 0; % (m)
    ydot0 = 0; % (m/s)
    theta0 = 90; % (degrees)
    thetaDot0 = 0; % (degrees/s)
    m0 = 1.50; % kg
    
    m_tube = 0.1135 * (3.00); % (kg)
    C_D = 0.75; % (unitless)
    S = pi * ((3.1*25.4/1000)/2)^2; % (m^2)
end