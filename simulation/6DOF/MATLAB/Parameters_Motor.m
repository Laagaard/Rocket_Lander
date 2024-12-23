function Parameters_Motor
    % Specifies Motor thrust curve, mass, and propellant mass
    
    global thrustCurve propellantMass burnTime
    
    % =====================================
    % Aerotech G25W data
    
    % Load Aerotech G25W motor thrust curve
    thrustCurve = load('Aerotech_G25W.mat');
    % Convert loaded structure to array
    thrustCurve = struct2array(thrustCurve); % (s,N)
    propellantMass = 0.0625; % (kg)
    burnTime = thrustCurve(end,1); % (s)
    momentArm_ascentMotor = 0.45; % (m)
    % ======================================
end