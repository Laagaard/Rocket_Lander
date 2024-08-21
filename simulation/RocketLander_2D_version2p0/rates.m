function stateDot = rates(t,state)
    
    % RATES function provides state derivative for trajectory propagation
    
    % state(1)  -   x (m)
    % state(2)  -   xdot (m/s)
    % state(3)  -   y (m)
    % state(4)  -   ydot (m/s)
    % state(5)  -   theta (deg)
    % state(6)  -   thetaDot (deg/s)
    % state(7)  -   m (kg)
    
    % ratesOptions(1)   -   are we firing descent motor?
    % ratesOptions(2)   -   are we using TVC?
    
    % =========================================
    % Import global varialbes
    global thrustCurve ratesOptions rho S C_D g_accel t_fire burnTime
    global propellantMass theta0

    % ==========================================
    % import states from integration
    
        x = state(1);       % (m)
        xdot = state(2);    % (m/s)
        y = state(3);       % (m)
        ydot = state(4);    % (m/s)
        m = state(7);       % (kg)
    




    % ==========================================
    % Conditionally set pitch angle and rate depending on whether the  
    % descent motor has fired yet or not. If not fired yet (or not firing at
    % all), theta set by velocity direction and thetaDot set to zero (can go
    % back in later and calculate if needed).

        if ratesOptions(1)==true            % if firing descent motor
            if (t<t_fire)||(t<=burnTime)
                % if motor not fired yet and still burning first motor
                theta = theta0;  % (deg)
                thetaDot = 0;    % (deg/s)
            elseif (t<t_fire)
                % else, calculate from velocity direction
                theta = atan2d(ydot,xdot);   % (deg)
                thetaDot = 0;
            elseif ratesOptions(2)==true
                % if descent motor has fired and using TVC, calculate 
                % from TVC torques
                theta = state(5);       % (deg)
            else
                % if descent motor has fired but not using TVC
                theta = atan2d(ydot,xdot);   % (deg)
                thetaDot = 0;
            end
        elseif ratesOptions(2)==false % if not firing descent motor
            if t<burnTime
                % if motor not fired yet and still burning first motor
                theta = theta0;  % (deg)
                thetaDot = 0;    % (deg/s)
            else % else, calculate from velocity direction
                theta = atan2d(ydot,xdot);       % (deg)
                thetaDot = 0;    % (deg/s)
            end
        end
    
        
    % =========================================
    % Determine which engines are firing and evaluate thrust curve
        ascentThrust = interp1(thrustCurve(:,1),thrustCurve(:,2),t,'linear',0);      % (N)
        if ratesOptions(1) == true
            % descent thrust is pointing opposite direction of ascent thrust
            % (relative to rocket), so it is negative
            descentThrust = - interp1(thrustCurve(:,1),thrustCurve(:,2),t-t_fire,'linear',0);      % (N)
            Thrust = ascentThrust + descentThrust; % (N)
        else
            Thrust = ascentThrust;
        end

    % =========================================
    % Evaluate Drag
        Drag = 1/2 * rho * (xdot^2 + ydot^2) * S * C_D;        % (N)

    % =========================================
    % determine PID stuff
    if ratesOptions(2) == true
        % do PID thing
        u = 0;      % (degrees)
        delta = u;  % (degrees)
    else
        % don't do PID thing
        delta = 0;  % (degrees)
    end
            
    % =========================================
    % Sum forces to evaluate accelerations
        xddot = ( Thrust*cosd(theta+delta)    -   Drag*cosd(theta) ) / m;       % (m/s^2)
        yddot = ( Thrust*sind(theta+delta)    -   Drag*sind(theta) ) / m   -   g_accel;        % (m/s^2)
        thetaDDot = 0;   % (deg/s^2)
   
    % =========================================
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
    % Final state derivative vector
        stateDot = [xdot xddot ydot yddot thetaDot thetaDDot mdot]';
    
end