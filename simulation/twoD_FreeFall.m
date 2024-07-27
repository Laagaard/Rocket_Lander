%%
%{
NOTES and TO DO:

How to incorporate PID controller into ODE45

%}
%%

clc; clear; close all; format compact
%
tic
% GLOBAL VARIABLES  
global g_accel burnTimeDescent thrustForceDescent C_D rho S 
global descentBurn motorImpulse tdescent_burn_start timeToDescend 
global MDOT_Ascent MDOT_Descent l_t I_zz

% CONSTANTS
thrustCurveDescent = struct2array(load("Aerotech_G25W.mat")); % time (s), force (N)
burnTimeDescent = thrustCurveDescent(:,1); % s
thrustForceDescent = thrustCurveDescent(:,2); % N
descentBurn = false; 
motorImpulse = 120; % N-s
tdescent_burn_start = 100; % sD
PropMass_descent = 0.0625; % kg
PropMass_ascent = PropMass_descent; % kg
MDOT_Descent = (thrustForceDescent/norm(thrustForceDescent)) * PropMass_descent; % kg/s

g_accel = 9.81; % m/s^2
C_D = 0.75;
rho = 1.225; % kg/m^3
diam = 3.1 * (2.54/100) ; % m
S = pi*(diam/2)^2; % m^2
l_t = 0.5; % m
I_zz = .05; % kg-m^2



%%
%load in initial solution
twoD_initial = struct2array(load("twoD_initial.mat"));
h = 0.055/4  ; % s
tevaluate = 11:h:12; % s
horizontalLandingSpeed = zeros(size(tevaluate)); % m/s
verticalLandingSpeed = zeros(size(tevaluate)); % m/s
landingAngle = zeros(size(tevaluate)); % deg
figure(1); tiledlayout("flow")
figure(2); tiledlayout("flow")
for ctr=1:length(tevaluate)
    tol = h*.9;
    index = find(abs(tevaluate(ctr)-twoD_initial(:,1))<=tol);
    index = round(mean(index))

    % IC's
    y0 = twoD_initial(index,4); % m
    ydot0 = twoD_initial(index,5); % m/s
    m0 = 1.50 - PropMass_ascent; % kg
    x0 = twoD_initial(index,2); % m
    xdot0 = twoD_initial(index,3); % m/s
    theta0 = pi/2 - atan2(ydot0,xdot0); % rad, angle between vertical and velocity direction
    thetaDot0 = 0; % rad/s
    errorIntegr0 = 0; % radian-s
    errorIntegr02 = 0; % m-s
    tspan = [0 20]; % s
    options = odeset(MaxStep=0.01);
    state0 = [x0 xdot0 y0 ydot0 theta0 thetaDot0 m0 errorIntegr0 errorIntegr02];
    
    [t,states] = ode45(@rates_descent,tspan,state0,options);
    range = states(:,1); % m
    rangeDot = states(:,2); % m/s
    altitude = states(:,3); % m
    altitudeDot = states(:,4); % m/s
    theta = states(:,5); % rad
    thetaDot = states(:,6); % rad/s
    indices = find(altitude>=0);
    rangeDDot = zeros(size(range)); % m/s^2
    altitudeDDot = zeros(size(altitude)); % m/s^2
    thetaDDot = zeros(size(altitude)); % rad/s^2
    error = zeros(size(altitude)); % rad
    error2 = zeros(size(altitude)); % m
    % for ctr = 1:length(t)
    %     state_dot = rates_descent(t(ctr),states(ctr,:));
    %     rangeDDot(ctr) = state_dot(2); % m/s^2
    %     altitudeDDot(ctr) = state_dot(4); % m/s^2
    %     thetaDDot(ctr) = state_dot(6); % rad/s^2
    %     error(ctr) = state_dot(8); % rad
    %     error2(ctr) = state_dot(9); % m
    % end

    horizontalLandingSpeed(ctr) = rangeDot(indices(end)); % m/s
    verticalLandingSpeed(ctr) = altitudeDot(indices(end)); % m/s
    landingAngle(ctr) = rad2deg(theta(indices(end))); % deg


    if mod(tevaluate(ctr),0.01)==0
        figure(1)
        nexttile;
        plot(range(indices),altitude(indices))
        axis equal; %xlim([40 60])
        title(sprintf('t = %.2f',tevaluate(ctr)))


        figure(2);
        nexttile;
        plot(t(indices),theta(indices))
        title(sprintf('t = %.2f',tevaluate(ctr)))

    end
end
%% Graph Stuff
%
figure(1); print('range altitude plots.png','-dpng','-r300');
figure(2); print('angular position plots.png','-dpng','-r300');

figure(3); hold on
plot(tevaluate,horizontalLandingSpeed,'o')
plot(tevaluate,verticalLandingSpeed,'d')
plot(tevaluate,rad2deg(landingAngle),'--')
xlabel('time of ignition (s)')
ylabel('landing state values')
legend('horizontal landing speed (m/s)','veritcal landing speed (m/s)','landing angle from vetical (deg)','location','best')
print('Landing states plot.png','-dpng','-r300')
ylim([-50 10])
%}

minVertSpeed = max(verticalLandingSpeed); % m/s
index = find(verticalLandingSpeed==minVertSpeed);
pseudoMinHorizSpeed = horizontalLandingSpeed(index); % m/s
pseudoMinLandAngle = landingAngle(index); % rad
disp(sprintf('For "optimal" solution:\n\tvertical landing speed = %.2f m/s.',minVertSpeed))
disp(sprintf('\thorizontal landing speed = %.2f m/s.\n\tlanding angle from vertical = %.2f degrees.',[pseudoMinHorizSpeed rad2deg(pseudoMinLandAngle)]))
save('optimal solution states.mat',"minVertSpeed","pseudoMinHorizSpeed","pseudoMinLandAngle",'-mat')
toc

%% Dynamic display of solution trajectory
%{
T_begin_burn = tevaluate(index);

% calculate trajectory of using optimal timing and use it to plot rocket
% graphic
initialIndex = find(abs(tevaluate(index)-twoD_initial(:,1))<=1.25E-4);
firstHalf_states = twoD_initial(1:initialIndex,:);
    % IC's
    y0 = twoD_initial(initialIndex,4); % m
    ydot0 = twoD_initial(initialIndex,5); % m/s
    m0 = 1.50 - PropMass_ascent; % kg
    x0 = twoD_initial(initialIndex,2); % m
    xdot0 = twoD_initial(initialIndex,3); % m/s
    theta0 = pi/2 - atan2(ydot0,xdot0); % rad, angle between vertical and velocity direction
    thetaDot0 = 0; % rad/s
    errorIntegr0 = 0; % radian-s
    errorIntegr02 = 0; % m-s
    tspan = [0 10]; % s
    options = odeset(MaxStep=0.01);
    state0 = [x0 xdot0 y0 ydot0 theta0 thetaDot0 m0 errorIntegr0 errorIntegr02];
[t_secondHalf,secondHalfStates] = ode45(@rates_descent,tspan,state0,options);
indices_aboveGround = find(secondHalfStates(:,4)>-0.2);

t_graphics = [firstHalf_states(:,1); t_secondHalf(indices_aboveGround)];
x_graphics = [firstHalf_states(:,2); secondHalfStates(indices_aboveGround,1)];
xDot_graphics = [firstHalf_states(:,3); secondHalfStates(indices_aboveGround,2)];
y_graphics = [firstHalf_states(:,4); secondHalfStates(indices_aboveGround,3)];
yDot_graphics = [firstHalf_states(:,5); secondHalfStates(indices_aboveGround,4)];
theta_graphics = [firstHalf_states(:,6) secondHalfStates(indices_aboveGround,5)];

figure(4);
hold on;
scale = 1;
width = diam*scale;
Rocket_length = l_t*scale;
coneLength = 0.15 * width; % m

x_vertices0 = [-width/2         -width/2        0                            width/2         width/2          -width/2        ];
y_vertices0 = [-Rocket_length/2 Rocket_length/2 (Rocket_length/2+coneLength) Rocket_length/2 -Rocket_length/2 -Rocket_length/2];
Rocket = area(x_vertices0,y_vertices0); % initial rocket shape
XY0 = [x_vertices0; y_vertices0];

for ctr=2:length(t_graphics)
    R = [cos(theta_graphics(ctr))   sin(theta_graphics(ctr));
         -sin(theta_graphics(ctr))  cos(theta_graphics(ctr))];
    XY_graphic = R*XY0;
    
    Rocket.XData = XY_graphic(1,:); % update data properties
    Rocket.YData = XY_graphic(2,:);
    drawnow  % display updates
    %pause(1/10)
end
%}

%% FUNCTIONS    

function state_dot = rates_descent(t,states)
    global g_accel burnTimeDescent thrustForceDescent rho C_D S MDOT_Descent
    global l_t I_zz
    x = states(1);    % m
    y = states(3);     % m
    xdot = states(2); % m/s`
    ydot = states(4);  % m/s     
    theta = states(5); % rad
    thetaDot = states(6); % rad/s
    mass = states(7); % kg
    errorIntegr = states(8); % radian-s
    errorIntegr2 = states(9); % m-s

    Tdescent = -interp1(burnTimeDescent,thrustForceDescent,t,'linear',0); % N
   
    
        % % construction of PID 
    % referenceTheta       = 0;     % reference theta desired (straight down)
    % e       = theta - referenceTheta;   % error signal
    % Kp      = 100;                  
    % Ki      = 1600;                            
    % Kd      = 2.25;          

        % construction of PID 1 (for angle)
    referenceTheta       = 0;     % reference theta desired (straight down)
    e       = theta - referenceTheta;   % error signal
    Kp      = 100;                  % proportional gain
    Ki      = 1600;                            % integral gain
    Kd      = 2.25;                  % derivative gain
    u1       = - Kp*e - Ki*errorIntegr - Kd*thetaDot;   % the PID thing
    
            % construction of PID 2 (for range position)    
    referenceX       = 50;     % reference range desired (50 m)
    e2       = x - referenceX;   % error signal
    Kp      = 75;                  % proportional gain
    Ki      = 15;                            % integral gain
    Kd      = .85;                  % derivative gain
    u2       = (- Kp*e2 - Ki*errorIntegr2 - Kd*xdot);   % the PID thing
    
    errorIntegrDot = e; % rad
    errorIntegrDot2 = e2; % m
    waitTime = 0;%.125; % s
    changeAltitude = 1; % m
    if t>=waitTime && y>=changeAltitude
        phi = u1+u2; % range and pitch
    elseif t>=waitTime
        phi = u1; % just pitch
    else
        phi = 0;
    end
        
    thetaDDot = Tdescent*l_t*sin(phi)/I_zz; % rad/s^2
    Drag = (1/2*rho*(xdot^2 + ydot^2))*S*C_D; % N
    if xdot>=0
        xdotSign = 1;
    else
        xdotSign = -1;
    end
    if ydot>=0
        ydotSign = 1;
    else
        ydotSign = -1;
    end

    xddot = Tdescent/mass*sin(theta+phi) - Drag*xdotSign/mass*sin(theta); % m/s^2
    yddot = Tdescent/mass*cos(theta+phi) - Drag*ydotSign/mass*cos(theta) - g_accel; % m/s^2
    mdot = -interp1(burnTimeDescent,MDOT_Descent,t,'linear',0); % kg/s
    state_dot = [xdot xddot ydot yddot thetaDot thetaDDot mdot errorIntegrDot errorIntegrDot2]';
end
