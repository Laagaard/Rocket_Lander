
%TODO:
    %UPDATE CGP(distance from CG to pivot point of tvc)             |x
    %IMPLEMENT KI                                                   |x
    %IMPLEMENT KD                                                   |x
    %IMPLEMENT BURNING DYNAMICS MASS FLOW                           |
    %UPDATE STARTING MASS                                           |x
    %IMPLEMENT RESTORATIVE MOMENT FROM AERO                         |x
    %IMPLEMENT PROPER INITIAL BODY ROTATIONS                        |~

g = -9.81;      %m/s^2
thrust = 24.13; %N, artifact won't ever get used, but initializes var
m = 1.217;        %kg after ascent motor fire
rho = 1.225; % (kg/m^3)
D_ref = 3.28*25.4/1000; % (m)
S_ref = pi*(D_ref^2)/4; % (m^2)

Ixx = 0.0484;
Iyy = 0.0484;
CGP = 0.165;        %m distance from cg to pivot (pivot is 242 mm from tip nose cone)

burn_time = 5.5;  %s
first_burn = true;
burn_start_x = 0;
burn_start_y = 0;
burn_start_z = 0;

resolution = 100; %hz
controllerFrequency = 75; %hz
dT = 1/resolution;
loopCtr = 1;
time = zeros(2,1);
previousTimePIDupdate = 0;

%GAINS
kp_body_x_rot = kpbxr; 
kp_body_y_rot = kpbxr;
ki_body_x_rot = kibxr; 
ki_body_y_rot = kibxr;
kd_body_x_rot = kdbxr; 
kd_body_y_rot = kdbxr;

integrated_body_x_error = 0;
integrated_body_y_error = 0;


%INITIAL CONDITIONS
target_body_x_rot = 0;  %deg
target_body_y_rot = 0;  %deg

tvc_x_rot = 0;  %relative to the body
tvc_y_rot = 0;  %relative to the body

tvc_x_rot_ctr = zeros(2,1);
tvc_y_rot_ctr = zeros(2,1);

body_x_rot = 30;    %deg;   zero when rocket is pointing straight down   %NEED TO UPDATE THESE WITH AN INITIAL ORIENTATION
body_y_rot = 10;     %deg;   zero when rocket is pointing straight down  %NEED TO UPDATE THESE WITH AN INITIAL ORIENTATION

body_x_rot_ctr = zeros(2,1); %deg
body_y_rot_ctr = zeros(2,1); %deg

last_body_x_error = 0;
last_body_y_error = 0;

body_x_rot_v = 0;   %deg/s
body_y_rot_v = 0;   %deg/s

body_x_rot_a = 0;   %deg/s^2
body_y_rot_a = 0;   %deg/s^2

% Values from Nate's Sim
body_x = 0;     %m
body_y = -2.59351400E1;  %m
body_z = 1.09805473E2;   %m

body_x_ctr = zeros(2,1); %m
body_y_ctr = zeros(2,1); %m
body_z_ctr = zeros(2,1); %m

body_x_v = 0;  %m/s
body_y_v = -6.52019524;  %m/s
body_z_v = 0;  %m/s

body_x_v_ctr = zeros(2,1); %m/s
body_y_v_ctr = zeros(2,1); %m/s
body_z_v_ctr = zeros(2,1); %m/s

test_data = readmatrix("STATIC FIRE TEST DATA.txt"); % import test data from static fire
first_thrust_curve = true;
first_gain_schedule = true;

% Arrays for gain scheduling
thrustCurveLength = 66;
timeArray = [0, 0.096, 0.193, 0.289, 0.386, 0.482, 0.579, 0.675, 0.772, 0.868,...
             0.965, 1.061, 1.158, 1.254, 1.351, 1.447, 1.544, 1.64, 1.736, 1.834,...
             1.929, 2.026, 2.122, 2.219, 2.315, 2.412, 2.508, 2.605, 2.701, 2.798,...
             2.894, 2.991, 3.087, 3.184, 3.28, 3.377, 3.473, 3.57, 3.666, 3.763,...
             3.859, 3.957, 4.052, 4.149, 4.245, 4.342, 4.438, 4.535, 4.631, 4.728,...
             4.824, 4.921, 5.017, 5.114, 5.21, 5.307, 5.403, 5.5, 5.596, 5.692,...
             5.789, 5.885, 5.982, 6.079, 6.175, 6.271];

thrustArray = [27, 27, 27, 27.147, 37.179, 37.993, 38.215, 38.715, 39.187, 39.183,...
               39.545, 40.162, 40.725, 40.657, 40.700, 40.264, 39.765, 39.181,...
               38.343, 37.504, 36.674, 35.507, 34.333, 32.917, 31.373, 29.696,...
               27.853, 25.905, 23.856, 21.759, 19.559, 17.803, 16.084, 14.518,...
               13.002, 11.646, 10.117, 8.786, 7.783, 6.943, 6.177, 5.576, 5.089,...
               4.621, 4.159, 3.855, 3.621, 3.326, 2.990, 2.646, 2.372, 2.014,...
               1.688, 1.288, 0.876, 0.582, 0.361, 0.181, 0.080, 0.041, 0.028,...
               0.024, 0.020, 0.015, 0.005, 0.001];

timeArrayCounter = 1;
maxThrust = 40.6;

while body_z > 0
    
    current_body_x_error = target_body_x_rot - body_x_rot; %update current error
    current_body_y_error = target_body_y_rot - body_y_rot; %update current error

    d_body_x_error = (current_body_x_error - last_body_x_error)/(1/controllerFrequency); %update current derivative of error
    d_body_y_error = (current_body_y_error - last_body_y_error)/(1/controllerFrequency); %update current derivative of error

    %HANDLES TURNING ON THRUSTER AND CHANGING ABSOLUTE THRUST WITH CURVE
    if time(loopCtr) >= burn_start && time(loopCtr) <= burn_start + burn_time

        if first_burn
            first_burn = false;
            burn_start_x = body_x;
            burn_start_y = body_y;
            burn_start_z = body_z;
        end
           
        %these are only updated while the motor is firing so that the
        %response doesn't blow up during normal flight
        integrated_body_x_error = integrated_body_x_error + current_body_x_error*(1/controllerFrequency); %update current integral of error
        integrated_body_y_error = integrated_body_y_error + current_body_y_error*(1/controllerFrequency); %update current integral of error

        targetValue = time(loopCtr) - burn_start; %time under thrust
        thrust_curve 
        absolute_thrust = load_processed_notable_thrust(thrust_index);

        thrust = absolute_thrust;
         
        % Gain Scheduling
        % Get current thrust from thrust curve
        currentTimeInBurn = time(loopCtr) - burn_start;
        [~, idx] = min(abs(timeArray - currentTimeInBurn));
        currentThrust = thrustArray(idx);
        
        % Gain scheduling based on current thrust
        if currentThrust > 0
            kp_body_x_rot = kpbxr * (maxThrust / currentThrust);
            kp_body_y_rot = kpbxr * (maxThrust / currentThrust);
            kd_body_x_rot = kdbxr * (maxThrust / currentThrust);
            kd_body_y_rot = kdbxr * (maxThrust / currentThrust);
            ki_body_x_rot = kibxr * (maxThrust / currentThrust);
            ki_body_y_rot = kibxr * (maxThrust / currentThrust);
        
        else
            % Reset gains to original values when not burning
            kp_body_x_rot = kpbxr;
            kp_body_y_rot = kpbxr;
            kd_body_x_rot = kdbxr;
            kd_body_y_rot = kdbxr;
            ki_body_x_rot = kibxr;
            ki_body_y_rot = kibxr;
        end


    else
        thrust = 0;
    end
    

    if time(loopCtr)>=previousTimePIDupdate + 1/controllerFrequency
        tvc_x_rot = kp_body_x_rot * current_body_x_error + kd_body_x_rot * d_body_x_error + ki_body_x_rot * integrated_body_x_error; %PID equation
        tvc_y_rot = kp_body_y_rot * current_body_y_error + kd_body_y_rot * d_body_y_error + ki_body_y_rot * integrated_body_y_error; %PID equation
    
        last_body_x_error = current_body_x_error; %track the current error for next time
        last_body_y_error = current_body_y_error;
    
        if tvc_x_rot > 10
            tvc_x_rot = 10;
        end
        if tvc_x_rot < -10
            tvc_x_rot = -10;
        end
        if tvc_y_rot > 10
            tvc_y_rot = 10;
        end
        if tvc_y_rot < -10
            tvc_y_rot = -10;
        end

        previousTimePIDupdate = time(loopCtr);
    end


    v_inf = sqrt(body_x_v^2 + body_y_v^2 + body_z_v^2); % m/s
    q_inf = 0; % 1/2*rho*v_inf^2; % Pa
    C_m_q = @(q) -0.0246*q;
    C_m_alpha = @(alpha) 17.701*alpha^3 - 2.495*alpha^2 - 20.942*alpha + 0.157;

    alpha_x = 0;% deg2rad(body_x_rot);
    alpha_y = 0;% deg2rad(body_y_rot);

    q_x = deg2rad(body_x_rot_v);
    q_y = deg2rad(body_y_rot_v);

    body_x_rot_a = ((thrust*sind(tvc_x_rot) * CGP) + q_inf*S_ref*D_ref*(C_m_alpha(alpha_x) + C_m_q(q_x)))/Ixx; % ADD AERODYNAMICS
    body_y_rot_a = ((thrust*sind(tvc_y_rot) * CGP) + q_inf*S_ref*D_ref*(C_m_alpha(alpha_y) + C_m_q(q_y)))/Iyy;

    % body_x_rot_a = ((thrust*sind(tvc_x_rot) * CGP) )/Ixx; % ADD AERODYNAMICS
    % body_y_rot_a = ((thrust*sind(tvc_y_rot) * CGP) )/Iyy;

    %split thrust into components based on body and tvc angle
    thrust_x = thrust * sind(tvc_x_rot + body_x_rot);
    thrust_y = thrust * sind(tvc_y_rot + body_y_rot);
    thrust_z = thrust * cosd(tvc_x_rot + body_x_rot) * cosd(tvc_y_rot + body_y_rot);

    % sqrt(thrust_x.^2 + thrust_y.^2 + thrust_z.^2) %checks that all thrust
    % components equal the total thrust from the motor

    body_x_v = body_x_v + (thrust_x/m)*dT; 
    body_y_v = body_y_v + (thrust_y/m)*dT;
    body_z_v = body_z_v + g*dT + (thrust_z/m)*dT;

    body_x_rot_v = body_x_rot_v + body_x_rot_a*dT;
    body_y_rot_v = body_y_rot_v + body_y_rot_a*dT;

    body_x_rot = body_x_rot + body_x_rot_v*dT;
    body_y_rot = body_y_rot + body_y_rot_v*dT;

    body_x = body_x + body_x_v*dT;
    body_y = body_y + body_y_v*dT;
    body_z = body_z + body_z_v*dT;

    body_x_ctr(loopCtr) = body_x;
    body_y_ctr(loopCtr) = body_y;
    body_z_ctr(loopCtr) = body_z;

    body_x_rot_ctr(loopCtr) = body_x_rot;
    body_y_rot_ctr(loopCtr) = body_y_rot;

    body_x_v_ctr(loopCtr) = body_x_v;
    body_y_v_ctr(loopCtr) = body_y_v;
    body_z_v_ctr(loopCtr) = body_z_v;

    tvc_x_rot_ctr(loopCtr) = tvc_x_rot;
    tvc_y_rot_ctr(loopCtr) = tvc_y_rot;

    loopCtr = loopCtr +1;
    time(loopCtr) = time(loopCtr-1) + dT;
end

time(end) = [];
% figure
% % axis equal
% hold on
% plot3(body_x_ctr,body_y_ctr,body_z_ctr)
% plot3(burn_start_x,burn_start_y,burn_start_z,'x')
% hold off
% 
% figure
% plot(time,body_z_v_ctr)
% 
% figure
% hold on
% xlabel('Time (s)')
% ylabel('Angle (Degrees)')
% plot(time,body_x_rot_ctr)
% plot(time,body_y_rot_ctr)
% plot(time,tvc_x_rot_ctr)
% plot(time,tvc_y_rot_ctr)
% legend('Body Rotation X','Body Rotation Y','TVC Rotation X','TVC Rotation Y')
% hold off
% 
% body_x
% body_y
% 
% body_x_v
% body_y_v
% body_z_v































