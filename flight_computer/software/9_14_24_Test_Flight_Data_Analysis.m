stratologgerData = readmatrix("COTS_Test_Flight_Data_9_14_24.csv"); % data from the onboard COTS altimeter
stratologgerData = stratologgerData(1:end-2,:); % trim unrealistic end points

stratoLoggerTime = stratologgerData(:,1); % [s]
stratoLoggerAltitude = stratologgerData(:,2); % [ft]
stratoLoggerVoltage = stratologgerData(:,4); % [V]

DART = readmatrix("only salvagable 'flight' data.txt"); % data from DART FC (MK I)
DART_altitude = atmospalt(DART(:,8)) / 0.3048; % [ft]
DART_time = mod(DART(:,1),DART(1,1)) / 1E3; % [s]

figure
hold on
plot(stratoLoggerTime, stratoLoggerAltitude)
grid minor
xlabel("Time [s]")
ylabel("Altitude [ft]")
plot(DART_time, DART_altitude, 'g')

yyaxis right
plot(stratoLoggerTime, stratoLoggerVoltage)
ylabel("Voltage [V]")
ylim([-1 9])

title("Altitude [ft]. Voltage [V] vs. Time [s]")