#include <Servo.h>
#include <math.h>
//#include <Vector.h>
#include <Wire.h>  //TBR: may need for i2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
//#include <Adafruit_GPS.h>
#include <SD.h>
// #include <EEPROM.h>
#include <LittleFS.h>
//#include <SPI.h>  //TBR: may need for eeprom management

#define tvcServoXPin 36
#define tvcServoYPin 37
#define pyro1GatePin 8
#define pyro2GatePin 38
#define pyro1ContinuityPin 9
#define pyro2ContinuityPin 39
#define landingLegServoPin 12
#define LEDIndicatorPin 41
#define LEDMainPin 40
#define buzzerPin 31
#define BMP280_ADDRESS 0x76
//#define GPSSerial Serial7
const int chipSelect = BUILTIN_SDCARD;

enum state {
  ASSEMBLY,
  INITIALIZATION,
  AWAITGOFORLAUNCH,
  AWAITLIFTOFF,
  POWEREDASCENT,
  UNPOWEREDASCENT,
  UNPOWEREDDESCENT,
  TERMINALGUIDANCE,
  LANDED,
  INFLIGHTABORT
};

enum state machineState = INITIALIZATION;  //INITIALIZATION

uint32_t globalTimer = millis();
uint32_t XBeeTimer = millis();  //TBR: artifact of string assembly method buffer clearing
uint32_t telemetryTimer = millis();
uint32_t timeSinceLastBuzz = millis();
uint32_t lightTimer = millis();
uint32_t flightTimer = millis();
uint32_t memTimer = millis();
uint32_t abortPyroTimer = millis();
uint32_t postAbortTimer = millis();
uint32_t pressureCheckTimer = millis();
uint32_t continuityTimer = millis();
uint32_t current_DNT_time = millis();
uint32_t lastPIDTime = micros();
uint32_t controlTimer = millis();
uint32_t ignitionDelayTimer = millis();
uint32_t loopTime = millis();

float linAccelX = 0;  //measured
float linAccelY = 0;  //measured
float linAccelZ = 0;  //measured

double quatW = 0;  //measured
double quatX = 0;  //measured
double quatY = 0;  //measured
double quatZ = 0;  //measured

float pitch = 0;  //derived
float roll = 0;   //derived
float yaw = 0;    //derived

//TBR: set lat long to 1.0
float decimalLatitude = 1.000000;   //28.562916;  //measured //TBR: THIS WAS CHANGED FROM DOUBLE TO FLOAT MAKE SURE THATS OKAY
float decimalLongitude = 1.000000;  //-81.017365; //measured
float rawSpeed = 0;                 //derived

float bmpPressure = 0;     //measured
float bmpTemperature = 0;  //measured
float bmpAltitude = 0;     //"measured" (derived within Adafruit_BMP280.h)
float altitude = 0;
const int pressureCheckLength = 5;
int pressureCheckCounter = 0;
float pressureCheck[pressureCheckLength] = { 101325, 101325, 101325, 101325, 101325 };
float pressureVariance = 0;

int altitudeCheckCounter = 0;
const int altitudeCheckLength = 5;
float altitudeCheck[altitudeCheckLength] = { 1000, 500, 250, 125, 50 };  //want initial variance to stay high so as to not trigger landing
float altitudeVariance = 100;

int fiftyHzTimer = 0;  //various counters for time based operations
int onehundredHzTimer = 0;
int tenHzTimer = 0;
int oneHzTimer = 0;
int onethirdHzTimer = 0;
unsigned int telemetryRate = 20;  //hz //TBR: may be a ground station problem, data wont report over 10 hz

//TBR: should all be reset on vehicle recycle
float pressurePropogator[10] = {};
int pressurePropogatorCounter = 0;
float padPressure = 0;
double maxbmpAltitude = 0;  //TBR: must be declared globally to eliminate first cycle apogeeDetected error
float maxAltitude = 0;
float altitudePropogator = 0;  //TBR: if there is something that doesn't need to be global it is the propogators, fix method
int altitudePropogatorCounter = 0;
float altitudeOffset = 0;
float adjustedAltitude = 0;
float gravity = 9.79;
float engine_cutoff_tolerance = 0;
float launch_accel_tolerance = 4;
int expected_motor_burn_time = 6272;  //ms
String dataString = "";
String altDataString = "";
String assembledString = "";  //TBR: remove if string assembly remains unused
uint32_t pyroStartTimer = millis();
const int maxSoundBufferLength = 100;
int sounds[maxSoundBufferLength][2] = {};  //TBR:must flush at end and reset counter. this sucks, arduino vectors are terrible, classic
int soundIndex = 1;
int soundHandlerIndex = 1;
uint32_t soundTimer = millis();
unsigned int eepromAddress = 0;  //current EEPROM address, TBR: should be checked within code so as to not overwrite memory
int continuityState1 = 0;
int continuityState2 = 0;
int pyro1Gate = LOW;
int pyro2Gate = LOW;
int comCode = 0;    //allows for proper 2 way communication between rocket and ground
int errorCode = 0;  //allows for the onboard error handling to be used on ground, instead of a separate ground based error detection
bool vehicleHealthy = false;


String numBuffer = "";
int inputNumber = 0;

//TBR: should all be reset on vehicle recycle
bool abortFlight = false;
bool abortTriggered = false;
bool abortFirstTrigger = true;
bool potentialTouchdown = false;
bool SOUND_ON = true;  //enables/disables buzzer
bool codeLightEnabled = false;
bool bmpInitialized = false;
bool bnoInitialized = false;
bool bmppropogatorCompletion = false;
bool bnoCalibrationCompletion = false;
bool validGPSData = false;
bool apogeeFirstTime = true;           //TBR: must be declared globally to eliminate first cycle apogeeDetected error
bool firstLoop = true;                 //exists only to accomodate SD card header printing
bool SDCardInserted = true;            //start as true and set to false upon failure
bool printDataStringToSerial = false;  //should always stay false unless commanded otherwise for testing purposes
bool printToXBee = false;
bool printToSD = true;
//bool totalString = false;           //TBR: artifact of string assembly, may still need
bool clearedForLaunch = false;  //the almighty all powerful final launch flag

int tvcServoXAngle = 90;  //TBR: assumes 90 degrees as neutral position
int tvcServoYAngle = 90;  //TBR: assumes 90 degrees as neutral position
int lastTvcServoXAngle = tvcServoXAngle;
int lastTvcServoYAngle = tvcServoYAngle;
int landingLegServoAngle = 90;
float tvc_x_rot = 0;
float tvc_y_rot = 0;
float last_body_x_error = 0;
float last_body_y_error = 0;
float integrated_body_x_error = 0;
float integrated_body_y_error = 0;
float body_x_rot = 0;
float body_y_rot = 0;

float target_body_x_rot = 0.0;
float target_body_y_rot = 0.0;

//\/\/\/\/\/\/\/\/\/\/\/\/\/CONTROLLER VALUES  TO BE MODULATED\/\/\/\/\/\/\/\/\/\/\/\/\/
//\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

const int thrustCurveLength = 66;

float timeArray[thrustCurveLength] = {
  0, 0.096, 0.193, 0.289, 0.386, 0.482, 0.579, 0.675, 0.772, 0.868,
  0.965, 1.061, 1.158, 1.254, 1.351, 1.447, 1.544, 1.64, 1.736, 1.834,
  1.929, 2.026, 2.122, 2.219, 2.315, 2.412, 2.508, 2.605, 2.701, 2.798,
  2.894, 2.991, 3.087, 3.184, 3.28, 3.377, 3.473, 3.57, 3.666, 3.763,
  3.859, 3.957, 4.052, 4.149, 4.245, 4.342, 4.438, 4.535, 4.631, 4.728,
  4.824, 4.921, 5.017, 5.114, 5.21, 5.307, 5.403, 5.5, 5.596, 5.692,
  5.789, 5.885, 5.982, 6.079, 6.175, 6.271
};

unsigned int timeArrayCounter = 0;

float thrustArray[thrustCurveLength] = {
  27, 27, 27, 27.1470168, 37.1793114, 37.9932471, 38.2150512, //0.6946461, 0.8619066, 6.6888504, first three real values but disregard
  38.7153612, 39.1872222, 39.1827096, 39.5452872, 40.1616495, 40.7250378, 40.6567602,
  40.6998261, 40.2635754, 39.7652274, 39.1810419, 38.3429736, 37.5035319, 36.6740964,
  35.5067064, 34.333038, 32.9173569, 31.3727724, 29.6963415, 27.8530425, 25.9045803,
  23.855958, 21.7589724, 19.5588837, 17.8027956, 16.0843779, 14.5184076, 13.0017816,
  11.6458434, 10.1174454, 8.7864246, 7.7827635, 6.9431256, 6.1772589, 5.5764945,
  5.0885451, 4.6209024, 4.1585571, 3.8547414, 3.6211653, 3.3260805, 2.9896956,
  2.6459532, 2.3715675, 2.0136987, 1.6876143, 1.2881511, 0.8757387, 0.5820273,
  0.3605175, 0.1806021, 0.0798534, 0.041202, 0.0282528, 0.0242307, 0.0204048,
  0.0152055, 0.0050031, 0.001
};

float lastThrust = 0;

float maxThrust = 40.6;

const float kp_body_x_rot_original = .25;  //0.25;//.5;
const float kp_body_y_rot_original = .25;  //0.25;//.5;
const float kd_body_x_rot_original = .1;   //0;
const float kd_body_y_rot_original = .1;   //0;
const float ki_body_x_rot_original = 0;    //0.009;   //.00002;
const float ki_body_y_rot_original = 0;    //0.009;   //.00002;

float kp_body_x_rot = kp_body_x_rot_original;  //0.25;//.5;
float kp_body_y_rot = kp_body_y_rot_original;  //0.25;//.5;
float kd_body_x_rot = kd_body_x_rot_original;  //0;
float kd_body_y_rot = kd_body_y_rot_original;  //0;
float ki_body_x_rot = ki_body_x_rot_original;  //0.009;   //.00002;
float ki_body_y_rot = ki_body_y_rot_original;  //0.009;   //.00002;

float controller_frequency = 75;
int ignitionDelay = 700;  //ms
int tvc_x_center = 82;
int tvc_x_upper_limit = 40;
int tvc_x_lower_limit = 40;
int tvc_y_center = 78;
int tvc_y_upper_limit = 40;
int tvc_y_lower_limit = 40;
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

LittleFS_QPINAND extChip;  //29 hours of storage at 10hz
File extFile;              //TBR: didn't want to call dataFile as this is already used by the SD Card, could be a problem
int extChipPrintCount = 0;
bool storageAvailable = true;  //TBR: maybe make dataString print conditional so we can send these flags easier
bool chipFailure = false;
bool dataSaveAnomaly = false;
bool memClearAttempt = false;
bool memClearConfirm = false;
bool memClearDenied = false;

//DNT:
File dntFile;
bool DNTLoaded = false;
float t[400] = {};
float x_1[400] = {};
float y_1[400] = {};
float x_2[400] = {};
float y_2[400] = {};
int DNTLength = 0;
int abort_ctr = 0;

// bool clearGPSError = false;

uint32_t flightStartTimer = millis();
uint32_t estimatedTimeToApogee = 6000;  //ms //TBR: dial in
bool justLanded = true;


int loopCtr = 0;
bool firstLoopCtr = true;



Servo TVCX;
Servo TVCY;
Servo LANDINGLEGSERVO;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bmp;
SoftwareSerial XBee(21, 20);
//Adafruit_GPS GPS(&GPSSerial);

void setup() {
  Serial.begin(115200);

  TVCX.attach(tvcServoXPin);
  TVCY.attach(tvcServoYPin);
  LANDINGLEGSERVO.attach(landingLegServoPin);

  LANDINGLEGSERVO.write(120);

  pinMode(pyro1ContinuityPin, INPUT);
  pinMode(pyro1GatePin, OUTPUT);
  pinMode(pyro2ContinuityPin, INPUT);
  pinMode(pyro2GatePin, OUTPUT);
  pinMode(LEDIndicatorPin, OUTPUT);
  pinMode(LEDMainPin, OUTPUT);

  digitalWrite(LEDMainPin, HIGH);

  //GPS.begin(9600);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  //TBR: test 5HZ
  //GPSSerial.println(PMTK_Q_RELEASE);  //returns PMTK_DT_RELEASE, probably unnecessary but conventional and works

  if (!SD.begin(chipSelect)) {
    //TBR: add buzzer handler here for SD card failure, as this will require near total deintegration
    SDCardInserted = false;
  }

  XBee.begin(9600);  //TBR: this baud rate may need to be higher, will have to be configured on the xbee itself
                     //https://os.mbed.com/questions/79494/Sending-Large-Data-Packages-255-Bytes-vi/
                     //definitely exceeding the tx and rx byte buffer sizes which is causing BIG slowdowns
  if (!extChip.begin()) {
    Serial.println("ovah");
    chipFailure = true;  //TBR: want to check for chip failure before
    playSound(2000, 500);
    while (true) {
      if (millis() - globalTimer > 500) {
        XBee.println("chip failure");
        globalTimer = millis();
      }
    }
  } else {
    extFile = extChip.open("datalog.txt", FILE_WRITE);
    if (extFile) extFile.println("PROGRAM START");
    extFile.close();
    extFile = extChip.open("datalog.txt", FILE_READ);  //TBR: really hope LittleFS doesn't throw a fit about datalog being used on the SD card
    if (extFile) {
      while (extFile.size() > 100000000) {
        while (XBee.available()) {
          //if((char)XBee.read() == 'm') extChipFormat(); //TBR: taken out for safety for now, we wont fill this in like the lifespan of the project i think
        }
        if (millis() - globalTimer > 500) {
          XBee.println("MEMORY FULL");
        }
      }
    }
    extFile.close();
  }
}

void loop() {

  // loopCtr++;   //test the hz of the main loop

  // if(machineState == TERMINALGUIDANCE && firstLoopCtr){
  //   if(millis() - loopTime > 1000){
  //     firstLoopCtr = false;
  //     comCode = loopCtr;
  //   }
  // }


  globalTimer = millis();  //set to the system time each cycle

  // updateGPSBuffer();

  updateXBee();

  // decimalLatitude = 28.562916;    //REMOVE REMOVE REMOVE REMOVE REMOVE REMOVE REMOVE
  // decimalLongitude = -81.017365;  //REMOVE REMOVE REMOVE REMOVE REMOVE REMOVE REMOVE
  // checkTrajectory();

  pyroContinuityCheck();

  //vehicleHealthCheck();

  //codeLight();

  if (machineState != INFLIGHTABORT) {
    if (abortFlight) {
      abortTriggered = true;
      machineState = INFLIGHTABORT;
    }
  }

  switch (machineState) {
    case ASSEMBLY:
      assembly();
      break;
    case INITIALIZATION:
      initialization();
      break;
    case AWAITGOFORLAUNCH:
      awaitGoForLaunch();
      break;
    case AWAITLIFTOFF:
      awaitLiftoff();
      break;
    case POWEREDASCENT:
      poweredAscent();
      break;
    case UNPOWEREDASCENT:
      unpoweredAscent();
      break;
    case UNPOWEREDDESCENT:
      unpoweredDescent();
      break;
    case TERMINALGUIDANCE:
      terminalGuidance();
      break;
    case LANDED:
      landed();
      break;
    case INFLIGHTABORT:
      inFlightAbort();
      break;
  }

  descentMotorCeaseFireHandler();

  buzzerHandler();

  if (machineState != TERMINALGUIDANCE){updateData(globalTimer, machineState);}
  else{attitudeControlTestUpdateData(globalTimer, machineState);}

  if (millis() - memTimer > 80 && machineState == TERMINALGUIDANCE) {
    extChipLog();
    memTimer = millis();
  }

  memClearHandler();

  if (machineState != TERMINALGUIDANCE) telemetry();
}

void assembly() {
}

void initialization() {
  // TVCX.write(90); //TBR: remove
  // TVCY.write(90); //TBR: remove
  if (!DNTLoaded) {
    loadDNT();
  }

  if (!bmpInitialized || !bnoInitialized) {
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS);
    if (!status) {
      bmpInitialized = false;
      playSound(800, 800);
    } else {
      bmpInitialized = true;
      pressureCheckTimer = millis();
      while (pressureCheckCounter < 5) {
        if (millis() - pressureCheckTimer > 200) {
          pressureCheck[pressureCheckCounter] = bmp.readPressure();  //TBR: check this implementation on hardware
          pressureCheckTimer = millis();
          pressureCheckCounter++;
        }
      }
      pressureCheckCounter = 0;
      playSound(2800, 300);
    }
    if (!bno.begin()) {
      bnoInitialized = false;
      playSound(800, 800);
    } else {
      // uint8_t accConfig = 0;
      // bno.getSensorOffsets(&accConfig);
      // bno.setMode(OPERATION_MODE_CONFIG);
      // Wire.beginTransmission(BNO055_ADDRESS_A);
      // Wire.write(0x08);  // Accelerometer config register
      // Wire.write(accConfig);
      // Wire.endTransmission();
      // bno.setMode(OPERATION_MODE_NDOF);
      bnoInitialized = true;
      playSound(3400, 300);
    }
  } else if (!bnoCalibrationCompletion) {
    uint8_t system, gyro, accel, mag;  //taking the calibration state of the bno055
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (system == 3) {  //full system calibration happens at 3
      bnoCalibrationCompletion = true;
      playSound(3800, 200);
    }
  } else {
    machineState = AWAITGOFORLAUNCH;
    //machineState = TERMINALGUIDANCE; //TBR: SUPER IMPORTANT THAT THIS GETS REMOVED SUPER IMPORTANT THAT THIS GETS REMOVED SUPER IMPORTANT THAT THIS GETS REMOVED SUPER IMPORTANT THAT THIS GETS REMOVED
  }
}

void awaitGoForLaunch() {

  guidance();

  pressurePropogation();

  // tvcServoXAngle = (0 * (4 / 1.5)) + tvc_x_center; //TBR: remove

  // if (tvcServoXAngle != lastTvcServoXAngle) {
  //     TVCX.write(tvcServoXAngle);
  //     lastTvcServoXAngle = tvcServoXAngle;
  // }

  // tvcServoYAngle = (0 * (4 / 1.5)) + tvc_y_center;

  // if (tvcServoYAngle != lastTvcServoYAngle) {
  //     TVCY.write(tvcServoYAngle);
  //     lastTvcServoYAngle = tvcServoYAngle;
  // }
  // comCode = tvc_x_center;

  if (clearedForLaunch) machineState = AWAITLIFTOFF;
}

void awaitLiftoff() {
  guidance();

  pressurePropogation();

  if (abs(linAccelZ) > launch_accel_tolerance) {  //TBR: dial in threshold value, maybe only use one measurement
    flightStartTimer = millis();
    machineState = POWEREDASCENT;
    playSound(1000, 400);
  }
}

void poweredAscent() {
  guidance();

  if (linAccelZ > engine_cutoff_tolerance && millis() - flightStartTimer > expected_motor_burn_time / 2) {  //basically checking that our body z acceleration has gone from negative to positive indicating that aero forces have overpowered the tail end of the thrust curve
    machineState = UNPOWEREDASCENT;
    //playSound(1200,400);
  }
  if (millis() - flightStartTimer > estimatedTimeToApogee + 1500) {  //this is called here just in case the state does not change
    machineState = UNPOWEREDASCENT;
  }
}

void unpoweredAscent() {
  guidance();

  //calcAltitudeVariance();
  //detect apogee
  if (altitude < maxAltitude - 2.5) {  //if we are less than 2.5 meters below our max altitude //WORKING
    machineState = UNPOWEREDDESCENT;
  }

  if (millis() - flightStartTimer > estimatedTimeToApogee + 1500) {  //JUST IN CASE ABOVE DOESNT TRIGGER //TBR: can this be removed?
    machineState = UNPOWEREDDESCENT;
  }
}

void unpoweredDescent() {
  guidance();

  //forward progpogation

  //start to ignite the motor when our condition is met in ___ ms

  if (false) {  //TBR: add detection of motor fire start here
    releaseLegs();
    lastPIDTime = micros();  //must be updated before updatePID is called for the first time,
    machineState = TERMINALGUIDANCE;
  }
}

void terminalGuidance() {
  //terminal(); //lol
  guidance();
  // if (millis() - ignitionDelayTimer > 700+3000) {
  //   kp_body_x_rot = 3;
  //   kp_body_y_rot = 3;
  // }
  if (micros() - lastPIDTime > 1000000.00 / controller_frequency) {
    updatePID();
    //SET Ki GAINS TO ZERO TO TEST IMPLEMENTATIONS
    //SET Kd GAINS TO ZERO TO TEST IMPLEMENTATIONS

    tvcServoXAngle = (tvc_x_rot * (4 / 1.5)) + tvc_x_center;

    if (tvcServoXAngle > tvc_x_center + tvc_x_upper_limit) tvcServoXAngle = tvc_x_center + tvc_x_upper_limit;
    if (tvcServoXAngle < tvc_x_center - tvc_x_lower_limit) tvcServoXAngle = tvc_x_center - tvc_x_lower_limit;

    if (tvcServoXAngle != lastTvcServoXAngle) {
      TVCX.write(tvcServoXAngle);
      lastTvcServoXAngle = tvcServoXAngle;
    }

    tvcServoYAngle = (tvc_y_rot * (4 / 1.5)) + tvc_y_center;

    if (tvcServoYAngle > tvc_y_center + tvc_y_upper_limit) tvcServoYAngle = tvc_y_center + tvc_y_upper_limit;
    if (tvcServoYAngle < tvc_y_center - tvc_y_lower_limit) tvcServoYAngle = tvc_y_center - tvc_y_lower_limit;

    if (tvcServoYAngle != lastTvcServoYAngle) {
      TVCY.write(tvcServoYAngle);
      lastTvcServoYAngle = tvcServoYAngle;
    }
  }

  //  comCode = tvcServoYAngle;
}

void landed() {
  guidance();
  if (justLanded) {
    extChipSdDump();
    justLanded = false;
    playSound(2000, 100);
    playSound(2000, 100);
    playSound(2000, 100);
  }
  //playSound(2000, 60);
}

void inFlightAbort() {
  guidance();
  comCode = 6;  //redundant data flag for parachute deploy
  pyro1Gate = HIGH;
  digitalWrite(pyro1GatePin, HIGH);

  //calcAltitudeVariance();

  bool landed = false;

  if (millis() - postAbortTimer > 5000) {  //if it has been 5 seconds since abort

    if (sqrt(pow(linAccelX, 2) + pow(linAccelY, 2) + pow(linAccelZ, 2)) < 0.5) {
      landed = true;
      pyro1Gate = LOW;
      digitalWrite(pyro1GatePin, LOW);
    }
  }
  if (landed) {  //&& pressureVariance < 1.0){ //TBR: replace with actual value //TBR: check pressure variance values at speed on hardware
    //postAbortTimer = millis();
    machineState = LANDED;
    abortFlight = false;
  }
}

void updatePID() {

  float dT = 1 / controller_frequency;  //50/1000; //TBR: idealized

  // Quaternion to Euler conversion
  float sinr_cosp = 2.0 * (quatW * quatX + quatY * quatZ);
  float cosr_cosp = 1.0 - 2.0 * (quatX * quatX + quatY * quatY);
  float body_x_rot = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0 * (quatW * quatY - quatZ * quatX);

  if (abs(sinp) >= 1)
    body_y_rot = copysign(PI / 2, sinp);  // Handle out-of-range values
  else
    body_y_rot = asin(sinp);

  body_x_rot *= RAD_TO_DEG;
  body_y_rot *= RAD_TO_DEG;

  float current_body_x_error = target_body_x_rot - body_x_rot;
  float current_body_y_error = target_body_y_rot - body_y_rot;

  float d_body_x_error = (current_body_x_error - last_body_x_error) / (micros() - lastPIDTime) * 1000000;
  float d_body_y_error = (current_body_y_error - last_body_y_error) / (micros() - lastPIDTime) * 1000000;

  if (millis() - ignitionDelayTimer > ignitionDelay) {                         //only build the I response when the motor is actually firing
    integrated_body_x_error = integrated_body_x_error + current_body_x_error;  //TBR: *dT????
    integrated_body_y_error = integrated_body_y_error + current_body_y_error;
  }
  if (millis() - ignitionDelayTimer > ignitionDelay) {  // <= ignitionDelayTimer + ignitionDelay + expected_motor_burn_time
    float time = (millis() - (ignitionDelayTimer + ignitionDelay)) / 1000.0;
    
    float thrust = nearestThrust(time);

    if(abs(thrust - lastThrust) > 35){ //weird predictable jump happening, bandaid lol
      thrust = lastThrust;
    }
    else{
      lastThrust = thrust;
    }

    // LOOKUP FOR GAINS
    kp_body_x_rot = kp_body_x_rot_original * (maxThrust/thrust);
    kp_body_y_rot = kp_body_y_rot_original * (maxThrust/thrust);
    kd_body_x_rot = kd_body_x_rot_original * (maxThrust/thrust);
    kd_body_y_rot = kd_body_y_rot_original * (maxThrust/thrust);
    ki_body_x_rot = ki_body_x_rot_original * (maxThrust/thrust);
    ki_body_y_rot = ki_body_y_rot_original * (maxThrust/thrust);
    // comCode = 500;
  }
  else{
    kp_body_x_rot = kp_body_x_rot_original;
    kp_body_y_rot = kp_body_y_rot_original;
    kd_body_x_rot = kd_body_x_rot_original;
    kd_body_y_rot = kd_body_y_rot_original;
    ki_body_x_rot = ki_body_x_rot_original;
    ki_body_y_rot = ki_body_y_rot_original;
  }

  tvc_x_rot = kp_body_x_rot * current_body_x_error + kd_body_x_rot * d_body_x_error + ki_body_x_rot * integrated_body_x_error;
  tvc_y_rot = kp_body_y_rot * current_body_y_error + kd_body_y_rot * d_body_y_error + ki_body_y_rot * integrated_body_y_error;

  last_body_x_error = current_body_x_error;
  last_body_y_error = current_body_y_error;

  //comCode = d_body_x_error;

  // comCode = body_x_rot; //tvcServoXAngle;
  lastPIDTime = micros();
}

float nearestThrust(float thrustTime) {
  if(thrustTime >= timeArray[timeArrayCounter]){
    timeArrayCounter++;
  }
  return thrustArray[timeArrayCounter];
}

// float linearInterpolate(float x, float data[][2]) {
//   float x0, x1, y0, y1;
//   for (int i = 0; i < sizeof(data) / (sizeof(data[0][0]) * 2); i++) {
//     if (x > data[i][0] && x < data[i + 1][0]) {
//       y0 = data[i][1];
//       y1 = data[i + 1][1];
//       x0 = data[i][0];      //lower bound
//       x1 = data[i + 1][0];  //upper bound
//       return (y0 + ((y1 - y0) * ((x - x0) / (x1 - x0))));
//     }
//   }
// }

void codeLight() {
  if (millis() - lightTimer > 500) {
    if (codeLightEnabled == true) {
      digitalWrite(LEDIndicatorPin, LOW);
      codeLightEnabled = false;
    } else {
      digitalWrite(LEDIndicatorPin, HIGH);
      codeLightEnabled = true;
    }
    lightTimer = millis();
  }
}

void guidance() {  //tbr: fix timing of each to 1, 10, 50hz etc
  if (millis() - tenHzTimer > 100) {
    // readGPS();
    readBMPPressure();
    tenHzTimer = millis();
    calcPressureVariance();
  }
  if (millis() - onehundredHzTimer > 10) {
    readLinAccel();
    readQuaternion();
    onehundredHzTimer = millis();
  }
  //TBR: this is where all data filtering should go
}

void pressurePropogation() {
  if (pressurePropogatorCounter == 10) {
    pressurePropogatorCounter = 0;
  }
  pressurePropogator[pressurePropogatorCounter] = bmpPressure;
  pressurePropogatorCounter++;
  padPressure = 0;
  for (int i = 0; i < 10; i++) {
    padPressure += pressurePropogator[i];
  }
  padPressure = padPressure / 10;
  altitudeOffset = abs(44330 * (1 - pow((bmpPressure / padPressure), (1 / 5.255))));
}

void fireDescentMotor() {  //TBR: write handler at end of loop to disable motor after some time check;
  pyro2Gate = HIGH;
  digitalWrite(pyro2GatePin, pyro2Gate);
  pyroStartTimer = millis();
}

void descentMotorCeaseFireHandler() {
  if (millis() - pyroStartTimer > 1000 && pyro2Gate == HIGH) {  //TBR: dial in the timing of how long the channel is open before a no thrust event turns into an abort
    pyro2Gate = LOW;
    digitalWrite(pyro2GatePin, pyro2Gate);
  }
}

void vehicleResafe() {
  machineState = AWAITGOFORLAUNCH;
  clearedForLaunch = false;
}

void recycleVehicle() {  //TBR: implement all boolean and var resets
  machineState = AWAITGOFORLAUNCH;
  TVCX.write(90);
  TVCY.write(90);
  telemetryRate = 20;
}

// void readGPS(){
//   decimalLatitude = GPS.latitudeDegrees;   //TBR: MUST BE ADDED BACK!!!!!!!-------------------------------------------------------------------------------------------------
//   decimalLongitude = GPS.longitudeDegrees; //TBR: MUST BE ADDED BACK!!!!!!!-------------------------------------------------------------------------------------------------
//   rawSpeed = GPS.speed;
// }

void readLinAccel() {
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  linAccelX = linearAccelData.acceleration.x;
  linAccelY = linearAccelData.acceleration.y;
  linAccelZ = linearAccelData.acceleration.z;
}

void readQuaternion() {
  imu::Quaternion quat = bno.getQuat();
  quatW = quat.w();
  quatX = quat.x();
  quatY = quat.y();
  quatZ = quat.z();
}

void readBMPPressure() {
  bmpTemperature = bmp.readTemperature();
  bmpPressure = bmp.readPressure();
  //bmpAltitude = bmp.readAltitude(1013.25);
  altitude = altitudeOffset + 44330 * (1 - pow((bmpPressure / padPressure), (1 / 5.255)));
  if (machineState > 3) {
    if (altitude > maxAltitude) {
      maxAltitude = altitude;
    }
  }
}

void calcPressureVariance() {
  pressureCheck[pressureCheckCounter] = bmpPressure;  //log pressure
  pressureCheckCounter++;
  if (pressureCheckCounter >= pressureCheckLength) pressureCheckCounter = 0;

  float averagePressure = 0;
  for (int i = 0; i < pressureCheckLength; i++) {  //find the average pressure of the last pressureCheckLength readings
    averagePressure += pressureCheck[i];
  }
  averagePressure = averagePressure / pressureCheckLength;

  float varSum = 0;
  for (int i = 0; i < pressureCheckLength; i++) {
    varSum += pow((pressureCheck[i] - averagePressure), 2);  //summation of square of reading - average
  }
  pressureVariance = varSum / pressureCheckLength;  //variance in pressure data over pressureCheckLength readings
}

void calcAltitudeVariance() {
  /*
  altitudeCheck[]
  altitudeCheckCounter
  altitudeCheckLength
  altitudeVariance
  */
  altitudeCheck[altitudeCheckCounter] = altitude;  //log pressure
  altitudeCheckCounter++;
  if (altitudeCheckCounter >= altitudeCheckLength) altitudeCheckCounter = 0;

  float averageAltitude = 0;
  for (int i = 0; i < altitudeCheckLength; i++) {  //find the average pressure of the last pressureCheckLength readings
    averageAltitude += altitudeCheck[i];
  }
  averageAltitude = averageAltitude / altitudeCheckLength;

  float altVarSum = 0;
  for (int i = 0; i < altitudeCheckLength; i++) {
    altVarSum += pow((altitudeCheck[i] - averageAltitude), 2);  //summation of square of reading - average
  }
  altitudeVariance = altVarSum / altitudeCheckLength;  //variance in pressure data over pressureCheckLength readings
}

void callAbort() {
  abortFlight = true;
  postAbortTimer = millis();
}

void vehicleHealthCheck() {
  //TBR: check continuity states and maybe for good data?
  vehicleHealthy = true;
  errorCode = 0;
  if (quatW != 0.0 && quatX != 0.0 && quatY != 0.0 && quatZ != 0.0) {  //this is unreadable ah hell
  } else {
    errorCode = 1;
  }

  if (continuityState2 == true) {  //TBR: continuityState2 == true, FOR ASCENT TEST THIS DOESNT NEED TO HAPPEN, REINTEGRATE LATER
  } else {
    errorCode = 3;
  }
  // if(!clearGPSError){ //allows us to clear the flag for bad gps just in case
  //   if((decimalLatitude > 2 || decimalLatitude < -2) && (decimalLongitude > 2 || decimalLongitude < -2)){
  //   }
  //   else{
  //     errorCode = 4;
  //   }
  // }

  if (continuityState1 == true) {
  } else {
    //errorCode = 2;
  }

  // if(machineState == AWAITLIFTOFF && errorCode != 0){ //if there are errors present roll back the
  //     clearedForLaunch = false;                       //State from await liftoff to await go for
  //     vehicleHealthy = false;                         //Launch, this will stop erroneous button presses
  //     machineState = AWAITGOFORLAUNCH;                //When things are awaiting initialization/fixes
  // }                                                   //NOTE: in full flight software the button is doubling to ignite the motor
}

void printXBee() {
  XBee.println(dataString);
}

void updateData(uint32_t datatimer, int machineState) {  //may be smart to make a separate data compiler that is less intensive for scarier parts of the flight
  //This method can lead to fragmented data and a program crash if the program memory fills
  //may be worth setting a custom char [] buffer then using strcat()
  dataString = "";
  dataString += String(datatimer);
  dataString += ",";
  dataString += String(linAccelX, 2);
  dataString += ",";
  dataString += String(linAccelY, 2);
  dataString += ",";
  dataString += String(linAccelZ, 2);
  dataString += ",";
  dataString += String(quatW, 4);  //TBR: REALLY SHOULD CHECK A CUT DOWN VERSION UNDER 120 BYTES/S
  dataString += ",";               //WILL VERIFY BUFFER CLEARING LAG
  dataString += String(quatX, 4);
  dataString += ",";
  dataString += String(quatY, 4);
  dataString += ",";
  dataString += String(quatZ, 4);
  dataString += ",";
  // dataString += String(decimalLatitude, 6);
  // dataString += ",";
  // dataString += String(decimalLongitude, 6); //TBR: CHANGE BACK TO decimalLongitude
  // dataString += ",";
  dataString += String(altitude, 2);  //bmpPressure  //TBR: change back to altitude!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  dataString += ",";
  if (continuityState1 == 1) { dataString += "1,"; }  //really doesn't have to be comma delimited as the length is constant
  else {
    dataString += "0,";
  }
  if (pyro1Gate == HIGH) { dataString += "1,"; }  //TBR: check this implementation
  else {
    dataString += "0,";
  }
  if (continuityState2 == 1) {
    dataString += "1,";
  } else {
    dataString += "0,";
  }
  if (pyro2Gate == HIGH) { dataString += "1,"; }  //TBR: check this implementation
  else {
    dataString += "0,";
  }
  dataString += String(comCode);
  dataString += ",";
  dataString += String(errorCode);
  dataString += ",";
  dataString += String(machineState);
  dataString += ",X";
}

void attitudeControlTestUpdateData(uint32_t datatimer, int machineState) {
  altDataString = "";
  altDataString += String(datatimer);
  altDataString += ",";
  altDataString += String(quatW, 4);
  altDataString += ",";
  altDataString += String(quatX, 4);
  altDataString += ",";
  altDataString += String(quatY, 4);
  altDataString += ",";
  altDataString += String(quatZ, 4);
  altDataString += ",";
  altDataString += String(tvc_x_rot);
  altDataString += ",";
  altDataString += String(tvc_y_rot);
  altDataString += ",";
  if (continuityState2 == 1) {
    altDataString += "1,";
  } else {
    altDataString += "0,";
  }
  if (pyro2Gate == HIGH) { altDataString += "1,"; }  //TBR: check this implementation
  else {
    altDataString += "0,";
  }
  altDataString += String(errorCode);
  altDataString += ",";
  altDataString += String(machineState);
  altDataString += ",X";
}

void telemetry() {
  if (millis() - telemetryTimer > (int)1000 / telemetryRate) {
    printXBee();
    telemetryTimer = millis();
    //Serial.println(dataString); //TBR: remove
  }
}

void releaseLegs() {
  LANDINGLEGSERVO.write(160);
}

void abortChargeFire() {
  if (millis() - abortPyroTimer > 1000) {  //TBR: DIAL IN TIME  //TBR: FOR GROUND STATION time should be higher maybe 3 seconds
    abortTriggered = false;
    abortFirstTrigger = true;
    pyro1Gate = LOW;
  }
  if (!abortTriggered) {
    abortPyroTimer = millis();
    digitalWrite(pyro1GatePin, LOW);
  } else {
    if (abortFirstTrigger) {
      abortPyroTimer = millis();
      abortFirstTrigger = false;
      pyro1Gate = HIGH;
    }
    digitalWrite(pyro1GatePin, HIGH);
  }
}

void pyroContinuityCheck() {
  if (millis() - continuityTimer > 100) {
    continuityState1 = digitalRead(pyro1ContinuityPin);
    continuityState2 = digitalRead(pyro2ContinuityPin);
    continuityTimer = millis();
  }
}

void memClearHandler() {
  if (memClearAttempt) {
    comCode = 4;               //if there has been an attempt to clear memory, tell ground station
    if (memClearConfirm) {     //if ground has confirmed a memory clear
      extChipSdDump();         //write current data to SD card
      extChipFormat();         //clear on board memory
      memClearDenied = false;  //reset all associated logic
      memClearConfirm = false;
      memClearAttempt = false;
      comCode = 5;  //tell ground station the memory has been logged and wiped
    }
    if (memClearDenied) {      //if ground denies attempt to clear memory
      memClearDenied = false;  //reset all associated logic as if nothing happened
      memClearConfirm = false;
      memClearAttempt = false;
      comCode = 0;
    }
  }
}

void updateXBee() {
  if (XBee.available()) {
    char xbeeChar = XBee.read();
    if (xbeeChar == 'L') {  //final launch flag clear with launch button
      clearedForLaunch = true;
    }
    if (xbeeChar == 'A') {  //abort the flight
      callAbort();
    }
    if (xbeeChar == 'R') {  //recycle //TBR: INCOMPLETE method done in a rush
      recycleVehicle();
    }
    // if(xbeeChar == 'G'){        //if bad gps data, continue anyways
    //   clearGPSError = true;
    // }
    if (xbeeChar == 'C') {  //start the process of wiping the flight computer memory
      memClearAttempt = true;
    }
    if (xbeeChar == 'c') {  //if we confirm the memory clear ground side, then record all current data to sd card and clear the memory
      memClearConfirm = true;
    }
    if (xbeeChar == 'd') {  //abort the process of clearing the flight computer memory
      memClearDenied = true;
    }

    if (xbeeChar == 'm') {
      extChipSdDump();
    }

    if (xbeeChar == 'o') {
      LANDINGLEGSERVO.write(160);
    }
    if (xbeeChar == 'l') {
      LANDINGLEGSERVO.write(120);
    }

    if (xbeeChar == 'T') {
      timeArrayCounter = 0;
      telemetryRate = 4;
      loopCtr = 0;
      firstLoopCtr = true;
      loopTime = millis();
      ignitionDelayTimer = millis();
      //releaseLegs();
      lastPIDTime = micros();
      integrated_body_x_error = 0;
      integrated_body_y_error = 0;
      machineState = TERMINALGUIDANCE;
    }

    if (xbeeChar == 'Q') {
      timeArrayCounter = 0;
      telemetryRate = 4;
      fireDescentMotor();
      lastPIDTime = micros();
      ignitionDelayTimer = millis();
      integrated_body_x_error = 0;  //TBR: need these to be reset when the actual thrust starts
      integrated_body_y_error = 0;
      machineState = TERMINALGUIDANCE;
    }
    if (xbeeChar == '-') {
      tvc_x_center -= 1;
    }
    if (xbeeChar == '=') {
      tvc_x_center += 1;
    }
    if (xbeeChar == '[') {
      tvc_y_center -= 1;
    }
    if (xbeeChar == ']') {
      tvc_x_center += 1;
    }

    // String xbeeConverted = (String) xbeeChar;

    // if(xbeeConverted.toInt() != 0){  //if the input is an integer
    //   numBuffer += xbeeChar;         //then append to current buffer
    // }
    // else{                               //if the input is not an integer
    //   inputNumber = xbeeConverted.toInt();  //create integer from buffer
    //   numBuffer = "";                   //clear the buffer
    // }
    // //the above else is redundancy, because the integer should be terminated by 't' //TBR: implement this
    // if(xbeeChar == 'y'){                  //attempt to terminate the integer creation
    //   if(numBuffer != ""){                //if the numBuffer isn't already clear; this will stop the inputNumber from getting replaced by the empty numBuffer
    //     inputNumber = xbeeConverted.toInt();  //create integer from buffer
    //     numBuffer = "";                   //clear the buffer
    //   }
    // }
  }
}

void sdHeader() {
  if (firstLoop && SDCardInserted) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("timer,accelx,accely,accelz,quatW,quatX,quatY,quatZ,decimalLatitude,decimalLongitude,bmpPressure,bmpTemperature,bmpAltitude,pyro1continuity,pyro1gate,pyro2continuity,pyro2gate,machineState,StopByte");
      dataFile.close();
    }
    firstLoop = false;
  }
}

void sdCardPrint() {
  if (printToSD) {
  }
}

void extChipLog() {
  extFile = extChip.open("datalog.txt", FILE_WRITE);  //TBR: really hope LittleFS doesn't throw a fit about datalog being used on the SD card
  if (extFile) {
    extFile.println(altDataString);  //DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE DATA GOES HERE
    extChipPrintCount++;
  } else {
    chipFailure = true;
  }
  extFile.close();
}

void extChipSdDump() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);  //TBR: should this file be declared globally?
  extFile = extChip.open("datalog.txt", FILE_READ);
  while (extFile.available()) {            //returns 0 when index advances beyond existing data
    dataFile.print((char)extFile.read());  //dataFile.read() returns the next byte and advances the file index
  }
  dataFile.close();
  dataFile = SD.open("datalog.txt", FILE_READ);  //previously in write mode must change
  dataFile.seek(dataFile.size());                //advance sd card file to last character
  int checkLength = 10;
  if (dataFile.size() > checkLength - 1 && extFile.size() > checkLength - 1) {  //make sure we wont cause an exception when we seek
    for (int i = 1; i <= checkLength; i++) {                                    //loop
      int sdCardPos = dataFile.size() - checkLength + i;                        //
      int extChipPos = extFile.size() - checkLength + i;
      dataFile.seek(sdCardPos);
      extFile.seek(extChipPos);
      if (dataFile.peek() != extFile.peek()) {  //check for matching characters
        dataSaveAnomaly = true;
        Serial.println("DATA ANOMALY DETECTED");
        break;
      }
    }
  }
  dataFile.seek(dataFile.size());  //extra safety, should not be needed as the indexes will
  extFile.seek(extFile.size());    //be corrected upon the next initialization through .open
  dataFile.close();
  extFile.close();
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.print("Execution Time: ");
  dataFile.print(millis() - globalTimer);
  dataFile.print(" ms");
  dataFile.close();
  Serial.println("Done!");
}

void extChipFormat() {  //TBR: terrifyingly easy
  extChip.quickFormat();
}

void customWrite(String inString) {
  altDataString = inString;
  extChipLog();
  altDataString = "";
}

void loadDNT() {
  dntFile = SD.open("DNT.txt", FILE_READ);  //TBR: make sure file name is DNT ---------------------
  String dntAssembler = "";
  int dataTypeCounter = 1;
  int discretizationCounter = 0;
  DNTLength = 0;
  while (dntFile.available()) {
    char inChar = (char)dntFile.read();  //reads current indexed character then increments index
    if (inChar != ',') {
      dntAssembler += inChar;
    } else {  //if assembly complete
      float dntFloat = dntAssembler.toFloat();
      switch (dataTypeCounter) {
        case 1:
          {
            t[discretizationCounter] = dntFloat;
            break;
          }
        case 2:
          {
            x_1[discretizationCounter] = dntFloat;
            break;
          }
        case 3:
          {
            y_1[discretizationCounter] = dntFloat;
            break;
          }
        case 4:
          {
            x_2[discretizationCounter] = dntFloat;
            break;
          }
        case 5:
          {
            y_2[discretizationCounter] = dntFloat;
            break;
          }
      }
      dataTypeCounter++;
      if (dataTypeCounter == 6) {
        discretizationCounter++;
        dataTypeCounter = 1;
      }
      dntAssembler = "";
    }
  }
  dntFile.close();
  DNTLoaded = true;
  for (int i = 0; i < 400; i++) {
    if (x_1[i] != 0.0) {
      DNTLength++;
    } else {
      break;
    }
  }
  Serial.print("DNT Length: ");
  Serial.println(DNTLength);
  for (int i = 0; i < DNTLength; i++) {  //TBR: REMOVE THIS OR MAKE IT COMMANDED
    Serial.print(t[i], 6);
    Serial.print(",");
    Serial.print(x_1[i], 6);
    Serial.print(",");
    Serial.print(y_1[i], 6);
    Serial.print(",");
    Serial.print(x_2[i], 6);
    Serial.print(",");
    Serial.println(y_2[i], 6);
  }
}

void buzzerHandler() {  //TBR: CHECK BUZZER CODE, arrays were out of bounds but it seems like the compiler took care of it???
  if (SOUND_ON && sounds[soundIndex] != 0) {
    if (millis() > soundTimer + sounds[soundHandlerIndex - 1][1] + 30) {
      if (sounds[soundHandlerIndex][0] != 0) {
        tone(buzzerPin, sounds[soundHandlerIndex][0], sounds[soundHandlerIndex][1]);
        soundHandlerIndex++;
      }
      soundTimer = millis();
    }
  }
}

void playSound(int buzzerFrequency, int buzzTime) {
  if (soundIndex == maxSoundBufferLength - 1) {
    for (int i = 0; i < maxSoundBufferLength; i++) {
      sounds[i][0] = 0;  //once again, arduino does not actually have a NULL type, it just calls it zero when you type NULL
      sounds[i][1] = 0;
    }
    soundIndex = 1;
  }
  sounds[soundIndex][0] = buzzerFrequency;
  sounds[soundIndex][1] = buzzTime;
  soundIndex++;
}

// int checkDNT(int abort_counts, float current_lat, float current_long){

//   if(millis() - current_DNT_time > 1000*(t[2] - t[1])){
//     current_DNT_time = millis();
//     abort_counts = 0;
//     return abort_counts;
//   }

//   bool check_1s[] = {};
//   bool check_2s[] = {};
//   bool check_3s[] = {};
//   bool check_4s[] = {};

//   for(int i = 0; i < DNTLength; i++){

//     bool check_1 = false;
//     bool check_2 = false;
//     bool check_3 = false;
//     bool check_4 = false;

//     float left_x_1 = x_1[i];
//     float left_y_1 = y_1[i];
//     float left_x_2 = x_1[i+1];
//     float left_y_2 = y_1[i+1];

//     float right_x_1 = x_2[i];
//     float right_y_1 = y_2[i];
//     float right_x_2 = x_2[i+1];
//     float right_y_2 = y_2[i+1];

//     bool special_case_1_flag = false;

//     if(left_x_1 == right_x_1 && left_y_1 == right_y_1){ //TBR: could probably be if i = 0
//       special_case_1_flag = true;
//     }
//     else if(left_x_1 == left_x_2 && left_y_1 == left_y_2){
//       continue;
//     }
//     else if(right_x_1 == right_x_2 && right_y_1 == right_y_2){
//       continue;
//     }

//     if(special_case_1_flag){
//       check_1 = true;
//     }
//     else{
//       float vec_1[2] = {right_x_1 - left_x_1, right_y_1 - left_y_1};
//       float vec_1_mag = sqrt((pow(vec_1[0],2)+pow(vec_1[1],2))); //remember indexes are 0 and 1 not 1 and 2
//       float vec_2[2] = {left_x_2 - left_x_1, left_y_2 - left_y_1};
//       float vec_2_mag = sqrt((pow(vec_2[0],2)+pow(vec_2[1],2)));
//       float vec_3[2] = {current_long - left_x_1, current_lat - left_y_1};
//       float vec_3_mag = sqrt((pow(vec_3[0],2)+pow(vec_3[1],2)));

//       float vec1_vec2_dot = (vec_1[0] * vec_2[0]) + (vec_1[1] * vec_2[1]);

//       float vec_1_2_acos_input = vec1_vec2_dot/(vec_1_mag * vec_2_mag);

//       if(vec_1_2_acos_input < -1){
//         vec_1_2_acos_input = -1;
//       }
//       else if(vec_1_2_acos_input > 1){
//         vec_1_2_acos_input = 1;
//       }

//       float angle_vec1_vec2 = acos(vec_1_2_acos_input);

//       float vec1_vec3_dot = (vec_1[0] * vec_3[0]) + (vec_1[1] * vec_3[1]);

//       float vec_1_3_acos_input = vec1_vec3_dot/(vec_1_mag * vec_3_mag);

//       if(vec_1_3_acos_input < -1){
//         vec_1_3_acos_input = -1;
//       }
//       else if(vec_1_3_acos_input > 1){
//         vec_1_3_acos_input = 1;
//       }

//       float angle_vec1_vec3 = acos(vec_1_3_acos_input);
//       // Serial.print("angle_vec1_vec2: ");
//       // Serial.print(angle_vec1_vec2);
//       // Serial.print(", angle_vec1_vec3: ");
//       // Serial.print(angle_vec1_vec3);
//       if(angle_vec1_vec2 >= angle_vec1_vec3){
//         check_1 = true;
//       }
//       // Serial.print(", check_1: ");
//       // Serial.println(check_1);
//     }

//     check_1s[i] = check_1;
//     check_2s[i] = check_2;
//     check_3s[i] = check_3;
//     check_4s[i] = check_4;
//   }
//   // for(int i = 0; i < DNTLength; i++){
//   //   delay(5);
//   //   if(check_1s[i]) Serial.print("1");
//   //   if(!check_1s[i]) Serial.print("0");
//   // }
//   // Serial.println("END");

//   return 0; //TBR: REMOVE PROBABLY
// }

// void checkTrajectory(){
//   abort_ctr = checkDNT(abort_ctr, decimalLongitude, decimalLatitude);
//   if(abort_ctr > 5){
//     abortFlight = true;
//   }
// }



//TBR: add checksum to sentences
