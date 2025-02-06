#include <Servo.h>
#include <math.h>
//#include <Vector.h>
//#include <Wire.h> //TBR: may need for i2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <EEPROM.h>
#include "HX711.h"
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
#define GPSSerial Serial7
const int chipSelect = BUILTIN_SDCARD;
#define LOADCELL_DOUT_PIN 18
#define LOADCELL_SCK_PIN 19

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
  INFLIGHTABORT,
  STATICFIRE
};

enum state machineState = STATICFIRE;

uint32_t globalTimer = millis();
uint32_t XBeeTimer = millis();  //TBR: artifact of string assembly method buffer clearing
uint32_t telemetryTimer = millis();
uint32_t timeSinceLastBuzz = millis();
uint32_t lightTimer = millis();
uint32_t scaleTimer = millis();
uint32_t pyroTimer = millis();

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

double decimalLatitude = 1.00;   //measured
double decimalLongitude = 1.00;  //measured
float rawSpeed = 0;              //derived

float bmpPressure = 0;     //measured
float bmpTemperature = 0;  //measured
float bmpAltitude = 0;     //"measured" (derived within Adafruit_BMP280.h)

int fiftyHzTimer = 0;  //various counters for time based operations
int onehundredHzTimer = 0;
int tenHzTimer = 0;
int oneHzTimer = 0;
int onethirdHzTimer = 0;
unsigned int telemetryRate = 10;  //hz //TBR: may be a ground station problem, data wont report over 10 hz

//TBR: should all be reset on vehicle recycle
double maxbmpAltitude = 0;     //TBR: must be declared globally to eliminate first cycle apogeeDetected error
float altitudePropogator = 0;  //TBR: if there is something that doesn't need to be global it is the propogators, fix method
int altitudePropogatorCounter = 0;
float altitudeOffset = 0;
float adjustedAltitude = 0;
float gravity = 9.79;
String dataString = "";
String assembledString = "";
String assemblyString = "";
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

//TBR: should all be reset on vehicle recycle
bool abortFlight = false;
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
bool printToSD = false;
//bool totalString = false;           //TBR: artifact of string assembly, may still need
bool clearedForLaunch = false;  //the almighty all powerful final launch flag
bool triggered = false;
bool firstTrigger = true;
bool thrustDetected = false;
bool firing = false;

int tvcServoXAngle = 90;  //TBR: assumes 90 degrees as neutral position
int tvcServoYAngle = 90;  //TBR: assumes 90 degrees as neutral position
int landingLegServoAngle = 90;

float offset = 0.0;               // Offset value from the scale without any load
float calibrationFactor = 205.0;  // Calibration factor determined by the experiment (205 worked well)
const float g = 9.81;

Servo TVCX;
Servo TVCY;
Servo LANDINGLEGSERVO;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bmp;
SoftwareSerial XBee(21, 20);
Adafruit_GPS GPS(&GPSSerial);
HX711 scale;

void setup() {
  Serial.begin(115200);

  TVCX.attach(tvcServoXPin);
  TVCY.attach(tvcServoYPin);
  LANDINGLEGSERVO.attach(landingLegServoPin);

  pinMode(pyro1ContinuityPin, INPUT);
  pinMode(pyro1GatePin, OUTPUT);
  pinMode(pyro2ContinuityPin, INPUT);
  pinMode(pyro2GatePin, OUTPUT);
  pinMode(LEDIndicatorPin, OUTPUT);
  pinMode(LEDMainPin, OUTPUT);

  digitalWrite(LEDMainPin, HIGH);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  //TBR: test 5HZ
  GPSSerial.println(PMTK_Q_RELEASE);          //returns PMTK_DT_RELEASE, probably unnecessary but conventional and works

  if (!SD.begin(chipSelect)) {
    //TBR: add buzzer handler here for SD card failure, as this will require near total deintegration
    SDCardInserted = false;
  }

  XBee.begin(9600);

  // initialization();

  delay(10);

  // scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // delay(10);
  // scale.tare();
  // delay(10);
  // scale.set_scale(calibrationFactor);
  // delay(10);

  //initialization();
  
}

void loop() {

  sdHeader();

  globalTimer = millis();  //set to the system time each cycle

  //updateGPSBuffer();

  updateXBee();

  //if(abortFlight) machineState = INFLIGHTABORT;
  //staticFire();
  //  switch(machineState){
  //    case ASSEMBLY: assembly();
  //            break;
  //    case INITIALIZATION: initialization();
  //            break;
  //    case AWAITGOFORLAUNCH: awaitGoForLaunch();
  //            break;
  //    case AWAITLIFTOFF: awaitLiftoff();
  //            break;
  //    case POWEREDASCENT: poweredAscent();
  //            break;
  //    case UNPOWEREDASCENT: unpoweredAscent();
  //            break;
  //    case UNPOWEREDDESCENT: unpoweredDescent();
  //            break;
  //    case TERMINALGUIDANCE: terminalGuidance();
  //            break;
  //    case LANDED: landed();
  //            break;
  //    case INFLIGHTABORT: inFlightAbort();
  //            break;
  //    case STATICFIRE: staticFire();
  //            break;
  //  }
  staticFire();
  //  descentMotorCeaseFireHandler();

  buzzerHandler();

  //updateData(globalTimer, machineState);

  //telemetry();

  //  staticFireTelemetry();

  //  sdCardPrint();
}

void assembly() {
  TVCX.write(90);
  TVCY.write(90);
  delay(1000);
  TVCX.write(120);
  TVCY.write(120);
  delay(300);
  TVCX.write(90);
  TVCY.write(90);
  delay(1000);
  TVCX.write(60);
  TVCY.write(60);
  delay(300);
}

void initialization() {
  while (!bnoInitialized) {
    if (!bno.begin()) {
      bnoInitialized = false;
      playSound(800, 800);
    } else {
      bnoInitialized = true;
      playSound(3400, 300);
    }
  }
}

void awaitGoForLaunch() {
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
  guidance();
}

void awaitLiftoff() {
}

void poweredAscent() {
}

void unpoweredAscent() {
}

void unpoweredDescent() {
}

void terminalGuidance() {
}

void landed() {
}

void inFlightAbort() {
}

void staticFire() {
  
  if (millis() - scaleTimer > 100) {
    //readQuaternion();  
    float load_grams = 0.0; //scale.get_units();  // Get the load in grams 

    continuityState1 = digitalRead(pyro1ContinuityPin);

    if (millis() - pyroTimer > 1000) { //TBR: DIAL IN TIME  //TBR: FOR GROUND STATION time should be higher maybe 3 seconds
      triggered = false;
      firstTrigger = true;
    }

    if (!triggered) {
      pyroTimer = millis();
      digitalWrite(pyro1GatePin, LOW);
    } else {
      if (firstTrigger) {
        pyroTimer = millis();
        firstTrigger = false;
      }
      digitalWrite(pyro1GatePin, HIGH);
    }

    assemblyString = "";
    if (continuityState1 == 1) {
      assemblyString += "1";
      assemblyString += ",";
    } else {
      assemblyString += "0";
      assemblyString += ",";
    }
    if (triggered) {
      assemblyString += "1";
      assemblyString += ",";
    } else {
      assemblyString += "0";
      assemblyString += ",";
    }

    assemblyString += String(load_grams, 2);
    assemblyString += ",";

    assemblyString += String(globalTimer);
    assemblyString += ",";

    if (SDCardInserted && printToSD) {
      assemblyString += "1";
      assemblyString += ",";
    } else {
      assemblyString += "0";
      assemblyString += ",";
    }
    if (SDCardInserted && printToSD) sdCardPrint();

    assemblyString += 'X';

    XBee.print(assemblyString);

    scaleTimer = millis();
  }
}

void updateGPSBuffer() {
  char c = GPS.read();                 //read char from GPS buffer
  if (GPS.newNMEAreceived()) {         //if there is a new nmea sentence, then parse it
    if (!GPS.parse(GPS.lastNMEA())) {  //sets the newNMEAreceived() flag to false
      return;                          //just check for another sentence next cycle if it fails
    }
  }
}

void guidance() {  //tbr: fix timing of each to 1, 10, 50hz etc
  if (millis() - tenHzTimer > 100) {
    readGPS();
    readBMPPressure();
    tenHzTimer = millis();
  }
  if (millis() - onehundredHzTimer > 10) {
    readLinAccel();
    readQuaternion();
    onehundredHzTimer = millis();
  }
  //TBR: this is where all data filtering should go
}

void fireDescentMotor() {  //TBR: write handler at end of loop to disable motor after some time check;
                           //  pyro2Gate = HIGH;
                           //  digitalWrite(pyro2GatePin, pyro2Gate);
                           //  pyroStartTimer = millis();
  triggered = true;
  playSound(3000, 400);
}

void descentMotorCeaseFireHandler() {
  if (millis() - pyroStartTimer > 1000) {  //&& pyro2Gate == HIGH){ //TBR: dial in the timing of how long the channel is open before a no thrust event turns into an abort
    pyro1Gate = LOW;  //TBR: not sure if this needed to be changed from 2 to 1 for abort system but dont use this code
    digitalWrite(pyro2GatePin, pyro1Gate);
  }
}

void vehicleResafe() {
  machineState = AWAITGOFORLAUNCH;
  clearedForLaunch = false;
}

void recycleVehicle() {  //TBR: implement all boolean and var resets
  machineState = INITIALIZATION;
}

void readGPS() {
  decimalLatitude = GPS.latitudeDegrees;
  decimalLongitude = GPS.longitudeDegrees;
  rawSpeed = GPS.speed;
}

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
  bmpAltitude = bmp.readAltitude(1013.25);  //TBR: this method sucks and needs to be replaced with a velocity offset based value
}

void printXBee() {
  XBee.println(dataString);
}

void updateData(uint32_t datatimer, int machineState) {  //may be smart to make a separate data compiler that is less intensive for scarier parts of the flight
  dataString = "";
  dataString += String(datatimer);
  dataString += ",";
  dataString += String(linAccelX, 2);
  dataString += ",";
  dataString += String(linAccelY, 2);
  dataString += ",";
  dataString += String(linAccelZ, 2);
  dataString += ",";
  dataString += String(quatW, 6);
  dataString += ",";
  dataString += String(quatX, 6);
  dataString += ",";
  dataString += String(quatY, 6);
  dataString += ",";
  dataString += String(quatZ, 6);
  dataString += ",";
  dataString += String(decimalLatitude, 6);
  dataString += ",";
  dataString += String(decimalLongitude, 6);
  dataString += ",";
  dataString += String(bmpPressure, 2);
  dataString += ",";
  dataString += String(bmpTemperature, 2);
  dataString += ",";
  dataString += String(bmpAltitude, 2);
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
  dataString += String(machineState);
  dataString += ",X";
}

void telemetry() {
  if (millis() - telemetryTimer > (int)1000 / telemetryRate) {
    printXBee();
    telemetryTimer = millis();
    //Serial.println(dataString); //TBR: remove
  }
}

void staticFireTelemetry() {
}

void updateXBee() {
  if (XBee.available()) {
    char xbeeChar = XBee.read();
    //XBee.print(xbeeChar);
    if (xbeeChar == 'P') {
      playSound(1000, 200);
    }
    if (xbeeChar == 'S') {
      printToSD = true;
    }
    if (xbeeChar == 'R') {
      printToSD = false;
    }
    if (xbeeChar == 'F') {
      playSound(3000, 500);
      triggered = true;
    }


    //      switch(xbeeChar){
    //        case 'A':{  //manual abort
    //          abortFlight = true;
    //          break;
    //        }
    //        case 'P':{
    //          playSound(1000, 200);  //TBR: check this to make sure there's no crash during string parse ground side
    //          break;
    //        }
    //        case 'L':{
    //          clearedForLaunch = true;
    //          break;
    //        }
    ////        case 'R':{
    ////          recycleVehicle();
    ////          break;
    ////        }
    //        case 's':{
    //
    //          break;
    //        }
    //        case 'S':{
    //          printToSD = true;
    //          break;
    //        }
    //        case 'R':{
    //          printToSD = false;
    //          break;
    //        }
    //        case 'v':{
    //          vehicleResafe();
    //          break;
    //        }
    ////        case 'X':{  //MANUAL DESCENT MOTOR FIRE //TBR: should probably be removed for safety after static fire?
    ////          fireDescentMotor(); //TBR: static fire had this as "X", one must be changed
    ////          //digitalWrite(LEDIndicatorPin, HIGH);
    ////          break;              //'F' feels safer as X is used as the delimiter already
    ////        }
    //        default:{
    //          //TBR: implement erroneous packet detection, xbeeprint('f')
    //          break;
    //        }
    //        //break;
    //      }
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
  if (printToSD && SDCardInserted) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      //TBR: add quaternion values to assemblyString
      String newAssembly = assemblyString;
      newAssembly += String(quatW, 4);
      newAssembly += ",";
      newAssembly += String(quatX, 4);
      newAssembly += ",";
      newAssembly += String(quatY, 4);
      newAssembly += ",";
      newAssembly += String(quatZ, 4);
      newAssembly += ",";
      newAssembly += 'X';
      dataFile.println(newAssembly);
      dataFile.close();
    }
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
















//TBR: add checksum to sentences
