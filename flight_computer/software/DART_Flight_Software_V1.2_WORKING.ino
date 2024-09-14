




/*      _      _ _                    _           
       | |    (_) |                  (_)          
       | |     _| |__  _ __ __ _ _ __ _  ___  ___ 
       | |    | | '_ \| '__/ _` | '__| |/ _ \/ __|
       | |____| | |_) | | | (_| | |  | |  __/\__ \
       |______|_|_.__/|_|  \__,_|_|  |_|\___||__*/

       
                #include <Servo.h>
                #include <math.h>
                #include <Wire.h>
                //#include <SPI.h>   //may need to include this as the bmp280test sketch has it, however we are using i2c not spi so i think its unnecessary 
                #include <Adafruit_Sensor.h>
                #include <Adafruit_BNO055.h>
                #include <utility/imumaths.h>
                #include <Adafruit_BMP280.h>
                #include <SoftwareSerial.h> //for xbee
                //#include <Adafruit_GPS.h>     //surely
                #include <SD.h>
                #include <SPI.h>
                #include <string>
                #include <EEPROM.h>
//
                using namespace std;


/*
        _____  ______ _____ _               _____         _______ _____ ____  _   _  _____ 
       |  __ \|  ____/ ____| |        /\   |  __ \     /\|__   __|_   _/ __ \| \ | |/ ____|
       | |  | | |__ | |    | |       /  \  | |__) |   /  \  | |    | || |  | |  \| | (___  
       | |  | |  __|| |    | |      / /\ \ |  _  /   / /\ \ | |    | || |  | | . ` |\___ \ 
       | |__| | |___| |____| |____ / ____ \| | \ \  / ____ \| |   _| || |__| | |\  |____) |
       |_____/|______\_____|______/_/    \_\_|  \_\/_/    \_\_|  |_____\____/|_| \_|____*/


                #define throttleClampServoPin2 9
                #define throttleClampServoPin1 10
                #define tvcServo1Pin 11
                #define tvcServo2Pin 12
                #define pyro3GatePin 13
                #define pyro2GatePin 14
                #define pyro1GatePin 15
                #define landingLegServoPin 25
                #define LEDIndicatorPin 30
                #define LEDMainPin 31
                #define finServoPin1 41
                #define finServoPin2 40
                #define finServoPin3 39
                #define finServoPin4 38
                #define buzzerPin 35

                #define BMP280_ADDRESS 0x76

                //#define GPSSerial Serial1 //surely

                const int chipSelect = BUILTIN_SDCARD;
                
                uint32_t timer = millis();
                uint32_t globalTimer = millis();
                uint32_t XBeeTimer = millis();

                float accelX = 0;
                float accelY = 0;
                float accelZ = 0;
                float yawRaw = 0;
                float rollRaw = 0;
                float pitchRaw = 0;

                float bmpPressure = 0;
                float bmpTemperature = 0;
                float bmpAltitude = 0;

                int throttleClampServo1Angle = 90;
                int throttleClampServo2Angle = 90;

                int tvcServo1Angle = 90;
                int tvcServo2Angle = 90;
                
                int landingLegServoAngle = 90; //this is definitely wrong, needs to be updated by structures, wouldn't plug into system with this value 
                
                int finServo1Angle = 90;
                int finServo2Angle = 90;
                int finServo3Angle = 90;
                int finServo4Angle = 90;

                enum state {
                  ASSEMBLY,
                  INITIALIZING,
                  AWAITGOFORLAUNCH,
                  AWAITLIFTOFF,
                  POWEREDASCENT,
                  UNPOWEREDASCENT,
                  UNPOWEREDDESCENT,
                  TERMINALGUIDANCE,
                  LANDED,
                  INFLIGHTABORT
                };

                enum state machineState = INITIALIZING; //INITIALIZING; //ASSEMBLY;

                int millisecondUpdateRate = 20; //updates every 20 ms

                bool abortFlight = false; 

                double maxbmpAltitude = 0;

                bool SOUND_ON = true;

                

                bool bmppropogatorCompletion = false;

                bool bnoCalibrationCompletion = false;

                bool gravityMeasurementCompletion = false;

                
    
                float altitudePropogator = 0;

                int altitudePropogatorCounter = 0;

                double gravityPropogator = 0;

                int gravityPropogatorCounter = 0;
                

                float altitudeOffset = 0;

                float adjustedAltitude = 0;

                bool apogeeFirstTime = true;

                int timerCycleCounter = 0;

                String dataString = "";

                bool firstLoop = true;

                bool SDCardInserted = true;

                bool printToSerial = false;

                double measuredGravity = 9.79; //THIS WILL BE UPDATED ON THE PAD

                string assembledString = "";

                bool totalString = false;

                int stringMillisecondUpdateRate = 1500; //flush assembledString every ___ ms

                bool clearedForLaunch = false;

                double lastAccelMag = 0; //this may need to be really high idk it depends on how the code flows

                unsigned int addr = 0;

                //need to add to list of reset vars probably

                int eepromAddressCounter = 0;


/*       _____                                __   _____                               
        / ____|                              / /  / ____|                              
       | (___   ___ _ ____   _____  ___     / /  | (___   ___ _ __  ___  ___  _ __ ___ 
        \___ \ / _ \ '__\ \ / / _ \/ __|   / /    \___ \ / _ \ '_ \/ __|/ _ \| '__/ __|
        ____) |  __/ |   \ V / (_) \__ \  / /     ____) |  __/ | | \__ \ (_) | |  \__ \
       |_____/ \___|_|    \_/ \___/|___/ /_/     |_____/ \___|_| |_|___/\___/|_|  |__*/


                Servo THROTTLECLAMP1;
                Servo THROTTLECLAMP2;
                
                Servo TVC1;
                Servo TVC2;
                
                Servo LANDINGLEGSERVO;
                
                Servo FIN1;
                Servo FIN2;
                Servo FIN3;
                Servo FIN4;

                Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

                Adafruit_BMP280 bmp;

                SoftwareSerial XBee(0,1); 
                
                //Adafruit_GPS GPS(&GPSSerial); surely


void setup() {
  Serial.begin(115200);
  
  THROTTLECLAMP1.attach(throttleClampServoPin1); //assigns servo objects to the declared pins
  THROTTLECLAMP2.attach(throttleClampServoPin2);
  
  TVC1.attach(tvcServo1Pin);
  TVC2.attach(tvcServo2Pin);

  LANDINGLEGSERVO.attach(landingLegServoPin);

  FIN1.attach(finServoPin1);
  FIN2.attach(finServoPin2);
  FIN3.attach(finServoPin3);
  FIN4.attach(finServoPin4);

  delay(10); //these little delays throughout the setup seem to help sensor initialization

  
  bool bmpStarted = false;
  bool bnoStarted = false;
  
  while((!bmpStarted) || (!bnoStarted)){  //keep looping until both sensors turn on at the same time
    
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS);
      
    if (!status) {
      Serial.println("BMP280 failure to start");   
      if(SOUND_ON) tone(buzzerPin, 300, 200);
      delay(200);
    }
    else{
      bmpStarted = true;
      Serial.println("BMP280 Initialized Successfully");
      if(SOUND_ON) tone(buzzerPin, 2000, 100);
      delay(100);
    }
    
    //delay(100);

    if(!bno.begin()){
      Serial.print("BNO055 failure to start");
      if(SOUND_ON) tone(buzzerPin, 200, 200);
      delay(200);
    }
    else{
      bnoStarted = true;
      Serial.println("BNO055 Initialized Successfully");
      if(SOUND_ON) tone(buzzerPin, 3000, 100);
      delay(100);
    }

    delay(200);
  }

  if (!SD.begin(chipSelect)) {                      //SHOULD BE ADDED INTO THE ABOVE WHILE STATEMENT PROBABLY
    Serial.println("SD CARD FAILURE");              //now upon further thought it would make testing even more obtuse
    if(SOUND_ON) tone(buzzerPin, 800, 300);         //will simply make the computer scream that something is wrong
    delay(600);
    if(SOUND_ON) tone(buzzerPin, 800, 300);
    SDCardInserted = false;
    //return;
  }
  
  XBee.begin(9600);

  delay(100);
  
  Serial.println("All Sensors Initialized Successfully");
  
}

void loop() {
  
  if(firstLoop && SDCardInserted){
    File dataFile = SD.open("datalog.txt", FILE_WRITE);   //if this is the first time the program is running, then write the datafile header to make the csv format work
    if (dataFile) {                                       //nasty code, but it doesn't want to work in the setup function so we may be stuck with thiss
        dataFile.println("timer,accelX,accelY,accelZ,yawRaw,rollRaw,pitchRaw,bmpPressure,bmpTemperature,bmpAltitude,machineState,StopByte");
        dataFile.close();
      }
    firstLoop = false;
  }

  updateXBee();

  sensors_event_t orientationData, accelerometerData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  readIMU(&orientationData);
  
  readIMU(&accelerometerData);

  globalTimer = millis();

  if(millis() - timer > millisecondUpdateRate){  //everything in here runs every millisecondUpdateRate milliseconds
    
    timer = millis();
  
    readBMPPressure();
    
    timerCycleCounter++;
    
    if(timerCycleCounter == 5){
      printXBee();
      timerCycleCounter = 0;
    }

    //if(assembledString.find("Z") == string::npos) assembledString = "";
    
  }

  
  //ALL SENSOR INTAKE SHOULD GO ABOVE THIS LINE -------------------------------------

  adjustedAltitude = bmpAltitude - altitudeOffset;

  if(abortFlight) machineState = INFLIGHTABORT;
  
  switch(machineState){
    case ASSEMBLY: assembly();
            break;
    case INITIALIZING: initializing();         
            break;
    case AWAITGOFORLAUNCH: awaitGoForLaunch();         
            break; 
    case AWAITLIFTOFF: awaitLiftoff();   
            break;
    case POWEREDASCENT: poweredAscent();
            break;
    case UNPOWEREDASCENT: unpoweredAscent();
            break;
    case UNPOWEREDDESCENT: unpoweredDescent();  
            break;
    case TERMINALGUIDANCE: terminalGuidance();
            break;
    case LANDED: landed();
            break;
    case INFLIGHTABORT: inFlightAbort();
            break;
      
  }

  updateData(globalTimer, machineState);
  
  if(printToSerial) printDataTemp();

  if(SDCardInserted && clearedForLaunch) printSDCard();
  
//  if(firstLoop){
//    writeStringToEEPROM(eepromAddressCounter, "timer,accelX,accelY,accelZ,yawRaw,rollRaw,pitchRaw,bmpPressure,bmpTemperature,bmpAltitude");
//    firstLoop = false;
//    }
//  printEEPROM();

  //XBee.print(2);
  
  //delay(200);
  
  

}



void assembly(){
  
  //printDataTemp();
  if(abortFlight){
  Serial.println("ABORT TRIGGERED");
  }
  else{
    //Serial.println("NO ABORT");
  }
  
}

void initializing(){
    
    if(!bmppropogatorCompletion){
      
      if(altitudePropogatorCounter < 1000){
        altitudePropogator = altitudePropogator + bmpAltitude;
        altitudePropogatorCounter++;
      }
      else{
        altitudeOffset = altitudePropogator/1000;
        Serial.print("Altitude Offset: ");
        Serial.println(altitudeOffset);
        if(SOUND_ON) tone(buzzerPin, 3000, 300);
        bmppropogatorCompletion = true;
      }
      
    }
    
    else if(!gravityMeasurementCompletion){ 
  
      gravityPropogator = gravityPropogator + sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));    //measuring and propogating the magnitude of all acceleration
      
      gravityPropogatorCounter++;
      if(gravityPropogatorCounter == 1000){
        Serial.println("Gravity calibrated");
        measuredGravity = gravityPropogator/1000;                                                                 //if vehicle is still, this should give a good gravity meaurement
        if(SOUND_ON) tone(buzzerPin, 3500, 300);
        gravityMeasurementCompletion = true;
        }
    }
    
    else{
      uint8_t system, gyro, accel, mag;   //taking the calibration state of the bno055
      system = gyro = accel = mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      
      if(system == 3){                    //full system calibration happens at 3
        Serial.println("BNO calibrated");
        bnoCalibrationCompletion = true;
        if(SOUND_ON) tone(buzzerPin, 4000, 600);
        machineState = AWAITGOFORLAUNCH;
      }
    }

}

void awaitGoForLaunch(){

  if(clearedForLaunch){
    
    machineState = AWAITLIFTOFF;
    
  }

  
}


void awaitLiftoff(){
  double accelMag = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
  //Serial.print("accelMag: ");
  //Serial.println(accelMag);
  if(accelMag > 20){
    machineState = POWEREDASCENT; 
  }
  
}

void poweredAscent(){
  double accelMag = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
  if(accelMag < 14){                                                                          //this is a guess, surely
    machineState = UNPOWEREDASCENT; 
  }
}

void unpoweredAscent(){

  if(apogeeDetected()){
    machineState = UNPOWEREDDESCENT; 
  }
  
}

void unpoweredDescent(){
  if(adjustedAltitude < 10){
    machineState = TERMINALGUIDANCE;
  }
}

void terminalGuidance(){
  double accelMag = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
  if(accelMag < 10.5 && accelMag > 8.5){        // + - grav measured                                                                  //this is a guess, surely
    machineState = LANDED; 
  }
}

void landed(){
  
}

void inFlightAbort(){
  
}





bool apogeeDetected(){
  bool apogeeDetected = false;
  if(apogeeFirstTime){                    //without this the initial value of maxbmpAltitude will be zero, and if the altitude reading is 
    maxbmpAltitude = bmpAltitude;         //negative, then it will detect "apogee" on the first run
    apogeeFirstTime = false;
  }
  if(bmpAltitude > maxbmpAltitude) maxbmpAltitude = bmpAltitude;
  
  if(bmpAltitude < maxbmpAltitude - 20){ //if it is 20 meter below its max, thats hopefully apogee, can be dialed in later but this is safe for now
    apogeeDetected = true;
  }
  
  return apogeeDetected; //apogeeDetected;
}




void readIMU(sensors_event_t* event){
  if(event->type == SENSOR_TYPE_ACCELEROMETER){ //this could be the wrong one, may have to switch with linear acceleration.
    accelX = event->acceleration.x;
    accelY = event->acceleration.y;
    accelZ = event->acceleration.z;
  }
  else if(event->type == SENSOR_TYPE_ORIENTATION){
    yawRaw = event->orientation.x;
    rollRaw = event->orientation.y;
    pitchRaw = event->orientation.z;
  }
}

void readBMPPressure(){
  bmpTemperature = bmp.readTemperature();
  bmpPressure = bmp.readPressure();
  bmpAltitude = bmp.readAltitude(1013.25);  //this is a local condition which should be changed on launch day 
  
}

void updateXBee(){

  if(millis() - XBeeTimer > stringMillisecondUpdateRate){                       //THIS MUST GO AT SOME POINT, LEADS TO PACKET LOSS
      XBeeTimer = millis();
        if(assembledString.find("Z") == string::npos) assembledString = "";
      }
      
if (XBee.available()) {

    // Read next character in XBee buffer
    char xbeeChar = XBee.read();
    Serial.print("Char stream: ");
    Serial.println(xbeeChar);

    assembledString = assemble_string(assembledString, xbeeChar, "Z");
    
    if(assembledString == "AB" && totalString == true){   //ABORT THE FLIGHT
      if(SOUND_ON) tone(buzzerPin, 2000, 10 );
      Serial.println("ABORT TRIGGERED");
      abortFlight = true;
      assembledString = "";
      
      totalString = false;
    }  
    if(assembledString == "PNG" && totalString == true){  //PING THE ROCKET
      if(SOUND_ON) tone(buzzerPin, 3000, 10 );
      Serial.println("PNG");
      assembledString = "";
      totalString = false;
      
      XBee.println("PNG");
    } 
    if(assembledString == "LNCH" && totalString == true){ //HUMAN CHECK BEFORE AWAIT LIFTOFF
      if(SOUND_ON) tone(buzzerPin, 1000, 400 );
      Serial.println("LNCH");
      assembledString = "";
      totalString = false;
      
      XBee.println("CLEARED FOR LAUNCH");
      clearedForLaunch = true;
      
    }
    
    if(assembledString == "RCYC" && totalString == true){  //RECYCLE THE FLIGHT, RETURN TO INITIALIZING
      //if(SOUND_ON) tone(buzzerPin, 1000, 400 );
      Serial.println("RCYC");
      assembledString = "";
      totalString = false;
      
      bmppropogatorCompletion = false;        //RESET GLOBAL VARS AS NECESSARY 
      bnoCalibrationCompletion = false;
      gravityMeasurementCompletion = false;
      
      abortFlight = false; 
      apogeeFirstTime = true;
      
      maxbmpAltitude = 0;
      altitudePropogator = 0;
      altitudePropogatorCounter = 0;
      altitudeOffset = 0;
      adjustedAltitude = 0;
      
      gravityPropogator = 0;
      gravityPropogatorCounter = 0;
      measuredGravity = 9.79;
      
      timerCycleCounter = 0;
      
      dataString = "";
      assembledString = "";
      totalString = false;
      
      firstLoop = true;

      clearedForLaunch = false;
      
      machineState = INITIALIZING;
    }
    if(assembledString == "X" && totalString == true){ 
      //if(SOUND_ON) tone(buzzerPin, 1000, 400 );
      Serial.println("X");
      assembledString = "";
      totalString = false;
    }




    //LNCHZ

  }
  
}


void printDataTemp(){

  Serial.print("Altitude: ");
  Serial.println(adjustedAltitude);
  Serial.print("Accel X: ");
  Serial.println(accelX);
  Serial.print("Accel Y: ");
  Serial.println(accelY);
  Serial.print("Accel Z: ");
  Serial.println(accelZ);
  Serial.print("Yaw: ");
  Serial.println(yawRaw);
  Serial.print("Pitch: ");
  Serial.println(pitchRaw);
  Serial.print("Roll: ");
  Serial.println(rollRaw);
  Serial.print("Altitude Offset: ");
  Serial.println(altitudeOffset);
  Serial.print("measuredGravity: ");
  Serial.println(measuredGravity);
  Serial.print("currentMagAccel: ");
  Serial.println(measuredGravity - sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ)));
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  
  
}


void updateData(uint32_t datatimer, int machineState){
  dataString = "";
  
  dataString += datatimer; //+ accelX_converted; //+ "," ;//+ accelX_converted;
  dataString += ",";
  dataString += accelX;
  dataString += ",";
  dataString += accelY;
  dataString += ",";
  dataString += accelZ;
  dataString += ",";
  dataString += yawRaw;
  dataString += ",";
  dataString += rollRaw;
  dataString += ",";
  dataString += pitchRaw;
  dataString += ",";
  dataString += bmpPressure;
  dataString += ",";
  dataString += bmpTemperature;
  dataString += ",";
  dataString += bmpAltitude;
  dataString += ",";
  dataString += machineState;
  dataString += ",";
  dataString += "X";
  
}

string assemble_string(string currentString, char newChar, string endString){

    Serial.println(currentString.find(endString));
    currentString += newChar;

    int position = currentString.find(endString);

    if (position != -1){
        totalString = true;
        endString = "";
        return currentString.substr(0, position);
    }

    return currentString;
}




void printXBee(){
  //XBee.println(dataString); //this rules EDIT: omg it does NOT rule
  XBee.println(dataString); //maybe change to just print
}




void printSDCard(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      }
}

void printEEPROM(){

  //writeStringToEEPROM(eepromAddressCounter, dataString);
  writeStringToEEPROM(eepromAddressCounter, dataString);
  if(SOUND_ON) tone(buzzerPin, 3000, 10);
  
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  eepromAddressCounter += len;
  //EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}
