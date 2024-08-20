



/*      _      _ _                    _           
       | |    (_) |                  (_)          
       | |     _| |__  _ __ __ _ _ __ _  ___  ___ 
       | |    | | '_ \| '__/ _` | '__| |/ _ \/ __|
       | |____| | |_) | | | (_| | |  | |  __/\__ \
       |______|_|_.__/|_|  \__,_|_|  |_|\___||__*/

       
                #include <Servo.h>
                #include <math.h>
                #include <Wire.h>
                //#include <SPI.h>   //may need to include this as the bmp280test sketch seems to have it, however we are using i2c not spi so i think its unnecessary 
                #include <Adafruit_Sensor.h>
                #include <Adafruit_BNO055.h>
                #include <utility/imumaths.h>
                #include <Adafruit_BMP280.h>
                #include <SoftwareSerial.h> //for xbee
                //#include <Adafruit_GPS.h>     //:3 surely


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

                //#define GPSSerial Serial1 //surely
                
                uint32_t timer = millis();

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
                  AWAITLIFTOFF,
                  POWEREDASCENT,
                  UNPOWEREDASCENT,
                  UNPOWEREDDESCENT,
                  TERMINALGUIDANCE,
                  LANDED,
                  INFLIGHTABORT
                };

                enum state machineState = ASSEMBLY;

                int millisecondUpdateRate = 20; //updates every 20 ms

                bool abortFlight = false; 

                double maxbmpAltitude = 0;


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
      
    if (!bmp.begin()) {
      Serial.println("BMP280 failure to start");   
    }
    else{
      bmpStarted = true;
      Serial.println("BMP280 Initialized Successfully");
      tone(buzzerPin, 1500, 200);
    }
    
    delay(100);

    if(!bno.begin()){
      Serial.print("BNO055 failure to start");
    }
    else{
      bnoStarted = true;
      Serial.println("BNO055 Initialized Successfully");
      tone(buzzerPin, 1600, 200);
    }

    delay(200);
  }
  
  XBee.begin(9600);

  delay(100);
  
  Serial.println("All Sensors Initialized Successfully");
  
}

void loop() {

  updateXBee();

  sensors_event_t orientationData, accelerometerData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  readIMU(&orientationData);
  
  readIMU(&accelerometerData);

  if(millis() - timer > millisecondUpdateRate){  //everything in here runs every millisecondUpdateRate milliseconds
    
    timer = millis();
  
    readBMPPressure();
    
  }
  //ALL SENSOR INTAKE SHOULD GO ABOVE THIS LINE -------------------------------------

  if(abortFlight) machineState = INFLIGHTABORT;
  
  switch(machineState){
    case ASSEMBLY: assembly();
            break;
    case INITIALIZING: initializing();         
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

}



void assembly(){
  
}

void initializing(){
  
  uint8_t system, gyro, accel, mag;   //taking the calibration state of the bno055
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  if(system == 3){                    //full system calibration happens at 3
    Serial.println("BNO calibrated");
    machineState = AWAITLIFTOFF;
  }
  
}

void awaitLiftoff(){
  
}

void poweredAscent(){
  
}

void unpoweredAscent(){


  if(apogeeDetected()) machineState = UNPOWEREDDESCENT; 
}

void unpoweredDescent(){
  
}

void terminalGuidance(){
  
}

void landed(){
  
}

void inFlightAbort(){
  
}





bool apogeeDetected(){
  bool apogeeDetected = false;
  if(bmpAltitude > maxbmpAltitude) maxbmpAltitude = bmpAltitude;
  if(bmpAltitude < maxbmpAltitude - 1){ //if it is 1 meter below its max, thats hopefully apogee, can be dialed in later but this is safe for now
    apogeeDetected = true;
  }
  return apogeeDetected;
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

if (XBee.available()) {

    // Read next character in XBee buffer
    char abortSignal = XBee.read();

    // Treat "1" as the abort signal
    if (abortSignal == "1"){
      
      // Output abort state to serial monitor
      Serial.println("ABORT TRIGGERED");
      abortFlight = true;
    
    }
  }
  
}
