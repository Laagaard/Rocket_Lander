
#include <Wire.h>
#include <SoftwareSerial.h> //for xbee


#define pyroSafeSwitch 3
#define buttonPin 4
#define launchButtonPin 5 //TBR: needs to be added to output


uint32_t timer = millis();
uint32_t tvcTimer = millis();

SoftwareSerial XBee(21,20); 

char xbeeChar;

String assembly = "";

bool assemblyReady = false;

bool pyroSafe = true;

int switchState = 0;

int lastSwitchState = 0; 

int launchButtonState = 0;

int buttonState = 0;

bool buttonPressed = false;

bool launchButtonPressed = false;

bool pyroContinuity = false;

bool lastButtonPressedState = buttonPressed;

bool sendTVCSignal = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(pyroSafeSwitch, INPUT_PULLUP);    //THE SWITCH MUST BE CONNECTED FROM GROUND TO INPUT
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(launchButtonPin, INPUT_PULLUP);

  delay(10);
  
  Serial.begin(9600);
  
  delay(10);
  
  XBee.begin(9600);
  
  delay(10);
}

void loop() {

  if(sendTVCSignal && millis() - tvcTimer > 100){
    XBee.print('R');
    tvcTimer = millis();
  }
  
  if(millis() - timer > 100){
    timer = millis();

    switchState = digitalRead(pyroSafeSwitch);

    buttonState = digitalRead(buttonPin);

    launchButtonState = digitalRead(launchButtonPin);
    
    //Serial.println(switchState);

    if(switchState){
      pyroSafe = true;
    }else{
      pyroSafe = false;
    }

    if(!buttonState){
      buttonPressed = true;
    } 
    else{
      buttonPressed = false;
    }

    if(!launchButtonState){
      launchButtonPressed = true;
    }
    else{
      launchButtonPressed = false;
    }

  }

  

  if(buttonPressed != lastButtonPressedState && buttonPressed){  //do tvc stuff
    sendTVCSignal = true;
    lastButtonPressedState = buttonPressed;
  }
  if(buttonPressed != lastButtonPressedState && !buttonPressed){
    sendTVCSignal = false;
    XBee.print('T');
    XBee.print('T');
    XBee.print('T');
    lastButtonPressedState = buttonPressed;
  }
  
  updateXBee();
  serialUpdate();
  //writeXBee();
//  if(millis() - timer > 10){
//    Serial.println(assembly);
////    Serial.println("bruh");
//    assembly = "";
//  }
}



void updateXBee(){

  if (XBee.available()) {
      xbeeChar = XBee.read();
      if(xbeeChar == 'X'){
        if(pyroSafe){
          assembly += '1';    
          assembly += ',';      
        }
        else{
          assembly += '0';  
          assembly += ',';
        }
        if(buttonPressed){
          assembly += '1';    
          assembly += ',';  
        }
        else{
          assembly += '0';  
          assembly += ',';
        }
        if(launchButtonPressed){
          assembly += '1';    
          assembly += ',';  
        }
        else{
          assembly += '0';  
          assembly += ',';
        }
        //all stuff that is non xbee goes here
        assembly += 'X';
          Serial.println(assembly);
        assembly = "";
      }
      else{
        assembly += xbeeChar;
      }

  }

}

void serialUpdate(){
  if(Serial.available() > 0){
    byte inByte = Serial.read();
    if(inByte == 88){
      //Serial.println("FIREDX");
      writeXBeeFire();
    }
    if(inByte == 68){ //D
      XBee.print('D');
    }
    if(inByte == 65){ //A
      XBee.print('A');
    }
    if(inByte == 83){ //S
      XBee.print('S');
    }
    if(inByte == 82){ //R
      XBee.print('R');
    }
    // if(inByte == )
  }
}

void writeXBeeFire(){
  XBee.print('F');
}
