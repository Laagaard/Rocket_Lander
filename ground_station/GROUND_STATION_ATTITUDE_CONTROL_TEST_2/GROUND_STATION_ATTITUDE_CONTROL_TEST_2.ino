
#include <Wire.h>
#include <SoftwareSerial.h> //for xbee


#define pyroSafeSwitch 3
#define buttonPin 4
#define launchButtonPin 5 //TBR: needs to be added to output

#define pyro2GatePin 38
#define pyro2ContinuityPin 39

int continuityState2 = 0;

int pyro2Gate = LOW;

uint32_t timer = millis();

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

bool clearedForLaunch = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(pyroSafeSwitch, INPUT_PULLUP);    //THE SWITCH MUST BE CONNECTED FROM GROUND TO INPUT
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(launchButtonPin, INPUT_PULLUP);
  pinMode(pyro2ContinuityPin, INPUT);
  pinMode(pyro2GatePin, OUTPUT);
  
  delay(10);
  
  Serial.begin(9600);
  
  delay(10);
  
  XBee.begin(9600);
  
  delay(10);
}

void loop() {
  
  if(millis() - timer > 100){
    timer = millis();

    switchState = digitalRead(pyroSafeSwitch);

    buttonState = digitalRead(buttonPin);

    launchButtonState = digitalRead(launchButtonPin);

    continuityState2 = digitalRead(pyro2ContinuityPin);
    
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
    if(continuityState2){
      pyroContinuity = true;
    }
    else{
      pyroContinuity = false;
    }

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
        if(pyroContinuity){
          assembly += '1';    
          assembly += ','; 
        }
        else{
          assembly += '0';  
          assembly += ',';
        }
        if(clearedForLaunch){
          assembly += 'L';    
          assembly += ','; 
        }
        else{
          assembly += 'N';  
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
    if(inByte == 88){ //
      //Serial.println("FIREDX");
      //writeXBeeFire();
    }
    if(inByte == 68){ //D
      XBee.print('D');
    }
    if(inByte == 65){ //A abort
      XBee.print('A');
      XBee.print('A');
      XBee.print('A');
    }
    if(inByte == 83){ //S
      XBee.print('S');
    }
    if(inByte == 82){ //R
      XBee.print('R');
    }
    if(inByte == 76){ //L
      XBee.print('L');
      XBee.print('L');
      XBee.print('L');
      clearedForLaunch = true;
    }
    if(inByte == 71){ //G
      XBee.print('G');
      XBee.print('G');
      XBee.print('G');
    }
    if(inByte == 67){ //C
      XBee.print('C');  //memory clear attempt
      XBee.print('C');
      XBee.print('C');
    }
    if(inByte == 99){ //c
      XBee.print('c');
      XBee.print('c');
      XBee.print('c');
    }
    if(inByte == 100){ //d
      XBee.print('d');
      XBee.print('d');
      XBee.print('d');
    }
    if(inByte == 111){ //o
      XBee.print('o');
    }
    if(inByte == 108){ //l
      XBee.print('l');
    }
    if(inByte == 84){ //T
      XBee.print('T');
    }
    if(inByte == 109){ //m
      XBee.print('m');
    }
  }
}

void writeXBeeFire(){
  XBee.print('F');
}
