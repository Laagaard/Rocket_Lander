
import processing.serial.*;

Serial myPort;

byte[] lastUsableBuffer = new byte[227];

boolean serialPortDisabled = false;

int serialClearCounter = 0;

PFont f;

boolean pyroSafe = false;

PImage groundstation;

boolean firstTrigger = true;



void setup() 
{
  //delay(1000);
  size(1280, 720);
  f = createFont("SpaceMono-Regular", 20);
  noStroke();
  groundstation = loadImage("groundstationim.PNG");
  
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  //delay(1000);
}

void draw()
{
    background(255);
    
    imageMode(CORNER);
    image(groundstation,width - 450,0,450,450);
    
    
    
    
    
    
    int XChar = 88;

    byte[] inBuffer = new byte[2000];
    
    if ( myPort.available() > 0) {
      
      myPort.readBytesUntil(XChar, inBuffer);
      
      
      
    }
    if(inBuffer[6] != 0 || inBuffer[7] != 0 || inBuffer[8] != 0 || inBuffer[9] != 0){
      lastUsableBuffer = inBuffer;
    }
    
    
    
    String myString = new String(lastUsableBuffer);
    myString = myString.trim();
    //println(myString);
    String[] fcTokens = split(myString,',');
    boolean validTokens = true;
    try{
    //println("Continuity: " + fcTokens[0] + ", Triggered: " + fcTokens[1] + ", Pyro Safe: " + fcTokens[5] + ", Button: " + fcTokens[6]+ ", Load: " + fcTokens[2] + ", Time: " + fcTokens[3] + ", SD Card: " + fcTokens[4]);
    }
    catch(Exception ArrayIndexOutOfBoundsException){
      println("CHECK XBEE CONNECTION");
      validTokens = false;
    }
    
    if(validTokens){
      try{
        if(Double.parseDouble(fcTokens[5]) == 0){
          pyroSafe = false;
        }
        if(Double.parseDouble(fcTokens[5]) == 1){
          pyroSafe = true;
        }
      }
      catch(Exception NumberFormatException){
      }
    }
    fill(230,230,230);
    circle(width/2, height/2, 420);
    if(!pyroSafe){
      fill(255,0,0);
    }
    else{
      fill(255,200,200);
    }
    circle(width/2, height/2, 400);
    //textMode(SHAPE);
    fill(255);
    textSize(120);
    textAlign(CENTER,CENTER);
    text("FIRE",width/2,height/2);
    
    if(validTokens){
      fill(230);
      rect(0,0,350,150);
      textAlign(CORNER,CORNER);
      fill(0);
      textSize(40);
      text("Continuity",5,37.5);
      text("Firing",5,87.5);
      text("SD Card",5,137.5);
      text("S: Start SD card Printing",5,187.5);
      text("R: Stop SD card Printing",5,237.5);
      
      try{
        if(Double.parseDouble(fcTokens[0]) == 1){
          fill(0,250,0);
          rect(200,0,150,50);
        }
        else{
          fill(250,0,0);
          rect(200,0,150,50);
        }
        if(Double.parseDouble(fcTokens[1]) == 1){
          fill(0,250,0);
          rect(200,50,150,50);
        }
        else{
          fill(250,0,0);
          rect(200,50,150,50);
        }
        if(!pyroSafe){    //pyro is NOT safe
          fill(250,0,0);
          rect(width/2 - 100,(height/2)-300,200,50);
          fill(255,255,255);
          textSize(40);
          textAlign(CENTER,CENTER);
          text("UNSAFE",width/2,(height/2)-275);
        }
        else{
          firstTrigger = true;
          fill(0,250,0);
          rect(width/2 - 100,(height/2)-300,200,50);
          fill(255,255,255);
          textSize(40);
          textAlign(CENTER,CENTER);
          text("SAFE",width/2,(height/2)-275);
        }
        
        if(Double.parseDouble(fcTokens[6]) == 1 && firstTrigger && !pyroSafe){
          myPort.write("X");
          firstTrigger = false;
        }
        
        if(Double.parseDouble(fcTokens[6]) == 1 && firstTrigger && !pyroSafe){
          myPort.write("X");
          firstTrigger = false;
        }
        
        if(Double.parseDouble(fcTokens[4]) == 1){  //sd card print
          fill(0,250,0);
          rect(200,100,150,50);
        }
        else{
          fill(250,0,0);
          rect(200,100,150,50);
        }
        fill(0);
        
        textAlign(CORNER,CORNER);
        textSize(40);
        text("Load: " + fcTokens[2] + "g" ,5, 287.5);
        
        text("Time: " + (Double.parseDouble(fcTokens[3])/1000), 5, 337.5);
        
      
      }
      catch(Exception NumberFormatException){
        println("uh oh");
      }
    }
    
  if(serialClearCounter == 5){
    try{
      myPort.clear();
    }
    catch(Exception NullPointerException){
    }
  serialClearCounter = 0;
  }
  serialClearCounter++;

}

void mousePressed(){
  if(sqrt((mouseX-(width/2))*(mouseX-(width/2)) + (mouseY-(height/2))*(mouseY-(height/2)))  < 200 && !pyroSafe){
    myPort.write("X");
  }
}


void keyPressed(){
  if(key == 's'){
    myPort.write("S");
  }
  if(key == 'r'){
    myPort.write("R");
  }
  if(key == 'd'){
    myPort.write("D");
  }
  if(key == 'a'){
    myPort.write("A");
  }
    
}
