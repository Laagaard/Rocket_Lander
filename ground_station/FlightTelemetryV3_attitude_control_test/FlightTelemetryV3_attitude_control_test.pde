import processing.serial.*;

Serial myPort;

PFont f;

PFont menuHeader;

PShape model2;

PImage dartLogo;

byte[] lastUsableBuffer = new byte[227];

boolean serialPortEnabled = false;

boolean firstTrigger = true;

boolean firstTestTrigger = true;

boolean IGNITETHATBITCH = false;

int serialClearCounter = 0;

FloatList data = new FloatList();

PrintWriter output;

String fileName = "Data From ";

String lastString = "";

void setup() {
  size(1920, 1061, P3D);
  noStroke();
  dartLogo = loadImage("DART Logo (No Background).png");  
  f = createFont("SpaceMono-Regular", 20);
  menuHeader = createFont("SpaceMono-Regular", 120);
  textFont(f);
  //PFont.list(); //use this to look for fonts
  model2 = loadShape("fixedFullRocketAssembly2.obj");
  surface.setResizable(true);
  
  fileName += str(month()) + "-" + str(day()) + "-" + str(hour()) + "-" + str(minute()) + "-" + str(second());
  fileName += ".txt";
  output = createWriter(fileName); 
}

void draw() {
  
if(!serialPortEnabled){
  try{
    String portName = Serial.list()[0];
    myPort = new Serial(this, portName, 9600);
    serialPortEnabled = true;
  }
  catch(Exception ArrayIndexOutOfBoundsException){
    serialPortEnabled = false;
    
  }
  background(255);
  fill(100);
  textFont(menuHeader);
  textAlign(CENTER, CENTER);
  text("Ground Software", width/2, height/2 -275);
  textFont(f);
  fill(170);
  text("Version 2.0", width/2, height/2 -200);      
  imageMode(CENTER);
  image(dartLogo,width/2 ,height/2-400,300,300);
}
else{
  rect(100,100, 300, 300);
  background(255); //should make conditional based on whether or not serial port is available so that screen isn't fully blank when no serial data
  pushMatrix();
  translate(width / 2, height / 2, -1000);  // Move back on the Z-axis to position it behind
  fill(230);  // Set the color to gray
  box(600);   // Draw a gray box
  popMatrix();
  
  boolean portAvailable = false;
  String myString;
  int XChar = 88;
  byte[] inBuffer = new byte[227];
  
  if(myPort.available() > 0) { //Gets rid of flilckering when no new character string, but lets the program continue to redraw the window
    myPort.readBytesUntil(XChar, inBuffer);
    lastUsableBuffer = inBuffer;
    portAvailable = true;
    myString = trim(new String(inBuffer));
  }
  else{
    myString = trim(new String(lastUsableBuffer));
    try{
      myPort.clear();
    }
    catch(Exception NullPointerException){
    }
   }
   if(myString != lastString && myString.length() > 2){
     lastString = myString;
     println(lastString);
   }
   String[] tokens = lastString.split(",");
   //if(myString != ""){
   //  tokens = myString.split(",");
   //}
   //else{
   //}
   try{
     int fcTime = Integer.parseInt(tokens[0]);
     float linaccX = parseFloat(tokens[1]);
     float linaccY = parseFloat(tokens[2]);
     float linaccZ = parseFloat(tokens[3]);
     float quatW = parseFloat(tokens[4]);
     float quatX = parseFloat(tokens[5]);
     float quatY = parseFloat(tokens[6]);
     float quatZ = parseFloat(tokens[7]);
     float altitude = parseFloat(tokens[8]);
     int abortContinuity = Integer.parseInt(tokens[9]);
     int abortPyroState = Integer.parseInt(tokens[10]);
     int descentMotorContinuity = Integer.parseInt(tokens[11]);
     int descentMotorPyroState = Integer.parseInt(tokens[12]);
     int comCode = Integer.parseInt(tokens[13]);
     int errorCode = Integer.parseInt(tokens[14]);
     int machineState = Integer.parseInt(tokens[15]);
     int switchState = Integer.parseInt(tokens[16]);
     int abortButtonState = Integer.parseInt(tokens[17]);
     int launchButtonState = Integer.parseInt(tokens[18]);
     int groundPyroState = Integer.parseInt(tokens[19]);
     
     float[] eulerAngles = quaternionToEuler(quatW, quatX, quatY, quatZ);
     
     
  
     fill(0);
     text("Time(ms):"+fcTime,0,50);
     text("Linear Acceleration X:"+linaccX,0,75);
     text("Linear Acceleration Y:"+linaccY,0,100);
     text("Linear Acceleration Z:"+linaccZ,0,125);
     text("quatw:"+quatW,0,150);
     text("quatx:"+quatX,0,175);
     text("quaty:"+quatY,0,200);
     text("quatz:"+quatZ,0,225);
     text("Altitude(m):"+altitude,0,300);
     text("Abort Continuity:"+abortContinuity,0,325);
     text("Abort Pyro State:"+abortPyroState,0,350);
     text("Descent Motor Continuity:"+descentMotorContinuity,0,375);
     text("Descent Motor Pyro State:"+descentMotorPyroState,0,400);
     text("Com Code:"+comCode,0,425);
     text("Error Code:"+errorCode,0,450);
     text("Machine State:"+machineState,0,475);
     text("Switch State:"+switchState,0,500);
     text("Abort Button State:"+abortButtonState,0,525);
     text("Launch Button State:"+launchButtonState,0,550);
     text("Ground Pryo State:"+groundPyroState,0,575);
     
     if(comCode == 4){
       text("ATTEMPT TO CLEAR MEMORY, Confirm/deny (c/d)",0,600);
     }
     if(comCode == 5){
       text("MEMORY CLEARED",0,600);
     }
     
     if(comCode == 6){
       text("PARACHUTES DEPLOYED",0,600);
     }
     
     text("Roll, Euler X: "+degrees(eulerAngles[0]),0,625);
     text("Pitch, Euler Y: "+degrees(eulerAngles[1]),0,650);
     text("Yaw, Euler Z: "+degrees(eulerAngles[2]),0,675);
     
     if(switchState == 0){    //pyro is NOT safe
          output.println(lastString);  //IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT 
        }
        else{
          output.flush();        //IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT IMPLEMENT 
          firstTrigger = true;
          firstTestTrigger = true;
        }
     
     if(launchButtonState == 1 && firstTrigger && switchState == 0){
          myPort.write("L");
          println("CLEARED FOR LAUNCH");
          firstTrigger = false;
     }
     if(firstTestTrigger && switchState == 0 && IGNITETHATBITCH){
          myPort.write('Q');  //LAUNCH DAT HOE!! in a testing manner of course
          firstTestTrigger = false;
     }
     else{
       IGNITETHATBITCH = false;
     }
     
     fill(230);
     if(fcTime > 100){ //if data is real and good
       drawRocket(tokens);
      
      if(serialClearCounter == 5){
        try{
          //myPort.clear();
        }
        catch(Exception NullPointerException){
        }
        serialClearCounter = 0;
      }
      serialClearCounter++;
     }
     else{
       //drawRocket(tokens);
       fill(0);
       text("OVAH!", 50, 100);
     }
   }
   catch(Exception NumberFormatException){
   }
   
}

}


void drawRocket(String[] tokens){
  fill(0);
     stroke(0, 0, 0);
     textAlign(LEFT);
     
     //drawData(tokens);
     //drawData(myString);
      
     pushMatrix();
     resetMatrix(); // Reset transformations to draw in 2D
     popMatrix(); 
     lights();
   
     pushMatrix();
  
     translate(width / 2, height/2, 0);  //THESE TWO ARE FOR THE ORIGINAL MODEL
     //rotateZ(HALF_PI);
     rotateY(HALF_PI);    //GOOD
     rotateX(-HALF_PI);  //GOOD
     //rotateY(HALF_PI);
     //rotateZ(HALF_PI+PI);
     //rotate(PI);
     rotateZ(PI);    //GOOD
     //rotateY(PI);
     

     float w2 = (float)parseFloat(tokens[4]);  //TBR: not sure if these have to be recast, but im assuming i did it for a reason
     float x2 = (float)parseFloat(tokens[5]);
     float y2 = (float)parseFloat(tokens[6]);
     float z2 = (float)parseFloat(tokens[7]);
      
     float[][] rotationMatrix2 = quatToMatrix(w2, x2, y2, z2);
  
     applyMatrix(rotationMatrix2[0][0], rotationMatrix2[0][1], rotationMatrix2[0][2], 0,
                 rotationMatrix2[1][0], rotationMatrix2[1][1], rotationMatrix2[1][2], 0,
                 rotationMatrix2[2][0], rotationMatrix2[2][1], rotationMatrix2[2][2], 0,
                 0, 0, 0, 1);
  
      fill(200);
      shapeMode(CENTER);
      translate(10,100);
      scale(0.4);
      shape(model2);
      popMatrix();
}


void drawData(String[] tokens){
  try{
  //String[] tokens = data.split(",");
  text("Time(ms):"+tokens[0],0,50);
  text("Linear Acceleration X:"+tokens[1],0,75);
  text("Linear Acceleration Y:"+tokens[2],0,100);
  text("Linear Acceleration Z:"+tokens[3],0,125);
  text("quatw:"+tokens[4],0,150);
  text("quatx:"+tokens[5],0,175);
  text("quaty:"+tokens[6],0,200);
  text("quatz:"+tokens[7],0,225);
  text("Latitude:"+tokens[8],0,250);
  text("Longitude:"+tokens[9],0,275);
  text("altitude(pa):"+tokens[10],0,300);
  text("Temperature(C):"+tokens[11],0,325);
  text("Altitude(m):"+tokens[12],0,350);
  text("Machine State:"+tokens[13],0,375);
  }
  catch(Exception ArrayIndexOutOfBounds){  //TBR: this sucks
  }
  
}

float[][] quatToMatrix(float w, float x, float y, float z) {
  float[][] m = new float[3][3];

  m[0][0] = 1 - 2 * (y * y + z * z);
  m[0][1] = 2 * (x * y - z * w);
  m[0][2] = 2 * (x * z + y * w);
  
  m[1][0] = 2 * (x * y + z * w);
  m[1][1] = 1 - 2 * (x * x + z * z);
  m[1][2] = 2 * (y * z - x * w);
  
  m[2][0] = 2 * (x * z - y * w);
  m[2][1] = 2 * (y * z + x * w);
  m[2][2] = 1 - 2 * (x * x + y * y);

  return m;
}

float[] quaternionToEuler(float x, float y, float z, float w){
  float[] euler = new float[3];
  
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  euler[0] = atan2(sinr_cosp, cosr_cosp);
  
  float sinp = 2 * (w * y - z * x);
  
  if(abs(sinp) >= 1)
    euler[1] = (sinp > 0 ? HALF_PI : -HALF_PI);
  else
    euler[1] = asin(sinp);
    
  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  euler[2] = atan2(siny_cosp, cosy_cosp);
  
  return euler;
}

void mousePressed() {
  // Check if Live Telemetry button is clicked
  
}

void keyPressed(){
  if(key == 'g'){
    myPort.write("G");  //clear gps error (big unlucky if we have to use this)
  }
  if(key == 'x'){
    myPort.write('C');  //memory clear attempt, will make the rocket ask for confirmation first
  }
  if(key == 'c'){
    myPort.write('c');  //confirm the memory clear, will also dump current memory to sd card before doing so
  }
  if(key == 'd'){
    myPort.write('d');  //deny the memory clear, reset all memory clear logic
  }
  if(key == 'o'){
    myPort.write('o');  //release legs
  }
  if(key == 'l'){
    myPort.write('l');  //lock legs
  }
  if(key == 't'){
    myPort.write('T');  //terminal guidance
  }
  if(key == 'r'){
    myPort.write('R');  //recycle
  }
  if(key == 'q'){
    IGNITETHATBITCH = true;
  }
  if(key == 'm'){
    myPort.write('m'); //transfer memory from flash to sd
  }
  
}
