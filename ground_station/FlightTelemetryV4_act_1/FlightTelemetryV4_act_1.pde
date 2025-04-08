import processing.serial.*;

Serial myPort;

PFont f;

PFont menuHeader;

PShape model2;

PShape CDRmodel;

PImage dartLogo;

PImage backgroundImage;

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

int arrayLength = 200;

int graphsArrayCtr = 0;

float[] fcTimes = new float[arrayLength];

float[] runningAltitudes = new float[arrayLength];

float[] runningQuatW = new float[arrayLength];
float[] runningQuatX = new float[arrayLength];
float[] runningQuatY = new float[arrayLength];
float[] runningQuatZ = new float[arrayLength];

float[] runningEulX = new float[arrayLength];
float[] runningEulY = new float[arrayLength];
float[] runningEulZ = new float[arrayLength];

void setup() {
  size(1920, 1061, P3D);
  noStroke();
  dartLogo = loadImage("DART Logo (No Background).png");  
  backgroundImage = loadImage("GROUND_STATION_V1_2.png");
  f = createFont("SpaceMono-Regular", 27);
  menuHeader = createFont("SpaceMono-Regular", 120);
  textFont(f);
  //PFont.list(); //use this to look for fonts
  model2 = loadShape("fixedFullRocketAssembly2.obj");
  CDRmodel = loadShape("CDRmodelY90.obj");
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
  
  //rect(100,100, 300, 300);
  background(255); //should make conditional based on whether or not serial port is available so that screen isn't fully blank when no serial data
  imageMode(CENTER);
  image(backgroundImage,width/2 ,height/2,width,height);
  textFont(menuHeader);
  textAlign(CENTER, CENTER);
  //text("Ground Software", width/2, height/2 -275);
  textFont(f);
  fill(170);
  //text("Version 2.0", width/2, height/2 -200);      
  imageMode(CENTER);
  //image(dartLogo,width/2 ,height/2-200,500,500);
  textAlign(LEFT);
  pushMatrix();
  translate(width / 2, height / 2, -1000);  // Move back on the Z-axis to position it behind
  fill(230);  // Set the color to gray
  //box(600);   // Draw a gray box
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
     
     
  
     //fill(205,208,217);
     fill(94,96,107);
     textAlign(LEFT,BOTTOM);
     text(fcTime/1000.0 + "(sec)",width/1.343,height/13);
     String mState = "";
     switch(machineState){
       case 0: mState = "Assembly";
       break;
       case 1: mState = "Initialization";
       break;
       case 2: mState = "Await GFL";
       break;
       case 3: mState = "Await Liftoff";
       break;
       case 4: mState = "Pow. Ascent";
       break;
       case 5: mState = "Unpow. Ascent";
       break;
       case 6: mState = "Unpow. Descent";
       break;
       case 7: mState = "Term. Guidance";
       break;
       case 8: mState = "Landed";
       break;
       case 9: mState = "Abort";
       break;
       default: mState = "ERROR";
       break;
     }
     text(mState,width/1.124,height/1.8);
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     fill(205,208,217);
     text(lastString, width/10.3, height/1.007);
     text(mouseX, width/2, height/2.1);
     text(mouseY, width/2, height/1.9);
     
     fill(139,237,135); //geen
     fill(237,139,135); //red
     rectMode(CORNERS);
     
     if(descentMotorContinuity == 1){fill(139,237,135);}//geen
     else{fill(237,139,135);}//red
     rect(1700,311,1828,345);  //descent cont
     
     if(descentMotorPyroState == 1){fill(139,237,135);}//geen
     else{fill(237,139,135);}//red
     rect(1834,311,1920,345);  //descent gate
     
     if(abortContinuity == 1){fill(139,237,135);}//geen
     else{fill(237,139,135);}//red
     rect(1700,205,1828,239);  //abort charge cont
     
     if(abortPyroState == 1){fill(139,237,135);}//geen
     else{fill(237,139,135);}//red
     rect(1834,205,1920,239);  //abort charge gate
     
     
     
     
     runningAltitudes[graphsArrayCtr] = altitude;
     runningQuatW[graphsArrayCtr] = quatW;
     runningQuatX[graphsArrayCtr] = quatX;
     runningQuatY[graphsArrayCtr] = quatY;
     runningQuatZ[graphsArrayCtr] = quatZ;
  
     fcTimes[graphsArrayCtr] = fcTime;
  
     graphsArrayCtr = graphsArrayCtr + 1;
  
     if(graphsArrayCtr == arrayLength) graphsArrayCtr = 0;
  
     drawGraph(fcTimes, runningQuatW, 30, 536, 492, 117, 152, 135, 237); //top left
     
     drawGraph(fcTimes, runningQuatX, 30, 536,  974, 599, 152, 135, 237); // bottom left
     
     drawGraph(fcTimes, runningQuatY, 606, 1112, 974, 599, 152, 135, 237); //bottom right
     
     drawGraph(fcTimes, runningQuatZ, 606, 1112, 492, 117, 152, 135, 237); //top right
     
     
     
     //text("Time(ms):"+fcTime,30,50);
     //text("Linear Acceleration X:"+linaccX,30,150);
     //text("Linear Acceleration Y:"+linaccY,30,200);
     //text("Linear Acceleration Z:"+linaccZ,30,250);
     //text("quatw:"+quatW,30,350);
     //text("quatx:"+quatX,30,400);
     //text("quaty:"+quatY,30,450);
     //text("quatz:"+quatZ,30,500);
     //text("Altitude(m):"+altitude,30,600);
     //text("Abort Continuity:"+abortContinuity,30,700);
     //text("Abort Pyro State:"+abortPyroState,30,750);
     //text("Descent Motor Continuity:"+descentMotorContinuity,30,800);
     //text("Descent Motor Pyro State:"+descentMotorPyroState,30,850);
     //text("Com Code:"+comCode,400,150);
     //text("Error Code:"+errorCode,400,200);
     //text("Machine State:"+machineState,400,250);
     //text("Switch State:"+switchState,400,350);
     //text("Abort Button State:"+abortButtonState,400,400);
     //text("Launch Button State:"+launchButtonState,400,450);
     //text("Ground Pryo State:"+groundPyroState,400,500);
     
     if(comCode == 4){
       //text("ATTEMPT TO CLEAR MEMORY, Confirm/deny (c/d)",0,600);
     }
     if(comCode == 5){
       //text("MEMORY CLEARED",0,600);
     }
     
     if(comCode == 6){
       //text("PARACHUTES DEPLOYED",0,600);
     }
     //text("Altitude(m):"+altitude,30,600);
     text("Roll: "+degrees(eulerAngles[0]),400,600);
     text("Pitch: "+degrees(eulerAngles[1]),400,650);
     text("Yaw: "+degrees(eulerAngles[2]),400,700);
     
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
       //drawRocket(tokens);
       textAlign(LEFT);
      
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











void drawGraph(float[] fcTimes, float[] inData, float xboundary_left, float xboundary_right, float yboundary_lower, float yboundary_upper, int r, int g, int b){
  
  stroke(r, g, b);
  strokeWeight(4);
  

  
  //for(int i = 0; i < arrayLength; i++){
  //  print(inDataArray[i] + "  ");
  //}
  //println();
  //for(int i = 0; i < arrayLength; i++){
  //  print(inTimeArray[i] + "  ");
  //}
  //println();
  
  float max_x = max(fcTimes);
  float min_x = min(fcTimes);
  
  float max_y = max(inData);
  float min_y = min(inData);
  
  int currentIndex = graphsArrayCtr;
  
  for (int i = 0; i < arrayLength - 1; i++) {
    int index1 = (currentIndex + i) % arrayLength;
    int index2 = (currentIndex + i + 1) % arrayLength;
  
    line(
      map(fcTimes[index1], min_x, max_x, xboundary_left, xboundary_right),
      map(inData[index1], min_y, max_y, yboundary_lower, yboundary_upper),
      map(fcTimes[index2], min_x, max_x, xboundary_left, xboundary_right),
      map(inData[index2], min_y, max_y, yboundary_lower, yboundary_upper)
    );
  }
  
  noStroke();
  
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
     
     rotateX(-HALF_PI);  // Try flipping this if needed
     rotateY(PI); 

     //rotateY(HALF_PI);   
     //rotateZ(PI);
     //rotateY(PI);
     //rotateZ(PI);
     //rotateX(-HALF_PI); 
     //rotateX(PI);
     //rotateZ(PI);   

     

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
      scale(1.4);
      shape(CDRmodel);
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
    //myPort.write('o');  //release legs
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
