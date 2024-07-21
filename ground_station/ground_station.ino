/*
DART Ground Station Software
*/

#include <SoftwareSerial.h> // for XBee communication
#include <LiquidCrystal.h> // for LCD display

SoftwareSerial XBee(NULL,3); // RX pin, TX pin

// Establish pin number for abort pin
int abort_pin = 4;

// Establish pin number for initiation pin
int init_pin = 2;

// Establish pin number for launch pin
int launch_pin = 6;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

void setup()
{
  // Open serial communication
  Serial.begin(9600);

  // Open XBee communication
  XBee.begin(9600);

  // Set abort pin as input
  pinMode(abort_pin, INPUT);

  // Set built-in LED as output
  pinMode(LED_BUILTIN, OUTPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  lcd.print("Mode: STANDBY");
}

void initialize()
{
  lcd.print("Mode: INITIALIZATION");
}

void loop()
{
  // Read state of initialization pin
  int init_state = digitalRead(init_pin);

  // Read state of launch pin
  int launch_state = digitalRead(launch_pin);

  // Read state of abort pin
  int abort_state = digitalRead(abort_pin);

  // Treat 1 (pin HIGH) as abort signal
  if (abort_state){

    // Send "1" over XBee communication
    XBee.print(1);

    // Clear LCD
    lcd.clear();

    // Print "ABORT" on LCD
    lcd.print("Mode: ABORT");

    Serial.println("Abort State");

  // Treat all else as nominal
  } else if (launch_state) {

    // Clear LCD
    lcd.clear();

    // Print "Mode: LAUNCH" on LCD
    lcd.print("Mode: LAUNCH");

    Serial.println("Launch State");

  } else if (init_state) {

    // Send "0" over XBee communication
    XBee.print(0);
    
    // Clear LCD
    lcd.clear();

    // Enter initialization mode
    lcd.print("Mode: INIT");

    Serial.println("Init State");

    // initialize();
  } else {

    // Clear LCD
    lcd.clear();

    // Print "Mode: STANDBY"
    lcd.print("Mode: STANDBY");
  }

  // Wait 250 ms
  delay(250);
}