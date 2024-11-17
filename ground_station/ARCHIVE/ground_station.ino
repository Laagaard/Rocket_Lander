/*
DART Ground Station Software
*/

#include <SoftwareSerial.h> // for XBee communication
#include <LiquidCrystal.h> // for LCD display

SoftwareSerial XBee(2,3); // RX pin, TX pin

// Establish pin number for launch pin
int launch_pin = 5;

// Establish pin number for abort pin
int abort_pin = 6;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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

  // Set up the LCD's number of columns and rows
  lcd.begin(16, 2);

  lcd.print("Mode: STANDBY");
}

void loop()
{
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

    // Print "Abort State" on serial monitor
    Serial.println("Abort State");

  // Treat 1 (pin HIGH) as launch signal
  } else if (launch_state) {

    // Clear LCD
    lcd.clear();

    // Print "Mode: LAUNCH" on LCD
    lcd.print("Mode: LAUNCH");

    // Print "Launch State" on serial monitor
    Serial.println("Launch State");

    // Treat all else as nominal (STANDBY mode)
  } else {

    // Clear LCD
    lcd.clear();

    // Print "Mode: STANDBY"
    lcd.print("Mode: STANDBY");
  }

  // Wait 250 ms
  delay(250);
}