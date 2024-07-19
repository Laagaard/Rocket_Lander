/*
DART Ground Station Software
*/

#include <SoftwareSerial.h> // for XBee communication
#include <LiquidCrystal.h> // for LCD display

SoftwareSerial XBee(2,3); // RX pin, TX pin

// Establish pin number for abort pin
int abort_pin = 4;

// Establish pin number for initiation pin
int initiation_pin = 5;

// Establish pin number for launch pin
int launch_pin = 6;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

void  setup()
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

  // Print a message to the LCD.
  lcd.print("hello, world!");
}

void loop()
{
  // Read state of abort pin
  int abort_state = digitalRead(abort_pin);

  // Treat 1 (pin HIGH) as abort signal
  if (abort_state == 1){
    // Send "1" over XBee communication
    XBee.print(1);
    digitalWrite(LED_BUILTIN, HIGH);
  // Treat all else as nominal
  } else {
    // Send "0" over XBee communication
    XBee.print(0);
    digitalWrite(LED_BUILTIN, LOW);
  }

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);

  // print the number of seconds since reset:
  lcd.print(millis() / 1000);

  // Wait 250 ms
  delay(250);
}