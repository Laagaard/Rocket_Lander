/*
DART Ground Station Software
*/

#include <SoftwareSerial.h>

SoftwareSerial XBee(2,3); // RX pin, TX pin

// Establish pin number for abort pin
int abort_pin = 8;

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

  // Wait 250 ms
  delay(250);
}