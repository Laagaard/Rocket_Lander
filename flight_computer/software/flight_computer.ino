/*
DART Flight Computer Software
*/

#include <SoftwareSerial.h>

SoftwareSerial XBee(2,3); // RX pin, TX pin

void setup()
{
  // Open serial monitor communication
  Serial.begin(9600);
  // Open XBee communication
  XBee.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // Check for unread data in XBee buffer
  if (XBee.available()) {

    // Read next character in XBee buffer
    char abortSignal = XBee.read();

    // Treat "1" as the abort signal
    if (abortSignal == "1"){
      // Turn on the onboard LED
      digitalWrite(LED_BUILTIN, HIGH);
      // Output abort state to serial monitor
      Serial.println(abortSignal);
    // Treat all else as nominal
    } else {
      // Turn off the onboard LED
      digitalWrite(LED_BUILTIN, LOW);
      // Output abort state to serial monitor
      Serial.println(abortSignal);
    }
  }
}