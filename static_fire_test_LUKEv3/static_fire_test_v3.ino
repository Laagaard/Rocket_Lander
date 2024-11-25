#include <Servo.h>

#include "HX711.h"

#include "math.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int RELAY_PIN = 5;
const int SERVO_PIN_BLUE = 9;
const int SERVO_PIN_WHITE = 10;

// Create an instance of the Hx711 class
HX711 scale;

float offset = 0.0;               // Offset value from the scale without any load
float calibrationFactor = 205.0;  // Calibration factor determined by the experiment (205 worked well)
const float g = 9.81;             // Acceleration due to gravity (m/s^2)
unsigned long timer = 0.0;
const double burnoutTime = 4.5;            // burn duration of G25W (s)
const double ignitionCommandTime = 10.0;   // time to command ignition
const double relayIgnitionDuration = 5.0;  // time to keep relay open (s)

Servo servoBlue;
Servo servoWhite;

void setup() {
  Serial.begin(9600);

  // Initialize the scale with the proper pins
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // "Zero" the load cell
  scale.tare();

  // Set the calibration factor (from previous calibration experiments)
  scale.set_scale(calibrationFactor);

  Serial.println("Scale initialized. Calibration factor set.");
  Serial.println("----------------- NEW DATA BEGIN -----------------");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  delay(1000);

  servoBlue.attach(SERVO_PIN_BLUE);
  servoWhite.attach(SERVO_PIN_WHITE);

  servoBlue.write(90);
  servoWhite.write(90);
}

void loop() {
  // START OF LOOP AND STATE MACHINE CODE
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(RELAY_PIN, LOW);
  servoBlue.write(90);
  servoWhite.write(90);
  timer = millis();
  while (timer / 1000.0 < ignitionCommandTime + relayIgnitionDuration + burnoutTime) {
    timer = millis();
    if (timer / 1000.0 < ignitionCommandTime) {
      // output normal time and load before ignition

      timer = millis();
      float load_grams = scale.get_units();  // Get the load in grams
      float load_newtons = (load_grams)*g;   // Convert to Newtons
      digitalWrite(RELAY_PIN, LOW);
      Serial.print(timer / 1000.0, 4);
      Serial.print(",");
      Serial.println(load_newtons);

      servoBlue.write(90);
      servoWhite.write(90);
    }
    if ((timer / 1000.0 >= ignitionCommandTime) & (timer / 1000.0 <= ignitionCommandTime + relayIgnitionDuration)) {
      // command motor ignition
      // output time, load, and Ignition message

      timer = millis();
      float load_grams = scale.get_units();            // Get the load in grams
      float load_newtons = (load_grams / 1000.0) * g;  // Convert to Newtons
      Serial.print(timer / 1000.0, 4);
      Serial.print(",");
      Serial.print(load_newtons);
      Serial.print(",");
      Serial.println("IGNITION!");
      digitalWrite(RELAY_PIN, HIGH);

      servoBlue.write(90 + round(30.0 * (cos(0.25 * (2 * PI) * timer / 1000.0))));
      servoWhite.write(90 + round(30.0 * (cos(0.25 * (2 * PI) * (timer / 1000.0 + 1.0)))));
    }
    if (timer / 1000.0 > ignitionCommandTime + relayIgnitionDuration) {
      // output normal time and load after ignition

      timer = millis();
      float load_grams = scale.get_units();            // Get the load in grams
      float load_newtons = (load_grams / 1000.0) * g;  // Convert to Newtons
      digitalWrite(RELAY_PIN, LOW);
      Serial.print(timer / 1000.0, 4);
      Serial.print(",");
      Serial.println(load_newtons);

      servoBlue.write(90);
      servoWhite.write(90);
    }
  }
}