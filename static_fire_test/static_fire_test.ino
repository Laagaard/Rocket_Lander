#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int RELAY_PIN = 1;
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 11;

// Create an instance of the Hx711 class
HX711 scale;

float offset = 0;               // Offset value from the scale without any load
float calibrationFactor = 205;  // Calibration factor determined by the experiment (205 worked well)
const float g = 9.81;           // Acceleration due to gravity (m/s^2)
unsigned long timer = 0;
const double burnoutTime = 4.5;           // burn duration of G25W (s)
const double ignitionCommandTime = 10.0;  // time to command ignition
const double relayIgnitionDuration = 5;   // time to keep relay open (s)
double ignitionDelay = 1;                 // time between ignition command and ignition detection
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
}

void loop() {
  // Read and print the real-time load (thrust) value in Newtons
  float load_grams = scale.get_units();            // Get the load in grams
  float load_newtons = (load_grams / 1000.0) * g;  // Convert to Newtons
  timer = millis();

  if ((timer / 1000.0 >= ignitionCommandTime) && (timer / 1000.0 <= ignitionCommandTime + relayIgnitionDuration)) {
    // command motor ignition
    digitalWrite(RELAY_PIN, HIGH);
    // output time, load, and Ignition message
    Serial.print(timer);
    Serial.print(",");
    Serial.print(load_newtons);
    Serial.print(",");
    Serial.println("IGNITION!");

  } else if (timer / 1000.0 < ignitionCommandTime) {
    // output normal time and load before ignition
    Serial.print(timer);
    Serial.print(",");
    Serial.println(load_newtons);
  } else if (timer / 1000.0 > ignitionCommandTime + relayIgnitionDuration) {
    // output normal time and load after ignition
    digitalWrite(RELAY_PIN, LOW);
    Serial.print(timer);
    Serial.print(",");
    Serial.println(load_newtons);
  } else {
    Serial.print(timer);
    Serial.print(",");
    Serial.print("Exception thrown.");
  }
}