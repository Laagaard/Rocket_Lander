#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

// Create an instance of the Hx711 class
HX711 scale;

float offset = 0;             // Offset value from the scale without any load
float calibrationFactor = 205; // Calibration factor determined by the experiment (205 worked well)
const float g = 9.81;         // Acceleration due to gravity (m/s^2)
unsigned long timer = 0;
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
}

void loop() {
  // Read and print the real-time load (thrust) value in Newtons
  float load_grams = scale.get_units(); // Get the load in grams
  float load_newtons = (load_grams / 1000.0) * g; // Convert to Newtons
  timer = millis();
  Serial.print(timer);
  Serial.print(",");
  Serial.println(load_newtons);
}