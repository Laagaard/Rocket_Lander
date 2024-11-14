#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

// Create an instance of the Hx711 class
HX711 scale;

float offset = 0;             // Offset value from the scale without any load
float calibrationFactor = 205; // Calibration factor determined by the experiment (205 worked well)
const float g = 9.81;         // Acceleration due to gravity (m/s^2)
int timer = 0;
void setup() {
  Serial.begin(9600);

  // Initialize the scale with the proper pins
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare();

  // Determine and set the offset automatically (with no weight)
  Serial.println("Place no load on the scale. Calculating offset...");
  offset = scale.get_units(10); // Get the average value from 10 readings
  // Serial.print("offset");
  // Serial.println(offset);
  // scale.set_offset(offset);     // Set the offset value automatically

  // Set the calibration factor (from previous calibration experiments)
  scale.set_scale(calibrationFactor);

  Serial.println("Scale initialized. Offset and calibration factor set.");
}

void loop() {
  // Read and print the real-time load (thrust) value in Newtons
  float load_grams = scale.get_units();          // Get the load in grams
  float load_newtons = (load_grams / 1000.0) * g; // Convert to Newtons
  if(millis()-timer >20){
  //Serial.print("Thrust: ");
  Serial.println(load_newtons);
  //Serial.println(" N"); // Display thrust in Newtons
  timer = millis();
  }
  //delay(200); // Update every 200 milliseconds
}