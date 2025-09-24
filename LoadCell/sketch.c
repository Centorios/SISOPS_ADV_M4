#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;
float weight = 0.0;
float calibration_factor = 420.0;
HX711 loadCell;

void setup() {
  Serial.begin(115200);
  Serial.println("System starting...");

  loadCell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("Load cell initializing...");
  
  while (!loadCell.is_ready()) {
    Serial.println("Waiting for load cell...");
    delay(100);
  }
  
  loadCell.set_scale(calibration_factor);
  
  Serial.println("Remove all weight from the load cell and press any key to tare...");
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read();
  
  loadCell.tare(10);
  
  Serial.println("Load cell tared and ready!");
  Serial.print("Current calibration factor: ");
  Serial.println(calibration_factor);

  delay(1000);
}

void loop() {
    if (loadCell.is_ready())
    {
        weight = loadCell.get_units(5);

        if (weight < -0.1)
        {
            Serial.println("Warning: Large negative weight detected - check calibration");
        }
        else if (weight < 0)
        {
            weight = 0;
        }
    }
    else
    {
        Serial.println("Load cell not ready!");
    }
    Serial.print("Weight: ");
    Serial.print(weight, 3);
    Serial.print("g ");
}
