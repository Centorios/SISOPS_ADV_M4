#include <ESP32Servo.h>
#include <HX711.h>

// LedPin1 se enciende si falta agua (potenciometro)
// LedPin2 se enciende si falta comida (ultrasonido)
// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)

#define LedPin1 22
#define LedPin2 23
#define LoadCellDTPin 4
#define LoadCellSCKPin 2
#define TriggerPin 27
#define EchoPin 26
#define ServoPin 33
#define PotentiometerPin 35

#define PotThreshold 2048
#define DistanceThreshold 20

#define WeightThreshold 1.0
int const ServoLowWeightPosition = 90;
int const ServoNormalPosition = 0;

#define ledResolution 8
#define ledFrequency 5000
#define ledHigh 255
#define ledLow 0

int potValue 				   = 0 ;
int angle 				   = 0 ;
int objectTime 		 = 0;
int objectDistance  = 0;
float weight = 0.0;

Servo servo1;
HX711 loadCell;

float calibration_factor = 420.0;

void setup()
{
  Serial.begin(9600);
  Serial.println("System starting...");
  
  servo1.attach(ServoPin, 500, 2500);
  servo1.write(ServoNormalPosition);

  loadCell.begin(LoadCellDTPin, LoadCellSCKPin);
  
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

  pinMode(TriggerPin, OUTPUT);  
  pinMode(EchoPin, INPUT);

  ledcAttach(LedPin1, ledFrequency, ledResolution);
  ledcAttach(LedPin2, ledFrequency, ledResolution);

  delay(1000);
}

void loop()
{
  readSensors();
  performCalculations();
  performActions();
  showLogs();

  delay(1000);
}

long readUltrasonicSensor()
{
  digitalWrite(TriggerPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(TriggerPin, LOW);
  
  return pulseIn(EchoPin, HIGH);
}


void readLoadCell()
{
  if (loadCell.is_ready()) {
    weight = loadCell.get_units(5);
    
    if (weight < -0.1) {
      Serial.println("Warning: Large negative weight detected - check calibration");
    } else if (weight < 0) {
      weight = 0;
    }
  } else {
    Serial.println("Load cell not ready!");
  }
}

void readSensors()
{
  potValue = analogRead(PotentiometerPin);
  
  objectTime = readUltrasonicSensor();

  readLoadCell();

}

void performCalculations() 
{
  angle=map(potValue,0,4096,0,180);
  
  objectDistance=0.01723*objectTime;
}

void performActions()
{
  if (weight < WeightThreshold) {
    servo1.write(ServoLowWeightPosition);
    Serial.println("Low weight detected! Servo moved to special position.");
  } 
  else if (weight >= WeightThreshold) {
    servo1.write(ServoNormalPosition);
    Serial.println("Normal weight restored. Servo in normal position.");
  }

  if (potValue > PotThreshold) {
  ledcWrite(LedPin1, ledHigh);
  } else {
  ledcWrite(LedPin1, ledLow);
  }
  
  if (objectDistance > DistanceThreshold) {

    ledcWrite(LedPin2, ledHigh);

  } else {

    ledcWrite(LedPin2, ledLow);
  }
}

void showLogs()
{
  Serial.print("Weight: ");
  Serial.print(weight, 3);
  Serial.print("kg ");

  Serial.print("PotValue:");
  Serial.print(potValue);
   
  Serial.print(" Angle:");
  Serial.print(angle);
 
  Serial.print(" Distance: ");
  Serial.print(objectDistance);
  Serial.print("cm");
  
  Serial.print(" LED1:");
  Serial.print(ledcRead(LedPin1) == ledHigh ? "ON" : "OFF");
  
  Serial.print(" LED2:");
  Serial.print(ledcRead(LedPin2) == ledHigh ? "ON" : "OFF");
  
  Serial.println();
}