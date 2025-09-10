#include <ESP32Servo.h>
#include <HX711.h>

// LedPin1 se enciende si falta agua (potenciometro)
// LedPin2 se enciende si falta comida (ultrasonido)
// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)

int const LedPin1       = 22;
int const LedPin2       = 23;
int const LoadCellDTPin       = 4;
int const LoadCellSCKPin       = 2;
int const TriggerPin       = 27;
int const EchoPin          = 26;
int const ServoPin		     = 33;
int const PotentiometerPin = 35;

int const PotThreshold = 2048;
int const DistanceThreshold = 20;

float const WeightThreshold = 1.0;
int const ServoLowWeightPosition = 90;
int const ServoNormalPosition = 0;

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
  pinMode(LedPin1, OUTPUT);
  pinMode(LedPin2, OUTPUT);

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

void readSensors()
{
  potValue = analogRead(PotentiometerPin);
  
  objectTime = readUltrasonicSensor();

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
    digitalWrite(LedPin1, HIGH);
  } else {
    digitalWrite(LedPin1, LOW);
  }
  
  if (objectDistance > DistanceThreshold) {
    digitalWrite(LedPin2, HIGH);
  } else {
    digitalWrite(LedPin2, LOW);
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
  Serial.print(digitalRead(LedPin1) ? "ON" : "OFF");
  
  Serial.print(" LED2:");
  Serial.print(digitalRead(LedPin2) ? "ON" : "OFF");
  
  Serial.println();
}
