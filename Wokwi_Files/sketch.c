#include <ESP32Servo.h>
#include <HX711.h>

// LedPinWater se enciende si falta agua (potenciometro)
// LedPinFood se enciende si falta comida (ultrasonido)
// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)

#define LedPinWater 22
#define LedPinFood 23
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

int currentState = 0;
int currentEvent = 0;

enum States
{
    STATE_IDLE = 0,
    STATE_LOAD_CELL_FULL = 1,
    STATE_LOAD_CELL_EMPTY = 2,
    STATE_WATER_LEVEL_DOWN = 3,
    STATE_WATER_LEVEL_OK = 4,
    STATE_ULTRASOUND_FAR = 5,
    STATE_ULTRASOUND_CLOSE = 6,
};

enum Events
{
    INSUFFICIENT_FOOD = 0,
    SUFFICIENT_FOOD = 1,
    INSUFFICIENT_WATER = 2,
    SUFFICIENT_WATER = 3,
    OBJECT_NEARBY = 4,
    OBJECT_FAR = 5,
    RETURN_TO_IDLE = 6,
};

#define ledResolution 8
#define ledFrequency 5000
#define ledHigh 255
#define ledLow 0

int potValue = 0;
int angle = 0;
int objectTime = 0;
int objectDistance = 0;
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

    while (!loadCell.is_ready())
    {
        Serial.println("Waiting for load cell...");
        delay(100);
    }

    loadCell.set_scale(calibration_factor);

    Serial.println("Remove all weight from the load cell and press any key to tare...");
    while (!Serial.available())
    {
        delay(100);
    }
    Serial.read();

    loadCell.tare(10);

    Serial.println("Load cell tared and ready!");
    Serial.print("Current calibration factor: ");
    Serial.println(calibration_factor);

    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);

    ledcAttach(LedPinWater, ledFrequency, ledResolution);
    ledcAttach(LedPinFood, ledFrequency, ledResolution);

}

void loop()
{
    stateMachine();
    delay(1000);
}

void stateMachine()
{
    showLogs();
    switch (currentState)
    {
    case STATE_IDLE:
        potValue = analogRead(PotentiometerPin);
        objectTime = readUltrasonicSensor();
        readLoadCell();
        performCalculations();
        getEvent();
        getNewState();
        break;

    case STATE_LOAD_CELL_FULL:

        switch (currentEvent)
        {
        case SUFFICIENT_FOOD:
            HandleLoadCellFull();
            break;

        case INSUFFICIENT_FOOD:
            currentState = STATE_LOAD_CELL_EMPTY;
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    case STATE_LOAD_CELL_EMPTY:

        switch (currentEvent)
        {
        case SUFFICIENT_FOOD:
            currentState = STATE_LOAD_CELL_FULL;
            break;

        case INSUFFICIENT_FOOD:

            HandleLoadCellEmpty();
            readLoadCell();
            currentEvent = calculateFood();
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    case STATE_WATER_LEVEL_DOWN:

        switch (currentEvent)
        {
        case SUFFICIENT_WATER:
            currentState = STATE_WATER_LEVEL_OK;
            break;
        case INSUFFICIENT_WATER:
            HandleWaterLevelDown();
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    case STATE_WATER_LEVEL_OK:

        switch (currentEvent)
        {
        case SUFFICIENT_WATER:
            HandleWaterLevelOk();
            break;
        case INSUFFICIENT_WATER:
            currentState = STATE_WATER_LEVEL_DOWN;
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    case STATE_ULTRASOUND_FAR:

        switch (currentEvent)
        {
        case OBJECT_FAR:
            HandleUltrasoundFar();
            break;

        case OBJECT_NEARBY:
            currentState = STATE_ULTRASOUND_CLOSE;
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    case STATE_ULTRASOUND_CLOSE:

        switch (currentEvent)
        {
        case OBJECT_FAR:
            currentState = STATE_ULTRASOUND_FAR;
            break;

        case OBJECT_NEARBY:
            HandleUltrasoundClose();
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;

    default:
        Serial.println("Unknown state!");
        break;
    }

    // showLogs();
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
}

void performCalculations()
{
    angle = map(potValue, 0, 4096, 0, 180);
    objectDistance = 0.01723 * objectTime;
}

void HandleLoadCellFull()
{
    servo1.write(ServoNormalPosition);
    Serial.println("Normal weight restored. Servo in normal position.");
    currentState = STATE_IDLE;
}

void HandleLoadCellEmpty()
{
    servo1.write(ServoLowWeightPosition);
    Serial.println("Low weight detected! Servo moved to special position.");
    currentState = STATE_LOAD_CELL_EMPTY;
}

void HandleWaterLevelDown()
{
    ledcWrite(LedPinWater, ledHigh);
    Serial.println("Low water level detected! LED1 ON.");
    currentState = STATE_IDLE;
}

void HandleWaterLevelOk()
{
    ledcWrite(LedPinWater, ledLow);
    Serial.println("Water level OK. LED1 OFF.");
    currentState = STATE_IDLE;
}

void HandleUltrasoundFar()
{
    ledcWrite(LedPinFood, ledHigh);
    Serial.println("No object detected nearby! LED2 ON.");
    currentState = STATE_IDLE;
}

void HandleUltrasoundClose()
{
    ledcWrite(LedPinFood, ledLow);
    Serial.println("Object detected nearby. LED2 OFF.");
    currentState = STATE_IDLE;
}

int calculateFood()
{
    if (weight < WeightThreshold)
    {
        return INSUFFICIENT_FOOD;
    }
    else
    {
        return SUFFICIENT_FOOD;
    }
}

void getEvent()
{

    currentEvent = -1;
    // most prioritized event first
    if (weight < WeightThreshold && currentEvent == -1)
    {
        currentEvent = INSUFFICIENT_FOOD;
    }

    if (potValue < PotThreshold && currentEvent == -1)
    {
        currentEvent = INSUFFICIENT_WATER;
    }

    if (objectDistance >= DistanceThreshold && objectDistance > 0 && currentEvent == -1)
    {
        currentEvent = OBJECT_FAR;
    }

    if (potValue >= PotThreshold && currentEvent == -1)
    {
        currentEvent = SUFFICIENT_WATER;
    }

    if (objectDistance < DistanceThreshold && objectDistance > 0 && currentEvent == -1)
    {
        currentEvent = OBJECT_NEARBY;
    }

    if (currentEvent == -1)
    {
        currentEvent = RETURN_TO_IDLE;
    }



}

void getNewState()
{
    switch (currentEvent)
    {
    case INSUFFICIENT_FOOD:
        currentState = STATE_LOAD_CELL_EMPTY;
        break;
    case INSUFFICIENT_WATER:
        currentState = STATE_WATER_LEVEL_DOWN;
        break;
    case SUFFICIENT_WATER:
        currentState = STATE_WATER_LEVEL_OK;
        break;
    case OBJECT_NEARBY:
        currentState = STATE_ULTRASOUND_CLOSE;
        break;
    case OBJECT_FAR:
        currentState = STATE_ULTRASOUND_FAR;
        break;
    case RETURN_TO_IDLE:
        currentState = STATE_IDLE;
        break;
    default:
        Serial.println("Unknown event!");
        break;
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
    Serial.print(ledcRead(LedPinWater));

    Serial.print(" LED2:");
    Serial.print(ledcRead(LedPinFood));

    Serial.print(" State:");
    Serial.print(currentState);

    Serial.print(" Event:");
    Serial.print(currentEvent);

    Serial.println();

}


