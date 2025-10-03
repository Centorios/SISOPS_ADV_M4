#include <ESP32Servo.h>
#include <HX711.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// LedPinWater se enciende si falta agua (potenciometro)
// LedPinFood se enciende si falta comida (ultrasonido)
// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)

#define LedPinWater 22
#define LedPinFood 23
#define ledResolution 8
#define ledFrequency 5000
#define ledHigh 255
#define ledLow 0
#define LedPinWaterChannel 6
#define LedPinFoodChannel 7

#define LoadCellDTPin 4
#define LoadCellSCKPin 2
#define TriggerPin 27
#define EchoPin 26
#define ServoPin 33
#define PotentiometerPin 35

#define PotThreshold 2048
#define DistanceThreshold 20

#define TIMER_CELL 1000 //1000ms = 1s
#define TIMER_GLOBAL 100 //100ms
#define TIMER_LOGS 1500 //1500ms

//120g
#define WeightThreshold 0.120

#define TAM_PILA 1024

enum States
{
    STATE_IDLE = 1,
    STATE_LOAD_CELL_EMPTY = 2,
    STATE_WATER_LEVEL_DOWN = 3,
    STATE_WATER_LEVEL_OK = 4,
    STATE_ULTRASOUND_FAR = 5,
    STATE_ULTRASOUND_CLOSE = 6,
};

enum Events
{
    INSUFFICIENT_FOOD = 1,
    INSUFFICIENT_WATER = 2,
    SUFFICIENT_WATER = 3,
    OBJECT_NEARBY = 4,
    OBJECT_FAR = 5,
    RETURN_TO_IDLE = 6,
};

int const ServoLowWeightPosition = 90;
int const ServoNormalPosition = 0;

int currentState = 0;
int currentEvent = 0;
int potValue = 0;
int angle = 0;
int objectTime = 0;
int objectDistance = 0;
int waterLed = 0;
int ultraLed = 0;
int ServoChk = 0;

float weight = 0.0;
float calibration_factor = 420.0;

unsigned long timeSinceBoot;
unsigned long timeCell;

Servo servo1;
HX711 loadCell;

static TaskHandle_t ServoHandler = NULL;

static void concurrentServoTask(void *parameters) {
    while(1)
    {   
        // Espera notificación
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        // Resuelve tarea del servo
        while (weight < WeightThreshold)
        {
            servo1.write(ServoNormalPosition);
        }
        servo1.write(ServoLowWeightPosition);
        ServoChk = false;
    }
}

bool timestampEnabler(unsigned long* lastTimestamp)
{
    if ((millis() - *lastTimestamp) >= TIMER_LOGS)
    {
        *lastTimestamp = millis();
        return true;
    }
    return false;
}

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
        delay(TIMER_CELL);
    }

    loadCell.set_scale(calibration_factor);

    Serial.println("Remove all weight from the load cell and press any key to tare...");
    while (!Serial.available()) { }
    Serial.read();
    loadCell.tare(10);

    Serial.println("Load cell tared and ready!");
    Serial.print("Current calibration factor: ");
    Serial.println(calibration_factor);

    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    ledcAttachChannel(LedPinWater, ledFrequency, ledResolution, LedPinWaterChannel);
    ledcAttachChannel(LedPinFood, ledFrequency, ledResolution, LedPinFoodChannel);

    xTaskCreate(concurrentServoTask,"concurrent_servo_task",TAM_PILA, NULL, 0, &ServoHandler);

    timeSinceBoot = timeCell = millis();
}

void loop()
{
    //Lectura de sensores
    potValue = analogRead(PotentiometerPin);
    objectTime = readUltrasonicSensor();
    
    if ((millis() - timeCell) >= TIMER_CELL)
    {   
        readLoadCell();
    }
    
    //Ejecucion de calculos en base a sensores
    performCalculations();
    
    //Obtencion del evento actual en base a sensores
    getEvent();

    stateMachine();
}

void stateMachine()
{   
    //Obtencion del estado actual en base al evento
    getState();

    if(timestampEnabler(&timeSinceBoot)){
        //Logs periodicos
        showLogs();
    }

    switch (currentState)
    {
    case STATE_IDLE:
        //Do nothing
        break;

    case STATE_WATER_LEVEL_DOWN:
        ledcWrite(LedPinWater, ledHigh);
        waterLed = true;
        break;

    case STATE_WATER_LEVEL_OK:
        ledcWrite(LedPinWater, ledLow);
        waterLed = false;
        break;

    case STATE_ULTRASOUND_FAR:
        ledcWrite(LedPinFood, ledHigh);
        ultraLed = true;
        break;

    case STATE_ULTRASOUND_CLOSE:
        ledcWrite(LedPinFood, ledLow);
        ultraLed = false;
        break;

    case STATE_LOAD_CELL_EMPTY:
        xTaskNotifyGive(ServoHandler);
        ServoChk = true;
        break;

    default:
        Serial.println("Unknown State!");
        break;
    }
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
    timeCell = millis();

    if (loadCell.is_ready())
    {
        weight = loadCell.get_units(5);
        if (weight < 0)
        {
            weight = 0;
        }
    }
}

void performCalculations()
{
    angle = map(potValue, 0, 4096, 0, 180);
    objectDistance = 0.01723 * objectTime;
}

void getEvent()
{
    //Vuelvo a IDLE por defecto
    currentEvent = RETURN_TO_IDLE;

    //Evaluo eventos segun sensor de agua
    switch (waterLed)
    {
        case true:
            if (potValue >= PotThreshold)
            {
                currentEvent = SUFFICIENT_WATER;
            }
            break;

        case false:
            if (potValue < PotThreshold)
            {
                currentEvent = INSUFFICIENT_WATER;
            }
            break;
    }

    //Evaluo eventos segun sensor de ultrasonido
    switch (ultraLed)
    {
        case true:
            if (objectDistance < DistanceThreshold)
            {
                currentEvent = OBJECT_NEARBY;
            }
            break;

        case false:
            if (objectDistance >= DistanceThreshold)
            {
                currentEvent = OBJECT_FAR;
            }
            break;
    }

    switch(ServoChk)
    {
        case true:
            //Do nothinf
        break;

        case false:
            if (weight < WeightThreshold)
            {
                currentEvent = INSUFFICIENT_FOOD;
            }
        break;
    }
}

void getState()
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
        Serial.println("Unknown Event!");
        break;
    }
}

void showLogs()
{
    Serial.print("Weight: ");
    Serial.print(weight, 3);
    Serial.print("g");

    Serial.print(" PotValue:");
    Serial.print(potValue);

    Serial.print(" Distance: ");
    Serial.print(objectDistance);
    Serial.print("cm");

    Serial.print(" State:");
    Serial.print(currentState);

    Serial.print(" Event:");
    Serial.print(currentEvent);

    Serial.println();
}