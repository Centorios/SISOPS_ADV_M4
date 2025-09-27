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

enum Priorities
{
    PRIORITY_HIGH = 0,
    PRIORITY_MEDIUM = 1,
    PRIORITY_LOW = 2,
};


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

int potValue = 0;
int angle = 0;
int objectTime = 0;
int objectDistance = 0;
float weight = 0.0;
float calibration_factor = 420.0;
unsigned long timeSinceBoot;

Servo servo1;
HX711 loadCell;

TimerHandle_t callbackTimer;
TaskHandle_t TaskHandlerCallback;

void callbackTemporizador(TimerHandle_t xTimer) {
    // Wake the servo task (counting notification)
    xTaskNotifyGive(TaskHandlerCallback);
}

void concurrentServoTask(void *parameters) {
    ulTaskNotifyTake(pdTRUE, 0);
    while(1) {
        // Block here until the timer gives a notification.
        // pdTRUE: clear (consume) the count so we don't accumulate stale ticks.
        // portMAX_DELAY: wait indefinitely.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Once awakened, perform the logic that was previously time-delayed.
        currentEvent = calculateFood();
        if (currentEvent == INSUFFICIENT_FOOD) {
            HandleLoadCellEmpty();
        } else if (currentEvent == SUFFICIENT_FOOD) {
            HandleLoadCellFull();
        }
    }
}

void concurrentReadSensorsAndPerformCalculations(void *parameters) {
    while(1) {
        potValue = analogRead(PotentiometerPin);
        objectTime = readUltrasonicSensor();
        readLoadCell();
        performCalculations();
        vTaskDelay(TIMER_GLOBAL);
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

    xTaskCreate(concurrentReadSensorsAndPerformCalculations,"concurrent_sensor_task",TAM_PILA, NULL, PRIORITY_MEDIUM, NULL);

    xTaskCreate(concurrentServoTask,"concurrent_servo_task",TAM_PILA, NULL, PRIORITY_MEDIUM, &TaskHandlerCallback);
    callbackTimer = xTimerCreate("timer",pdMS_TO_TICKS(TIMER_GLOBAL),pdTRUE,NULL,callbackTemporizador);
    xTimerStart(callbackTimer, 0);
    timeSinceBoot = millis();
}

//core 1 by default
void loop()
{
    stateMachine();
}

void stateMachine()
{
    if(timestampEnabler(&timeSinceBoot)){
    showLogs();
    }


    switch (currentState)
    {
    case STATE_IDLE:
        getEvent();
        getNewState();
    break;
/*
    case STATE_LOAD_CELL_FULL:

        switch (currentEvent)
        {
        case SUFFICIENT_FOOD:
            break;

        case INSUFFICIENT_FOOD:
            currentState = STATE_LOAD_CELL_EMPTY;
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;
*/
/*
    case STATE_LOAD_CELL_EMPTY:

        switch (currentEvent)
        {
        case SUFFICIENT_FOOD:
            currentState = STATE_LOAD_CELL_FULL;
            break;

        case INSUFFICIENT_FOOD:
            currentEvent = calculateFood();
            break;

        default:
            Serial.println("Unknown event!");
            break;
        }

        break;
*/
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
            if (timestampEnabler(&timeSinceBoot))
            {
                Serial.println("Unknown event!");
            }
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
            if (timestampEnabler(&timeSinceBoot))
            {
                Serial.println("Unknown event!");
            }
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
            if (timestampEnabler(&timeSinceBoot))
            {
                Serial.println("Unknown event!");
            }
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
            if (timestampEnabler(&timeSinceBoot))
            {
                Serial.println("Unknown event!");
            }
            break;
        }

        break;

    default:
        if (timestampEnabler(&timeSinceBoot))
        {
            Serial.println("Unknown event!");
        }
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

void HandleLoadCellFull()
{
    servo1.write(ServoNormalPosition);
    currentState = STATE_IDLE;
}

void HandleLoadCellEmpty()
{
    servo1.write(ServoLowWeightPosition);
    currentState = STATE_IDLE;
}

void HandleWaterLevelDown()
{
    ledcWrite(LedPinWater, ledHigh);
    currentState = STATE_IDLE;
}

void HandleWaterLevelOk()
{
    ledcWrite(LedPinWater, ledLow);
    currentState = STATE_IDLE;
}

void HandleUltrasoundFar()
{
    ledcWrite(LedPinFood, ledHigh);
    currentState = STATE_IDLE;
}

void HandleUltrasoundClose()
{
    ledcWrite(LedPinFood, ledLow);
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
    int lastEvent = currentEvent;
    currentEvent = STATE_IDLE;

    if (potValue < PotThreshold && currentEvent == STATE_IDLE && lastEvent != INSUFFICIENT_WATER)
    {
        currentEvent = INSUFFICIENT_WATER;
    }

    if (objectDistance >= DistanceThreshold && objectDistance > 0 && currentEvent == STATE_IDLE && lastEvent != OBJECT_FAR)
    {
        currentEvent = OBJECT_FAR;
    }

    if (potValue >= PotThreshold && currentEvent == STATE_IDLE && lastEvent != SUFFICIENT_WATER)
    {
        currentEvent = SUFFICIENT_WATER;
    }

    if (objectDistance < DistanceThreshold && objectDistance > 0 && currentEvent == STATE_IDLE && lastEvent != OBJECT_NEARBY)
    {
        currentEvent = OBJECT_NEARBY;
    }

    if (currentEvent == STATE_IDLE)
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

