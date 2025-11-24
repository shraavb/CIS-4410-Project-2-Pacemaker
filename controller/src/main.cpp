#include <Arduino.h>
#include <vector>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "arduino_secrets.h"

// ========================================
// Pacemaker VVI Mode Constants (from UPPAAL model)
// ========================================
const uint32_t LRL = 1000;      // Lower Rate Limit in ms (60 bpm)
const uint32_t URL = 180;       // Upper Rate Limit in ms (shortest time before sensing)
const uint32_t VRP = 150;       // Ventricular Refractory Period in ms
const uint32_t HRI = LRL + 200; // Hysteresis Rate Interval

// Communication protocol
const char SENSE_SIGNAL = 'S'; // Heart sends 'S' when it beats
const char PACE_SIGNAL = 'P';  // Pacemaker sends 'P' to pace the heart

// ========================================
// Pacemaker State Machine (based on UPPAAL)
// ========================================
enum PacemakerState
{
    WAIT_VRP,   // Waiting during Ventricular Refractory Period (after pace or sense)
    WAIT_RI,    // Waiting during Rate Interval (can't sense yet, before URL)
    SENSE_READY // Ready to sense heart signals (after URL, before LRL)
};

// ========================================
// Global State Variables
// ========================================
volatile PacemakerState currentState = WAIT_RI;
volatile uint32_t stateStartTime = 0;
volatile uint32_t currentRI = LRL; // Current active Rate Interval
volatile bool lastWasPace = false;
volatile bool lastWasSense = false;

// Mutex for protecting shared pacemaker state
SemaphoreHandle_t stateMutex;

// WiFi and MQTT configuration
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Using hardware Serial1 (physical pins: TX=pin 1, RX=pin 0) for Arduino-to-Arduino communication
#define heartSerial Serial1

// ========================================
// Forward Declarations
// ========================================
void sendPaceSignal();
void handleSenseSignal();
void transitionToState(PacemakerState newState);

// ========================================
// Task: Heart Signal Polling
// ========================================
// This task implements the pacemaker controller state machine
// based on the UPPAAL VVI mode model
void TaskHeartPolling(void *pvParameters)
{
    while (true)
    {
        uint32_t currentTime = millis();
        uint32_t timeInState = currentTime - stateStartTime;

        // Check for incoming sense signals from heart
        if (heartSerial.available() > 0)
        {
            char signal = heartSerial.read();

            if (signal == SENSE_SIGNAL)
            {
                xSemaphoreTake(stateMutex, portMAX_DELAY);

                // Only process sense if we're in SENSE_READY state
                // (UPPAAL: sense? transition only enabled in SenseReady state)
                if (currentState == SENSE_READY)
                {
                    Serial.println(F("[Pacemaker] SENSE detected"));
                    handleSenseSignal();
                }
                else
                {
                    Serial.println(F("[Pacemaker] SENSE ignored (in refractory/wait period)"));
                }

                xSemaphoreGive(stateMutex);
            }
        }

        // State machine transitions based on timing
        xSemaphoreTake(stateMutex, portMAX_DELAY);

        switch (currentState)
        {
        case WAIT_VRP:
            // UPPAAL: WaitVRP -> WaitRI when xv >= VRP
            if (timeInState >= VRP)
            {
                Serial.println(F("[Pacemaker] VRP complete -> WAIT_RI"));
                transitionToState(WAIT_RI);
            }
            break;

        case WAIT_RI:
            // UPPAAL: WaitRI -> SenseReady when xv >= URL
            if (timeInState >= URL)
            {
                Serial.println(F("[Pacemaker] URL reached -> SENSE_READY"));
                transitionToState(SENSE_READY);
            }
            // UPPAAL: WaitRI -> PaceEvent when xv >= LRL (shouldn't happen if URL < LRL)
            else if (timeInState >= LRL)
            {
                Serial.println(F("[Pacemaker] LRL timeout in WAIT_RI -> PACE"));
                sendPaceSignal();
            }
            break;

        case SENSE_READY:
            // UPPAAL: SenseReady -> PaceEvent when xv >= LRL
            if (timeInState >= LRL)
            {
                Serial.println(F("[Pacemaker] LRL timeout -> PACE"));
                sendPaceSignal();
            }
            break;
        }

        xSemaphoreGive(stateMutex);

        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ========================================
// Helper Functions for State Machine
// ========================================

// Transition to a new state and reset the state timer
void transitionToState(PacemakerState newState)
{
    currentState = newState;
    stateStartTime = millis();
}

// Handle a sense signal from the heart
// UPPAAL: SenseReady -> SenseEvent -> WaitVRP
void handleSenseSignal()
{
    // Update state flags (from UPPAAL model)
    lastWasPace = false;
    lastWasSense = true;
    currentRI = HRI; // Use hysteresis rate interval after a sense

    Serial.println(F("[Pacemaker] Sense event processed -> WAIT_VRP"));

    // Transition to VRP (refractory period)
    transitionToState(WAIT_VRP);
}

// Send a pace signal to the heart
// UPPAAL: PaceEvent -> WaitVRP
void sendPaceSignal()
{
    // Update state flags (from UPPAAL model)
    lastWasPace = true;
    lastWasSense = false;
    currentRI = LRL; // Use lower rate limit after a pace

    // Send pace signal to heart via serial
    heartSerial.write(PACE_SIGNAL);
    Serial.println(F("[Pacemaker] PACE signal sent -> WAIT_VRP"));

    // Transition to VRP (refractory period)
    transitionToState(WAIT_VRP);
}

// ========================================
// MQTT and OpenAPS Tasks
// ========================================

void onMqttMessage(int messageSize)
{
    // TODO: Implement MQTT message callback
    // Handle attribute updates and CGM data
    // Update openAPS, current_BG, current_time, and flags as needed
}

void TaskMQTT(void *pvParameters)
{
    // TODO: Implement MQTT task
    // Continuously poll for MQTT messages
}

void setup()
{
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println(F("\n[Pacemaker] VVI Mode Controller Starting..."));

    // Initialize hardware Serial1 for heart communication (physical pins: TX=pin 1, RX=pin 0)
    Serial1.begin(9600);
    Serial.println(F("[Pacemaker] Serial1 initialized for heart communication"));

    // Initialize WiFi connection
    Serial.print(F("[Network] Connecting to WiFi"));
    WiFi.begin(SECRET_SSID, SECRET_PASS);

    unsigned retryCount = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print('.');
        if (++retryCount % 10 == 0)
        {
            Serial.println();
        }
    }
    Serial.println(F("\n[Network] WiFi connected"));
    Serial.print(F("[Network] IP address: "));
    Serial.println(WiFi.localIP());

    // Initialize mutex for state machine protection
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL)
    {
        Serial.println(F("[ERROR] Failed to create mutex"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Pacemaker] Mutex created"));

    // Initialize pacemaker state machine
    stateStartTime = millis();
    currentState = WAIT_RI;
    Serial.println(F("[Pacemaker] State machine initialized -> WAIT_RI"));

    // Create FreeRTOS tasks
    BaseType_t result;

    result = xTaskCreate(
        TaskHeartPolling,
        "HeartPolling",
        256, // Stack size
        NULL,
        2, // Priority (higher than MQTT/OpenAPS)
        NULL);
    if (result != pdPASS)
    {
        Serial.println(F("[ERROR] Failed to create HeartPolling task"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Pacemaker] HeartPolling task created"));

    result = xTaskCreate(
        TaskMQTT,
        "MQTT",
        512, // Stack size
        NULL,
        1, // Priority
        NULL);
    if (result != pdPASS)
    {
        Serial.println(F("[ERROR] Failed to create MQTT task"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Network] MQTT task created"));

    Serial.println(F("\n[System] All tasks initialized, starting scheduler..."));

    // Note: vTaskStartScheduler() is called automatically by the Arduino framework
    // for FreeRTOS_SAMD21
}

void loop()
{
    // Empty. Tasks are handled by FreeRTOS
}