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
const uint32_t LRL = 1500;      // Lower Rate Limit in ms (60 bpm)
const uint32_t URL = 333;       // Upper Rate Limit in ms (shortest time before sensing)
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
    WAIT_VRP,   // UPPAAL: Waiting during VRP, invariant xv <= VRP
    WAIT_RI,    // UPPAAL: Waiting during Rate Interval, invariant xv <= LRL, can't sense until URL
    SENSE_READY // UPPAAL: Ready to sense (xv >= URL) or pace (xv >= LRL), invariant xv <= LRL
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
const char MQTT_BROKER[] = "mqtt-dev.precise.seas.upenn.edu";
const int MQTT_PORT = 1883;

// Using hardware Serial1 (physical pins: TX=pin 1, RX=pin 0) for Arduino-to-Arduino communication
#define heartSerial Serial1

// ========================================
// Heartbeat Monitor Configuration (Milestone 2)
// ========================================
const uint32_t WINDOW_SIZE = 20;     // W: window size in seconds (20, 40, or 60)
const uint32_t PUBLISH_INTERVAL = 5; // P: publish interval in seconds (5, 10, 15, or 20)
const uint8_t PACE_THRESHOLD = 70;   // Alarm if >70% of beats are paced (slow heart)

// Beat tracking structure
struct BeatEvent
{
    uint32_t timestamp; // Time in milliseconds
    bool wasPaced;      // true if paced, false if sensed
};

// Circular buffer for beat events
#define MAX_BEATS 200 // Maximum beats to track (enough for 60s window at high rates)
BeatEvent beatBuffer[MAX_BEATS];
volatile uint16_t beatCount = 0;
volatile uint16_t beatIndex = 0;
SemaphoreHandle_t beatMutex;

// ========================================
// Forward Declarations
// ========================================
void sendPaceSignal();
void handleSenseSignal();
void transitionToState(PacemakerState newState);
void recordBeat(bool wasPaced);
void calculateAndPublishMetrics();

// ========================================
// Task: Heart Signal Polling
// ========================================
// This task implements the pacemaker controller state machine
// based on the UPPAAL VVI mode model
void TaskHeartPolling(void *pvParameters)
{
    Serial.println(F("[Pacemaker] HeartPolling task started"));

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
                Serial.print(F("[Pacemaker] VRP complete (xv="));
                Serial.print(timeInState);
                Serial.println(F(" ms)"));
                // UPPAAL: WaitVRP -> WaitRI assigns xv = 0
                stateStartTime = millis();
                transitionToState(WAIT_RI);
            }
            break;

        case WAIT_RI:
            // UPPAAL: WaitRI -> SenseReady when xv >= URL
            if (timeInState >= URL)
            {
                Serial.print(F("[Pacemaker] URL reached (xv="));
                Serial.print(timeInState);
                Serial.println(F(" ms)"));
                transitionToState(SENSE_READY);
            }
            // UPPAAL: WaitRI -> PaceEvent when xv >= LRL (shouldn't happen if URL < LRL)
            else if (timeInState >= LRL)
            {
                Serial.print(F("[Pacemaker] LRL timeout in WAIT_RI (xv="));
                Serial.print(timeInState);
                Serial.println(F(" ms) -> PACE"));
                sendPaceSignal();
            }
            break;

        case SENSE_READY:
            // UPPAAL: SenseReady -> PaceEvent when xv >= LRL
            if (timeInState >= LRL)
            {
                Serial.print(F("[Pacemaker] LRL timeout (xv="));
                Serial.print(timeInState);
                Serial.println(F(" ms) -> PACE"));
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

// Transition to a new state (clock continues unless explicitly reset)
void transitionToState(PacemakerState newState)
{
    // Print state transition
    Serial.print(F("[Pacemaker] State change: "));
    switch (currentState)
    {
    case WAIT_VRP:
        Serial.print(F("WAIT_VRP"));
        break;
    case WAIT_RI:
        Serial.print(F("WAIT_RI"));
        break;
    case SENSE_READY:
        Serial.print(F("SENSE_READY"));
        break;
    }
    Serial.print(F(" -> "));
    switch (newState)
    {
    case WAIT_VRP:
        Serial.println(F("WAIT_VRP"));
        break;
    case WAIT_RI:
        Serial.println(F("WAIT_RI"));
        break;
    case SENSE_READY:
        Serial.println(F("SENSE_READY"));
        break;
    }

    currentState = newState;
    // Note: stateStartTime is NOT reset here - clock keeps running
    // Only reset when UPPAAL model explicitly assigns xv = 0
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

    // Record sensed beat (Milestone 2)
    recordBeat(false);

    // UPPAAL: SenseEvent -> WaitVRP assigns xv = 0
    stateStartTime = millis();

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

    // Record paced beat (Milestone 2)
    recordBeat(true);

    // UPPAAL: PaceEvent -> WaitVRP assigns xv = 0
    stateStartTime = millis();

    // Transition to VRP (refractory period)
    transitionToState(WAIT_VRP);
}

// ========================================
// Heartbeat Monitoring Functions (Milestone 2)
// ========================================

// Record a beat event in the circular buffer
void recordBeat(bool wasPaced)
{
    xSemaphoreTake(beatMutex, portMAX_DELAY);

    beatBuffer[beatIndex].timestamp = millis();
    beatBuffer[beatIndex].wasPaced = wasPaced;

    beatIndex = (beatIndex + 1) % MAX_BEATS;
    if (beatCount < MAX_BEATS)
    {
        beatCount++;
    }

    xSemaphoreGive(beatMutex);
}

// Calculate metrics and publish to MQTT
void calculateAndPublishMetrics()
{
    xSemaphoreTake(beatMutex, portMAX_DELAY);

    uint32_t currentTime = millis();
    uint32_t windowMs = WINDOW_SIZE * 1000;
    uint32_t cutoffTime = currentTime - windowMs;

    // Count beats in the window
    uint16_t totalBeats = 0;
    uint16_t pacedBeats = 0;

    for (uint16_t i = 0; i < beatCount; i++)
    {
        uint16_t idx = (beatIndex + MAX_BEATS - beatCount + i) % MAX_BEATS;
        if (beatBuffer[idx].timestamp >= cutoffTime)
        {
            totalBeats++;
            if (beatBuffer[idx].wasPaced)
            {
                pacedBeats++;
            }
        }
    }

    xSemaphoreGive(beatMutex);

    // Calculate average heart rate (beats per minute)
    float avgHeartRate = 0.0;
    if (totalBeats > 1)
    {
        avgHeartRate = (totalBeats * 60.0) / WINDOW_SIZE;
    }

    // Calculate pace percentage
    float pacePercentage = 0.0;
    if (totalBeats > 0)
    {
        pacePercentage = (pacedBeats * 100.0) / totalBeats;
    }

    // Detect alarms
    bool slowHeartAlarm = (pacePercentage > PACE_THRESHOLD);
    bool fastHeartAlarm = false;

    // Check if heart rate is too fast (average inter-beat interval < URL)
    // avgHeartRate in bpm, URL is in ms
    // Convert: if avgHeartRate > 60000/URL bpm, it's too fast
    float maxHeartRate = 60000.0 / URL; // ~333 bpm for URL=180ms
    if (avgHeartRate > maxHeartRate)
    {
        fastHeartAlarm = true;
    }

    // Publish to MQTT
    if (mqttClient.connected())
    {
        char payload[200];

        // Publish average heart rate
        snprintf(payload, sizeof(payload),
                 "{\"window\":%lu,\"avg_bpm\":%.2f,\"total_beats\":%u,\"paced_beats\":%u,\"pace_pct\":%.1f}",
                 WINDOW_SIZE, avgHeartRate, totalBeats, pacedBeats, pacePercentage);
        mqttClient.beginMessage("cis441-541/heart_racer/pacemaker/heartrate");
        mqttClient.print(payload);
        mqttClient.endMessage();

        Serial.print(F("[MQTT] Published heartrate: "));
        Serial.println(payload);

        // Publish slow heart alarm
        if (slowHeartAlarm)
        {
            snprintf(payload, sizeof(payload),
                     "{\"type\":\"SLOW_HEART\",\"pace_pct\":%.1f,\"threshold\":%u}",
                     pacePercentage, PACE_THRESHOLD);
            mqttClient.beginMessage("cis441-541/heart_racer/pacemaker/alarm");
            mqttClient.print(payload);
            mqttClient.endMessage();

            Serial.print(F("[MQTT] ALARM - Slow heart: "));
            Serial.println(payload);
        }

        // Publish fast heart alarm
        if (fastHeartAlarm)
        {
            snprintf(payload, sizeof(payload),
                     "{\"type\":\"FAST_HEART\",\"avg_bpm\":%.2f,\"max_bpm\":%.2f}",
                     avgHeartRate, maxHeartRate);
            mqttClient.beginMessage("cis441-541/heart_racer/pacemaker/alarm");
            mqttClient.print(payload);
            mqttClient.endMessage();

            Serial.print(F("[MQTT] ALARM - Fast heart: "));
            Serial.println(payload);
        }

        // Publish status if no alarms
        if (!slowHeartAlarm && !fastHeartAlarm)
        {
            mqttClient.beginMessage("cis441-541/heart_racer/pacemaker/status");
            mqttClient.print("{\"status\":\"NORMAL\"}");
            mqttClient.endMessage();
        }
    }
    else
    {
        Serial.println(F("[MQTT] Not connected, skipping publish"));
    }
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
    Serial.println(F("[MQTT] Task started"));

    // Connect to MQTT broker
    mqttClient.setUsernamePassword(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);

    Serial.print(F("[MQTT] Connecting to broker: "));
    Serial.println(MQTT_BROKER);

    while (!mqttClient.connect(MQTT_BROKER, MQTT_PORT))
    {
        Serial.print(F("[MQTT] Connection failed, error: "));
        Serial.println(mqttClient.connectError());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    Serial.println(F("[MQTT] Connected to broker"));

    // Publish interval tracking
    uint32_t lastPublishTime = millis();
    uint32_t publishIntervalMs = PUBLISH_INTERVAL * 1000;

    while (true)
    {
        // Keep MQTT connection alive
        mqttClient.poll();

        // Check if it's time to publish
        uint32_t currentTime = millis();
        if (currentTime - lastPublishTime >= publishIntervalMs)
        {
            calculateAndPublishMetrics();
            lastPublishTime = currentTime;
        }

        // Reconnect if disconnected
        if (!mqttClient.connected())
        {
            Serial.println(F("[MQTT] Disconnected, reconnecting..."));
            while (!mqttClient.connect(MQTT_BROKER, MQTT_PORT))
            {
                Serial.print(F("[MQTT] Reconnection failed, error: "));
                Serial.println(mqttClient.connectError());
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            Serial.println(F("[MQTT] Reconnected"));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
        Serial.println(F("[ERROR] Failed to create state mutex"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Pacemaker] State mutex created"));

    // Initialize mutex for beat tracking (Milestone 2)
    beatMutex = xSemaphoreCreateMutex();
    if (beatMutex == NULL)
    {
        Serial.println(F("[ERROR] Failed to create beat mutex"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Pacemaker] Beat mutex created"));

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

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here if scheduler started successfully
    Serial.println(F("[ERROR] Scheduler failed to start!"));
    while (1)
    {
        delay(1000);
    }
}

void loop()
{
    // Empty. Tasks are handled by FreeRTOS
}