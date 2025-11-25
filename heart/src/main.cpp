#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// ========================================
// Heart Behavior Constants (from UPPAAL model)
// ========================================
const uint32_t minIBI = 400;  // Minimum Inter-Beat Interval in ms
const uint32_t maxIBI = 1200; // Maximum Inter-Beat Interval in ms

// Communication protocol
const char SENSE_SIGNAL = 'S'; // Heart sends 'S' when it beats intrinsically
const char PACE_SIGNAL = 'P';  // Pacemaker sends 'P' to pace the heart

// ========================================
// Heart State Machine (based on UPPAAL Heart template)
// ========================================
enum HeartState
{
    RESTING,     // Initial state, waiting for intrinsic beat or pace
    SENSE_READY, // Ready to generate intrinsic beat (xh >= minIBI)
    SENSE_EVENT, // Committed state: sending sense signal
    PACED_EVENT  // Committed state: received pace signal
};

// ========================================
// Global State Variables
// ========================================
volatile HeartState currentState = RESTING;
volatile uint32_t stateStartTime = 0;
volatile uint32_t xh = 0; // Heart clock: time since last beat (intrinsic or paced)

// Random beat interval for this cycle (between minIBI and maxIBI)
volatile uint32_t currentBeatInterval = minIBI;

// Mutex for protecting shared heart state
SemaphoreHandle_t stateMutex;

// Using hardware Serial1 (physical pins: TX=pin 1, RX=pin 0) for Arduino-to-Arduino communication
#define pacemakerSerial Serial1

// ========================================
// Forward Declarations
// ========================================
void transitionToState(HeartState newState);
void sendSenseSignal();
void handlePaceSignal();
uint32_t generateRandomBeatInterval();

// ========================================
// Task: Heart State Machine
// ========================================
// This task implements the heart's ventricle behavior
// based on the UPPAAL Heart model
void TaskHeartStateMachine(void *pvParameters)
{
    Serial.println(F("[Heart] HeartStateMachine task started"));

    while (true)
    {
        uint32_t currentTime = millis();
        xh = currentTime - stateStartTime; // Update heart clock

        // Check for incoming pace signals from pacemaker
        if (pacemakerSerial.available() > 0)
        {
            char signal = pacemakerSerial.read();

            if (signal == PACE_SIGNAL)
            {
                xSemaphoreTake(stateMutex, portMAX_DELAY);

                // UPPAAL: Resting -> PacedEvent when pace? received and xh < minIBI
                if (currentState == RESTING && xh < minIBI)
                {
                    Serial.println(F("[Heart] PACE received (early) -> PACED_EVENT"));
                    handlePaceSignal();
                }
                else
                {
                    Serial.print(F("[Heart] PACE ignored (xh="));
                    Serial.print(xh);
                    Serial.print(F(", state="));
                    Serial.print(currentState);
                    Serial.println(F(")"));
                }

                xSemaphoreGive(stateMutex);
            }
        }

        // State machine transitions based on timing
        xSemaphoreTake(stateMutex, portMAX_DELAY);

        switch (currentState)
        {
        case RESTING:
            // UPPAAL: Resting -> SenseReady when xh >= minIBI
            if (xh >= minIBI)
            {
                Serial.print(F("[Heart] minIBI reached (xh="));
                Serial.print(xh);
                Serial.println(F(") -> SENSE_READY"));
                transitionToState(SENSE_READY);
            }
            break;

        case SENSE_READY:
            // UPPAAL: SenseReady -> SenseEvent (intrinsic beat occurs)
            // The heart beats intrinsically at a random time between minIBI and maxIBI
            if (xh >= currentBeatInterval)
            {
                Serial.print(F("[Heart] Intrinsic beat time reached (xh="));
                Serial.print(xh);
                Serial.println(F(") -> SENSE_EVENT"));
                transitionToState(SENSE_EVENT);
            }
            break;

        case SENSE_EVENT:
            // UPPAAL: SenseEvent -> Resting (committed state, immediate transition)
            sendSenseSignal();
            Serial.println(F("[Heart] Sense signal sent -> RESTING"));
            currentBeatInterval = generateRandomBeatInterval();
            Serial.print(F("[Heart] Next beat interval: "));
            Serial.println(currentBeatInterval);
            // UPPAAL: SenseEvent -> Resting assigns xh = 0
            stateStartTime = millis();
            transitionToState(RESTING);
            break;

        case PACED_EVENT:
            // UPPAAL: PacedEvent -> Resting (committed state, immediate transition)
            Serial.println(F("[Heart] Pace processed -> RESTING"));
            currentBeatInterval = generateRandomBeatInterval();
            Serial.print(F("[Heart] Next beat interval: "));
            Serial.println(currentBeatInterval);
            // UPPAAL: PacedEvent -> Resting assigns xh = 0
            stateStartTime = millis();
            transitionToState(RESTING);
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
void transitionToState(HeartState newState)
{
    currentState = newState;
    // Note: stateStartTime is NOT reset here - clock keeps running
    // Only reset when UPPAAL model explicitly assigns xh = 0
}

// Send a sense signal to the pacemaker
// UPPAAL: SenseEvent sends sense! signal
void sendSenseSignal()
{
    pacemakerSerial.write(SENSE_SIGNAL);
    Serial.println(F("[Heart] SENSE signal sent to pacemaker"));
}

// Handle a pace signal from the pacemaker
// UPPAAL: Resting -> PacedEvent on pace?
void handlePaceSignal()
{
    Serial.println(F("[Heart] Processing PACE signal"));
    transitionToState(PACED_EVENT);
}

// Generate a random beat interval between minIBI and maxIBI
// This simulates the heart's natural variability
uint32_t generateRandomBeatInterval()
{
    // Use random() which returns values in [min, max)
    return random(minIBI, maxIBI + 1);
}

// ========================================
// Arduino Setup and Loop
// ========================================

void setup()
{
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println(F("\n[Heart] Heart Simulator Starting..."));
    Serial.print(F("[Heart] minIBI = "));
    Serial.print(minIBI);
    Serial.print(F(" ms, maxIBI = "));
    Serial.print(maxIBI);
    Serial.println(F(" ms"));

    // Initialize hardware Serial1 for pacemaker communication (physical pins: TX=pin 1, RX=pin 0)
    Serial1.begin(9600);
    Serial.println(F("[Heart] Serial1 initialized for pacemaker communication"));

    // Initialize random seed
    randomSeed(analogRead(0));

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
    Serial.println(F("[Heart] Mutex created"));

    // Initialize heart state machine
    stateStartTime = millis();
    currentState = RESTING;
    currentBeatInterval = generateRandomBeatInterval();
    Serial.print(F("[Heart] Initial beat interval: "));
    Serial.println(currentBeatInterval);
    Serial.println(F("[Heart] State machine initialized -> RESTING"));

    // Create FreeRTOS task
    BaseType_t result;

    result = xTaskCreate(
        TaskHeartStateMachine,
        "HeartStateMachine",
        256, // Stack size
        NULL,
        2, // Priority
        NULL);
    if (result != pdPASS)
    {
        Serial.println(F("[ERROR] Failed to create HeartStateMachine task"));
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println(F("[Heart] HeartStateMachine task created"));

    Serial.println(F("\n[System] Task initialized, starting scheduler..."));

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
