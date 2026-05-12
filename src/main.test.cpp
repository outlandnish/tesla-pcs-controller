#include <Arduino.h>
#include <hwdefs.h>

// Define Serial1 instance using proper alternate function pins
HardwareSerial Serial1(PIN_USART1_RX, PIN_USART1_TX);

#define Debug Serial1

// State machine states (simplified from old firmware - manual mode without precharge/EVSE)
enum TestState {
  STATE_OFF = 0,
  STATE_WAITSTART = 1,
  STATE_ACTIVATE = 2,
  STATE_RUN = 3,
  STATE_STOP = 4
};

// State variables
TestState currentState = STATE_OFF;
uint32_t stateEntryTime = 0;
bool enableInput = false;

// Simulated inputs
bool simulateEnablePin = false;

// Timing
const uint32_t STARTUP_DELAY_MS = 1000;  // Delay before activation
const uint32_t RUN_TIME_MS = 10000;      // Run for 10 seconds
const uint32_t STOP_TIME_MS = 2000;      // Shutdown delay

void setup() {
  Debug.begin(115200);
  delay(100);
  Debug.println("========================================");
  Debug.println("PCS Boot-Up Test - Manual Mode");
  Debug.println("(No Precharge/HVENA, No EVSE)");
  Debug.println("========================================");

  // Configure outputs
  pinMode(PIN_PCS_ENABLE, OUTPUT);
  digitalWrite(PIN_PCS_ENABLE, LOW);  // Disable PCS (active high)

  pinMode(PIN_CHARGE_ENABLE, OUTPUT);
  digitalWrite(PIN_CHARGE_ENABLE, HIGH);  // Disable Charger (active low)

  pinMode(PIN_DCDC_ENABLE, OUTPUT);
  digitalWrite(PIN_DCDC_ENABLE, HIGH);  // Disable DCDC (active low)

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Debug.println("Pins configured - all outputs disabled");
  Debug.println("Starting state machine test in 3 seconds...");
  delay(3000);
  
  stateEntryTime = millis();
  Debug.println("\n=== State: OFF ===");
}

void stateMachine() {
  uint32_t now = millis();
  uint32_t timeInState = now - stateEntryTime;

  switch (currentState) {
    case STATE_OFF:
      // Initial state - all outputs disabled
      // Wait for enable signal
      if (simulateEnablePin) {
        Debug.println("Enable signal received");
        currentState = STATE_WAITSTART;
        stateEntryTime = now;
        Debug.println("\n=== State: WAITSTART ===");
      }
      break;

    case STATE_WAITSTART:
      // Wait for startup delay (simulates timedly parameter check)
      if (timeInState >= STARTUP_DELAY_MS) {
        Debug.println("Startup delay complete");
        currentState = STATE_ACTIVATE;
        stateEntryTime = now;
        Debug.println("\n=== State: ACTIVATE ===");
        
        // Skip ENABLE state (no precharge, no hvena_out)
        // Go directly to activating PCS and charger/DCDC
        Debug.println("Activating PCS...");
        digitalWrite(PIN_PCS_ENABLE, HIGH);  // Enable PCS (active high)
        Debug.println("  PIN_PCS_ENABLE = HIGH (PCS enabled)");
        
        Debug.println("Activating Charger...");
        digitalWrite(PIN_CHARGE_ENABLE, LOW);  // Enable Charger (active low)
        Debug.println("  PIN_CHARGE_ENABLE = LOW (Charger enabled)");
        
        Debug.println("Activating DCDC...");
        digitalWrite(PIN_DCDC_ENABLE, LOW);  // Enable DCDC (active low)
        Debug.println("  PIN_DCDC_ENABLE = LOW (DCDC enabled)");
        
        digitalWrite(PIN_LED, HIGH);  // LED on = running
      } else {
        Debug.printf("Waiting for startup delay... (%lu/%lu ms)\r\n", 
                     timeInState, STARTUP_DELAY_MS);
        delay(200);
      }
      break;

    case STATE_ACTIVATE:
      // In real firmware, would wait for PCS to report status
      // For this test, just wait a moment then transition to RUN
      if (timeInState >= 500) {  // 500ms for PCS to initialize
        Debug.println("PCS activation complete");
        currentState = STATE_RUN;
        stateEntryTime = now;
        Debug.println("\n=== State: RUN ===");
        Debug.println("System running normally");
        Debug.printf("Will run for %lu seconds\n", RUN_TIME_MS / 1000);
      }
      break;

    case STATE_RUN:
      // Normal operation
      // In real firmware, would:
      // - Send CAN messages to PCS
      // - Monitor voltages and currents
      // - Check for faults
      
      // Blink LED to show running
      if ((timeInState % 1000) < 100) {
        digitalWrite(PIN_LED, LOW);
      } else {
        digitalWrite(PIN_LED, HIGH);
      }
      
      // Check for stop condition
      if (!simulateEnablePin) {
        Debug.println("Enable signal removed - stopping");
        currentState = STATE_STOP;
        stateEntryTime = now;
        Debug.println("\n=== State: STOP ===");
      } else if (timeInState >= RUN_TIME_MS) {
        Debug.println("Run time complete - stopping");
        simulateEnablePin = false;  // Trigger stop
        currentState = STATE_STOP;
        stateEntryTime = now;
        Debug.println("\n=== State: STOP ===");
      } else {
        // Print status every 2 seconds
        if (timeInState % 2000 < 50) {
          Debug.printf("Running... (%lu/%lu seconds)\r\n", 
                       timeInState / 1000, RUN_TIME_MS / 1000);
        }
      }
      break;

    case STATE_STOP:
      // Shutdown sequence
      if (timeInState >= STOP_TIME_MS) {
        Debug.println("Shutdown delay complete - disabling all outputs");
        
        digitalWrite(PIN_PCS_ENABLE, LOW);      // Disable PCS (active high)
        digitalWrite(PIN_CHARGE_ENABLE, HIGH);  // Disable Charger
        digitalWrite(PIN_DCDC_ENABLE, HIGH);    // Disable DCDC
        digitalWrite(PIN_LED, LOW);             // LED off

        Debug.println("  PIN_PCS_ENABLE = LOW (PCS disabled)");
        Debug.println("  PIN_CHARGE_ENABLE = HIGH (Charger disabled)");
        Debug.println("  PIN_DCDC_ENABLE = HIGH (DCDC disabled)");
        
        currentState = STATE_OFF;
        stateEntryTime = now;
        Debug.println("\n=== State: OFF ===");
        Debug.println("========================================");
        Debug.println("Boot-up test cycle complete!");
        Debug.println("Test will restart in 5 seconds...");
        Debug.println("========================================\n");
        delay(5000);
      } else {
        Debug.printf("Stopping... (%lu/%lu ms)\r\n", 
                     timeInState, STOP_TIME_MS);
        delay(200);
      }
      break;
  }
}

void loop() {
  // Run state machine
  stateMachine();
  
  // Simulate enable signal for testing
  // In STATE_OFF, trigger enable after 2 seconds
  if (currentState == STATE_OFF && !simulateEnablePin) {
    uint32_t timeInOff = millis() - stateEntryTime;
    if (timeInOff >= 2000) {
      Debug.println("TEST: Simulating enable signal (manual mode)");
      simulateEnablePin = true;
    }
  }
  
  delay(50);  // Main loop delay
}