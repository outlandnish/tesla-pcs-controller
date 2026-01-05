/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 *
 * High-level controller that manages the PCS hardware pins and coordinates
 * with the low-level PCSCan communication layer.
 */

#include "pcs.h"
#include "debug_serial.h"  // Use common debug serial

// Import debug flag from main.cpp
#ifndef DEBUG_PCS_STATE
#define DEBUG_PCS_STATE 1
#endif

// Initialize static members
uint8_t PCSController::pcs_enable_pin = 0;
uint8_t PCSController::charge_enable_pin = 0;
uint8_t PCSController::dcdc_enable_pin = 0;

bool PCSController::pcs_pin_enabled = false;
bool PCSController::charge_pin_enabled = false;
bool PCSController::dcdc_pin_enabled = false;
bool PCSController::can_enabled = false;
bool PCSController::manual_override = false;

PCSState PCSController::current_state = PCS_STATE_INIT;
PCSState PCSController::target_state = PCS_STATE_OFF;
uint8_t PCSController::precharge_timer = 0;
uint8_t PCSController::pcs_wakeup_timer = 0;
uint8_t PCSController::powerdown_timer = 0;
HardwareTimer *PCSController::message_timer = nullptr;
volatile bool PCSController::flag_10ms = false;
volatile bool PCSController::flag_50ms = false;
volatile bool PCSController::flag_100ms = false;
volatile uint16_t PCSController::timer_ticks = 0;
QueueHandle_t PCSController::command_queue = nullptr;

void PCSController::begin(CANBus *ipc_can_bus, uint8_t pcs_en_pin, uint8_t charge_en_pin, uint8_t dcdc_en_pin) {
  DEBUG_SERIAL.println("PCS: Configuring pins...");
  // Store pin assignments
  pcs_enable_pin = pcs_en_pin;
  charge_enable_pin = charge_en_pin;
  dcdc_enable_pin = dcdc_en_pin;

  // Configure pins as outputs
  pinMode(pcs_enable_pin, OUTPUT);
  pinMode(charge_enable_pin, OUTPUT);
  pinMode(dcdc_enable_pin, OUTPUT);

  DEBUG_SERIAL.println("PCS: Calling disable_all()...");
  // Initialize to safe state (all disabled)
  disable_all();

  // Initialize hardware timer for message timing (1ms tick)
  // Use TIM6 (basic timer)
  message_timer = new HardwareTimer(TIM6);
  message_timer->setOverflow(1000, HERTZ_FORMAT); // 1kHz = 1ms period
  message_timer->attachInterrupt(timer_callback);
  message_timer->resume();
  DEBUG_SERIAL.println("PCS: Hardware timer started (1ms ticks)");

  DEBUG_SERIAL.println("PCS: Initializing CAN...");
  // Initialize low-level CAN communication
  PCSCan::begin(ipc_can_bus);

  DEBUG_SERIAL.println("PCS: Creating command queue...");
  // Create command queue (10 commands deep)
  command_queue = xQueueCreate(10, sizeof(PCSCommand));
  if (command_queue == nullptr) {
    DEBUG_SERIAL.println("PCS: Failed to create command queue");
    return;
  }

  current_state = PCS_STATE_OFF;

  // IPC CAN messages are gated behind state machine progress (matches old firmware)
  // CAN is enabled during PRECHARGE state after pcs_pin_enabled
  // Note: OpenInverter SDO on CAN3 is separate and always available
  can_enabled = false;
  DEBUG_SERIAL.println("PCS: IPC CAN communication disabled (will enable during precharge)");

  DEBUG_SERIAL.println("PCS: Controller initialized with precharge state machine");
}

bool PCSController::start_task() {
  BaseType_t result = xTaskCreate(
    task_wrapper,
    "PCS",
    2048,  // Stack size
    NULL,
    2,     // Priority (medium)
    NULL
  );

  if (result != pdPASS) {
    DEBUG_SERIAL.println("PCS: Failed to create task!");
    return false;
  }

  DEBUG_SERIAL.println("PCS: Task started");
  return true;
}

void PCSController::update() {
  // Process any pending commands from the queue
  process_command_queue();

  // Process incoming CAN messages
  PCSCan::process_messages();

  // Send periodic CAN messages based on hardware timer flags
  if (flag_10ms) {
    flag_10ms = false;
    send_10ms_messages();
  }

  if (flag_50ms) {
    flag_50ms = false;
    send_50ms_messages();
  }

  if (flag_100ms) {
    flag_100ms = false;
    send_100ms_messages();
    run_state_machine();
  }

  // Update physical control pins
  update_control_pins();
}

void PCSController::timer_callback() {
  // Called every 1ms by hardware timer
  timer_ticks++;

  // Set flags at appropriate intervals
  if (timer_ticks % 10 == 0) {
    flag_10ms = true;
  }

  if (timer_ticks % 50 == 0) {
    flag_50ms = true;
  }

  if (timer_ticks % 100 == 0) {
    flag_100ms = true;
  }
}

void PCSController::process_command_queue() {
  PCSCommand cmd;

  // Process all pending commands (non-blocking)
  while (xQueueReceive(command_queue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case PCS_CMD_SET_CHARGE_POWER:
        set_charge_power(cmd.value_u16);
        break;

      case PCS_CMD_SET_HV_VOLTAGE:
        set_hv_voltage(cmd.value_u16);
        break;

      case PCS_CMD_SET_DCDC_VOLTAGE:
        set_dcdc_voltage(cmd.value_float);
        break;

      case PCS_CMD_SET_AC_LIMIT:
        set_ac_current_limit((uint8_t)cmd.value_u16);
        break;

      case PCS_CMD_ENABLE_PCS:
        enable_pcs(cmd.value_bool);
        break;

      // State machine commands
      case PCS_CMD_START_CHARGING:
        start_charging();
        break;

      case PCS_CMD_START_DRIVE_MODE:
        start_drive_mode();
        break;

      case PCS_CMD_STOP:
        stop();
        break;

      case PCS_CMD_EMERGENCY_STOP:
        emergency_stop();
        break;
    }
  }
}

// Send critical heartbeat messages immediately when CAN is first enabled
// This prevents MIA errors by satisfying PCS heartbeat requirements before timer ticks
void PCSController::send_initial_messages() {
  Serial.println("PCS: Sending initial heartbeat messages");

  // Match old firmware message ordering - single call per message type
  // Mux variants alternate on subsequent calls
  PCSCan::Msg13D();   // Required for PCS operation (post-2020 firmwares)
  PCSCan::Msg22A();   // Main control (mode and voltage)
  PCSCan::Msg3B2();   // BMS log - prevents bmsMia
  PCSCan::Msg545();   // VCFront - prevents vcfrontMia
  PCSCan::Msg333();   // UI watchdog - prevents uiMia
}

// Send messages at 10ms intervals (matches old firmware Ms10Task)
void PCSController::send_10ms_messages() {
  // IPC heartbeat messages must be sent whenever CAN is enabled
  // These keep PCS responsive for communication even when not actively charging
  if (!can_enabled) {
    return;
  }

  // Order matches old firmware: Msg13D → Msg22A → Msg3B2
  PCSCan::Msg13D();   // Required for PCS operation (post-2020 firmwares)
  PCSCan::Msg22A();   // Main control (mode and voltage)
  PCSCan::Msg3B2();   // BMS log (IPC heartbeat)
}

// Send messages at 50ms intervals (matches old firmware Ms50Task)
void PCSController::send_50ms_messages() {
  // IPC heartbeat messages (0x545) must be sent whenever CAN is enabled
  // These keep PCS responsive for SDO reads even when not actively charging
  if (!can_enabled) {
    return;
  }

  PCSCan::Msg545();   // VCFront (IPC heartbeat)
}

// Send messages at 100ms intervals (matches old firmware Ms100Task)
void PCSController::send_100ms_messages() {
  // Send CAN messages whenever CAN is enabled (not just when PCS pin is enabled)
  // This keeps PCS responsive and allows it to provide status information
  if (!can_enabled) {
    return;
  }

  PCSCan::Msg20A();   // Configuration
  PCSCan::Msg212();   // Configuration
  PCSCan::Msg21D();   // Configuration
  PCSCan::Msg232();   // Configuration
  PCSCan::Msg23D();   // AC current limit (alternate)
  PCSCan::Msg25D();   // Configuration
  
  // Send actual charge power request (matches old firmware ChgPwrRamp())
  uint16_t charge_power = PCSCan::get_control_params().charge_power_w;
  PCSCan::Msg2B2(charge_power);
  
  PCSCan::Msg321();   // Configuration
  PCSCan::Msg333();   // UI watchdog
  PCSCan::Msg3A1();   // DCDC setpoint
}

void PCSController::update_control_pins() {
  // pcs_enable_pin is active HIGH (HIGH = enabled)
  digitalWrite(pcs_enable_pin, pcs_pin_enabled ? HIGH : LOW);

  // charge_enable_pin and dcdc_enable_pin are active LOW (LOW = enabled)
  digitalWrite(charge_enable_pin, charge_pin_enabled ? LOW : HIGH);
  digitalWrite(dcdc_enable_pin, dcdc_pin_enabled ? LOW : HIGH);
}

// Direct Control Methods

void PCSController::set_charge_power(uint16_t power_w) {
  PCSCan::set_charge_power(power_w);
}

void PCSController::set_hv_voltage(uint16_t voltage_v) {
  PCSCan::set_hv_voltage(voltage_v);
}

void PCSController::set_dcdc_voltage(float voltage_v) {
  PCSCan::set_dcdc_voltage(voltage_v);
}

void PCSController::set_ac_current_limit(uint8_t limit_a) {
  PCSCan::set_ac_current_limit(limit_a);
}

void PCSController::set_evse_limit(uint8_t limit_a) {
  PCSCan::set_evse_limit(limit_a);
}

void PCSController::set_cable_limit(uint8_t limit) {
  PCSCan::set_cable_limit(limit);
}

void PCSController::enable_pcs(bool enable) {
  pcs_pin_enabled = enable;
  manual_override = enable;  // Diagnostic mode - prevent state machine from overriding
  Serial.printf("PCS: Master enable %s (manual_override=%s)\r\n",
    enable ? "ON" : "OFF", manual_override ? "true" : "false");
}

// Async Control Methods (Queue-based)

bool PCSController::set_charge_power_async(uint16_t power_w) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_CHARGE_POWER;
  cmd.value_u16 = power_w;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_hv_voltage_async(uint16_t voltage_v) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_HV_VOLTAGE;
  cmd.value_u16 = voltage_v;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_dcdc_voltage_async(float voltage_v) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_DCDC_VOLTAGE;
  cmd.value_float = voltage_v;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_ac_current_limit_async(uint8_t limit_a) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_AC_LIMIT;
  cmd.value_u16 = limit_a;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::enable_pcs_async(bool enable) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_ENABLE_PCS;
  cmd.value_bool = enable;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

// FreeRTOS Task Functions

void PCSController::task_loop() {
  while (true) {
    // Call the main update function
    update();

    // Run at 100Hz (10ms cycle)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void PCSController::task_wrapper(void *pvParameters) {
  task_loop();
}

// State Machine Methods

void PCSController::disable_all() {
  pcs_pin_enabled = false;
  charge_pin_enabled = false;
  dcdc_pin_enabled = false;
  can_enabled = false;  // Disable IPC CAN (matches old firmware DisableAll())
  // Note: OpenInverter SDO on CAN3 remains available for parameter access

  // Set all pins to disabled state
  // pcs_enable_pin: active high, so LOW = disabled
  // charge_enable_pin & dcdc_enable_pin: active low, so HIGH = disabled
  digitalWrite(pcs_enable_pin, LOW);
  digitalWrite(charge_enable_pin, HIGH);
  digitalWrite(dcdc_enable_pin, HIGH);

  PCSCan::set_mode(PCS_MODE_OFF);
  PCSCan::set_charge_enable(false);
}

void PCSController::start_charging() {
  if (current_state == PCS_STATE_OFF) {
    Serial.println("PCS: Starting charging sequence");
    target_state = PCS_STATE_CHARGING;
    current_state = PCS_STATE_WAITSTART;
  } else {
    DEBUG_SERIAL.printf("PCS: Cannot start charging from state %d\r\n", current_state);
  }
}

void PCSController::start_drive_mode() {
  if (current_state == PCS_STATE_OFF) {
    Serial.println("PCS: Starting drive mode sequence (DCDC only)");
    target_state = PCS_STATE_DRIVE;
    current_state = PCS_STATE_WAITSTART;
  } else {
    Serial.printf("PCS: Cannot start drive mode from state %d\r\n", current_state);
  }
}

void PCSController::stop() {
  Serial.println("PCS: Stopping (graceful shutdown)");
  current_state = PCS_STATE_STOP;
}

void PCSController::emergency_stop() {
  Serial.println("PCS: EMERGENCY STOP");
  disable_all();
  current_state = PCS_STATE_OFF;
}

// Async state machine control methods (thread-safe)

bool PCSController::start_charging_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_START_CHARGING;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::start_drive_mode_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_START_DRIVE_MODE;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::stop_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_STOP;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::emergency_stop_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_EMERGENCY_STOP;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

void PCSController::run_state_machine() {
  // Get charger status from CAN
  uint8_t pcs_charge_status = PCSCan::get_charger_status().status;

  #if DEBUG_PCS_STATE
  static PCSState last_logged_state = PCS_STATE_INIT;
  static uint32_t last_state_log = 0;

  // Log state changes immediately
  if (current_state != last_logged_state) {
    DEBUG_SERIAL.printf("PCS State: %d -> %d (status=%d)\r\n",
                        last_logged_state, current_state, pcs_charge_status);
    last_logged_state = current_state;
    last_state_log = millis();
  }
  // Periodic state logging (every 5 seconds)
  else if (millis() - last_state_log > 5000) {
    const VoltageData& v = PCSCan::get_voltage_data();
    const ACStatus& ac = PCSCan::get_ac_status();
    const DCCurrentData& dc = PCSCan::get_dc_current_data();
    const DCDCStatus& dcdc = PCSCan::get_dcdc_status();
    const TemperatureData& temp = PCSCan::get_temperature_data();
    const ChargerStatus& charger = PCSCan::get_charger_status();
    
    DEBUG_SERIAL.printf("PCS: state=%d status=%d mode=%d\r\n", 
                        current_state, pcs_charge_status, PCSCan::get_mode());
    DEBUG_SERIAL.printf("  HV: %.1fV  LV: %.1fV\r\n", 
                        v.hv_v, v.lv_v);
    DEBUG_SERIAL.printf("  AC: %.1fA %.1fV %.2fkW limit=%dA\r\n",
                        ac.current_a, ac.voltage_v, ac.power_kw, ac.current_limit_a);
    DEBUG_SERIAL.printf("  DC: %.1fA (A:%.1f B:%.1f C:%.1f)\r\n",
                        dc.total_a, dc.phase_a_a, dc.phase_b_a, dc.phase_c_a);
    DEBUG_SERIAL.printf("  DCDC: %.1fA %.1fW\r\n",
                        dcdc.current_a, dcdc.power_w);
    DEBUG_SERIAL.printf("  Temp: %dC (DCDC_B: %.1fC)\r\n",
                        temp.local_c, temp.dcdc_b_c);
    DEBUG_SERIAL.printf("  Charger: hw=%d avail=%.1fkW\r\n",
                        charger.hw_type, charger.power_available_kw);
    last_state_log = millis();
  }
  #endif

  switch (current_state) {
    case PCS_STATE_INIT:
      // Initialization complete, go to OFF state
      current_state = PCS_STATE_OFF;
      break;

    case PCS_STATE_OFF:
      // Safe state - everything disabled
      disable_all();
      precharge_timer = PRECHARGE_TIME_TICKS;  // Reset precharge timer
      pcs_wakeup_timer = PCS_WAKEUP_TICKS;     // Reset PCS wakeup timer
      powerdown_timer = POWERDOWN_TIME_TICKS;  // Reset powerdown timer
      // Transition happens via start_charging() call
      break;

    case PCS_STATE_WAITSTART:
      // Optional delay state before precharge
      // For now, immediately transition to precharge
      // TODO: Add configurable delay if needed
      current_state = PCS_STATE_PRECHARGE;
      Serial.println("PCS: Entering precharge state");
      break;

    case PCS_STATE_PRECHARGE:
      // Internal PCS initialization (matches old firmware ENABLE state)
      // Note: Actual HV precharge is controlled by BMS via contactors

      // Step 1: Enable PCS hardware first
      if (!pcs_pin_enabled) {
        pcs_pin_enabled = true;
        Serial.println("PCS: Enabling PCS hardware");
      }

      // Step 2: Short delay for PCS to power up before enabling CAN
      // PCS needs brief time to be ready to ACK CAN messages
      if (!can_enabled && pcs_wakeup_timer > 0) {
        pcs_wakeup_timer--;
        break;  // Wait for wakeup delay
      }

      // Step 3: Enable CAN and send initial heartbeats
      if (!can_enabled) {
        can_enabled = true;
        Serial.println("PCS: Enabling IPC CAN after wakeup delay");
        // Send critical heartbeat messages immediately to prevent MIA errors
        send_initial_messages();
      }

      // Decrement precharge timer (runs at 100ms rate)
      if (precharge_timer > 0) {
        precharge_timer--;

        if (precharge_timer == 0) {
          Serial.println("PCS: Precharge complete, transitioning to activate");
          current_state = PCS_STATE_ACTIVATE;
        }
      }
      break;

    case PCS_STATE_ACTIVATE:
      // Enable charge and/or DCDC based on target state
      if (target_state == PCS_STATE_CHARGING) {
        charge_pin_enabled = true;
        dcdc_pin_enabled = false;  // TODO: Could enable both for charge+DCDC mode
        PCSCan::set_mode(PCS_MODE_CHARGE_ONLY);
        PCSCan::set_charge_enable(true);
        Serial.println("PCS: Transitioning to charging state");
      } else if (target_state == PCS_STATE_DRIVE) {
        charge_pin_enabled = false;
        dcdc_pin_enabled = true;
        PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
        PCSCan::set_charge_enable(false);
        Serial.println("PCS: Transitioning to drive mode");
      }

      current_state = target_state;
      break;

    case PCS_STATE_CHARGING:
      // Active charging state
      // Wait for PCS to enter "wait for AC" state (status == 3) before enabling EVSE
      // This is handled by external EVSE controller

      // TODO: Add fault detection and voltage monitoring
      // if (voltage_too_high || timeout || unplugged) {
      //   current_state = PCS_STATE_STOP;
      // }
      break;

    case PCS_STATE_DRIVE:
      // Drive mode - DCDC only, no charging (matches old firmware DRIVE state)
      charge_pin_enabled = false;
      dcdc_pin_enabled = true;
      pcs_pin_enabled = true;
      can_enabled = true;  // Ensure CAN is enabled in drive mode

      PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
      PCSCan::set_charge_enable(false);
      break;

    case PCS_STATE_STOP:
      // Graceful shutdown - ramp down power before disabling
      // Set charge power to zero
      PCSCan::set_charge_power(0);

      if (powerdown_timer > 0) {
        powerdown_timer--;

        if (powerdown_timer == 0) {
          Serial.println("PCS: Powerdown complete, returning to OFF state");
          disable_all();
          current_state = PCS_STATE_OFF;
        }
      }
      break;

    case PCS_STATE_FAULT:
      // Fault state - disable everything
      disable_all();
      // Recovery logic could be added here
      break;

    default:
      // Unknown state - go to safe OFF state
      Serial.printf("PCS: Unknown state %d, going to OFF\r\n", current_state);
      current_state = PCS_STATE_OFF;
      break;
  }
}
