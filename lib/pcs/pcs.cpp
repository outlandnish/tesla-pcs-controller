/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 *
 * High-level controller that manages the PCS hardware pins and coordinates
 * with the low-level PCSCan communication layer.
 *
 * PCSController owns all state. PCSCan pushes parsed data up via update_*() methods.
 */

#include "pcs.h"
#include "pcs-can.h"
#include "debug_serial.h"
#include "errormessage.h"
#include "param_prj.h"

#ifndef DEBUG_PCS_STATE
#define DEBUG_PCS_STATE 1
#endif

// ==================== STATIC MEMBER DEFINITIONS ====================

// Core status from CAN messages
ChargerStatus PCSController::charger_status_ = {
  .main_state = PCS_STATUS_INIT,
  .charge_status = CHARGE_STATUS_IDLE,
  .grid_config = PCS_GRID_SNA,
  .phase_a_enabled = false,
  .phase_b_enabled = false,
  .phase_c_enabled = false,
  .instant_power_available_kw = 0.0f,
  .max_power_available_kw = 0.0f,
  .phase_a_current_request_a = 0.0f,
  .phase_b_current_request_a = 0.0f,
  .phase_c_current_request_a = 0.0f,
  .pwm_enable = false
};

DCDCStatus PCSController::dcdc_status_ = {
  .current_a = 0.0f,
  .power_w = 0.0f,
  .precharge_status = DCDC_STATUS_IDLE,
  .support_12v_status = DCDC_STATUS_IDLE,
  .hvbus_discharge_status = DCDC_STATUS_IDLE,
  .main_state = DCDC_STATE_STANDBY,
  .sub_state = 0,
  .faulted = false,
  .output_limited = false,
  .max_output_current_a = 0.0f,
  .pwm_enable = false,
  .supporting_fixed_lv = false
};

DCDCBusStatus PCSController::dcdc_bus_status_ = {
  .lv_bus_voltage_v = 0.0f,
  .hv_bus_voltage_v = 0.0f,
  .lv_output_current_a = 0.0f
};

ACStatus PCSController::ac_status_ = {
  .current_limit_a = 0,
  .power_kw = 0.0f,
  .voltage_v = 0,
  .current_a = 0.0f
};

TemperatureData PCSController::temperature_data_ = {
  .phase_a_c = 0.0f,
  .phase_b_c = 0.0f,
  .phase_c_c = 0.0f,
  .dcdc_c = 0.0f,
  .ambient_c = 0.0f
};

VoltageData PCSController::voltage_data_ = {
  .hv_v = 0,
  .lv_v = 0.0f
};

DCCurrentData PCSController::dc_current_data_ = {
  .phase_a_a = 0.0f,
  .phase_b_a = 0.0f,
  .phase_c_a = 0.0f,
  .total_a = 0.0f
};

ControlParams PCSController::control_params_ = {
  .hv_voltage_v = UDCSPNT_DEFAULT,
  .charge_power_w = 0,
  .dcdc_voltage_v = UDCDC_DEFAULT,
  .ac_current_limit_a = IACLIM_DEFAULT,
  .evse_limit_a = 0,
  .cable_limit = 0,
  .charge_termination_percent = CHGTERMN_DEFAULT
};

// Logging data from 0x2C4 (PCS_logging)
PhaseLoggingData PCSController::phase_a_logging_ = {};
PhaseLoggingData PCSController::phase_b_logging_ = {};
PhaseLoggingData PCSController::phase_c_logging_ = {};
ChargerLineVoltageData PCSController::charger_line_voltage_ = {};
ChargerFrequencyData PCSController::charger_frequency_ = {};
ChargerPhaseStateData PCSController::charger_phase_state_ = {};
DCDCLoggingData PCSController::dcdc_logging_ = {};
DCDCControlData PCSController::dcdc_control_ = {};

// Alert matrix from 0x3A4 (PCS_alertMatrix)
AlertMatrixState PCSController::alert_matrix_ = {};

// PCS info from 0x3C4 (PCS_info)
PCSInfoData PCSController::pcs_info_ = {};

// Operating mode
PCSMode PCSController::current_mode_ = PCS_MODE_OFF;

// Hardware control
uint8_t PCSController::pcs_enable_pin_ = 0;
uint8_t PCSController::charge_enable_pin_ = 0;
uint8_t PCSController::dcdc_enable_pin_ = 0;

bool PCSController::pcs_pin_enabled_ = false;
bool PCSController::charge_pin_enabled_ = false;
bool PCSController::dcdc_pin_enabled_ = false;
bool PCSController::can_enabled_ = false;
bool PCSController::manual_override_ = false;

// State machine
PCSState PCSController::current_state_ = PCS_STATE_INIT;
PCSState PCSController::target_state_ = PCS_STATE_OFF;
uint8_t PCSController::precharge_timer_ = 0;
uint8_t PCSController::pcs_wakeup_timer_ = 0;
uint8_t PCSController::powerdown_timer_ = 0;

// Timing
HardwareTimer *PCSController::message_timer_ = nullptr;
volatile bool PCSController::flag_10ms_ = false;
volatile bool PCSController::flag_20ms_ = false;
volatile bool PCSController::flag_50ms_ = false;
volatile bool PCSController::flag_100ms_ = false;
volatile bool PCSController::flag_500ms_ = false;
volatile uint16_t PCSController::timer_ticks_ = 0;

// Task / Queue
QueueHandle_t PCSController::command_queue_ = nullptr;

// ==================== INITIALIZATION ====================

void PCSController::begin(CANBus *ipc_can_bus, uint8_t pcs_en_pin, uint8_t charge_en_pin, uint8_t dcdc_en_pin) {
  DEBUG_SERIAL.println("PCS: Configuring pins...");

  // Store pin assignments
  pcs_enable_pin_ = pcs_en_pin;
  charge_enable_pin_ = charge_en_pin;
  dcdc_enable_pin_ = dcdc_en_pin;

  // Configure pins as outputs
  pinMode(pcs_enable_pin_, OUTPUT);
  pinMode(charge_enable_pin_, OUTPUT);
  pinMode(dcdc_enable_pin_, OUTPUT);

  DEBUG_SERIAL.println("PCS: Calling disable_all()...");
  disable_all();

  // Initialize hardware timer for message timing (1ms tick)
  message_timer_ = new HardwareTimer(TIM6);
  message_timer_->setOverflow(1000, HERTZ_FORMAT);
  message_timer_->attachInterrupt(timer_callback);
  message_timer_->resume();
  DEBUG_SERIAL.println("PCS: Hardware timer started (1ms ticks)");

  DEBUG_SERIAL.println("PCS: Initializing CAN...");
  PCSCan::begin(ipc_can_bus);

  DEBUG_SERIAL.println("PCS: Creating command queue...");
  command_queue_ = xQueueCreate(10, sizeof(PCSCommand));
  if (command_queue_ == nullptr) {
    DEBUG_SERIAL.println("PCS: Failed to create command queue");
    return;
  }

  current_state_ = PCS_STATE_OFF;
  can_enabled_ = false;
  DEBUG_SERIAL.println("PCS: IPC CAN communication disabled (will enable during precharge)");
  DEBUG_SERIAL.println("PCS: Controller initialized with precharge state machine");
}

bool PCSController::start_task() {
  BaseType_t result = xTaskCreate(
    task_wrapper,
    "PCS",
    2048,
    NULL,
    2,
    NULL
  );

  if (result != pdPASS) {
    DEBUG_SERIAL.println("PCS: Failed to create task!");
    return false;
  }

  DEBUG_SERIAL.println("PCS: Task started");
  return true;
}

// ==================== UPDATE LOOP ====================

void PCSController::update() {
  process_command_queue();
  PCSCan::process_messages();

  if (flag_10ms_) {
    flag_10ms_ = false;
    send_10ms_messages();
  }

  if (flag_20ms_) {
    flag_20ms_ = false;
    send_20ms_messages();
  }

  if (flag_50ms_) {
    flag_50ms_ = false;
    send_50ms_messages();
  }

  if (flag_100ms_) {
    flag_100ms_ = false;
    send_100ms_messages();
    run_state_machine();
  }

  if (flag_500ms_) {
    flag_500ms_ = false;
    send_500ms_messages();
  }

  update_control_pins();
}

void PCSController::timer_callback() {
  timer_ticks_++;

  if (timer_ticks_ % 10 == 0) {
    flag_10ms_ = true;
  }

  if (timer_ticks_ % 20 == 0) {
    flag_20ms_ = true;
  }

  if (timer_ticks_ % 50 == 0) {
    flag_50ms_ = true;
  }

  if (timer_ticks_ % 100 == 0) {
    flag_100ms_ = true;
  }

  if (timer_ticks_ % 500 == 0) {
    flag_500ms_ = true;
  }
}

// ==================== STATE UPDATE METHODS (called by PCSCan) ====================

void PCSController::update_dcdc_current(float current_a, float lv_voltage) {
  dcdc_status_.current_a = current_a;
  dcdc_status_.power_w = current_a * lv_voltage;
}

void PCSController::update_dc_phase_current(uint8_t phase, float current_a) {
  switch (phase) {
    case 0: dc_current_data_.phase_a_a = current_a; break;
    case 1: dc_current_data_.phase_b_a = current_a; break;
    case 2: dc_current_data_.phase_c_a = current_a; break;
  }
  dc_current_data_.total_a = dc_current_data_.phase_a_a +
                              dc_current_data_.phase_b_a +
                              dc_current_data_.phase_c_a;
}

void PCSController::update_phase_logging(uint8_t phase, const PhaseLoggingData& data) {
  switch (phase) {
    case 0:
      phase_a_logging_ = data;
      dc_current_data_.phase_a_a = data.output_current_a;
      break;
    case 1:
      phase_b_logging_ = data;
      dc_current_data_.phase_b_a = data.output_current_a;
      break;
    case 2:
      phase_c_logging_ = data;
      dc_current_data_.phase_c_a = data.output_current_a;
      break;
  }
  dc_current_data_.total_a = dc_current_data_.phase_a_a +
                              dc_current_data_.phase_b_a +
                              dc_current_data_.phase_c_a;
}

void PCSController::update_alert_matrix_page(uint8_t page, const uint8_t* data) {
  uint8_t* stored_page = (page == 0) ? alert_matrix_.page0 : alert_matrix_.page1;
  bool* page_valid = (page == 0) ? &alert_matrix_.page0_valid : &alert_matrix_.page1_valid;

  memcpy(stored_page, data, 8);
  *page_valid = true;

  // Update total active alert count when we have both pages
  if (alert_matrix_.page0_valid && alert_matrix_.page1_valid) {
    alert_matrix_.active_count = count_alert_bits(alert_matrix_.page0) +
                                  count_alert_bits(alert_matrix_.page1);
  }
}

// ==================== ALERT HELPERS ====================

bool PCSController::is_alert_active(uint8_t alert_num) {
  if (alert_num < 1 || alert_num > 120) return false;

  const uint8_t* page;
  int bit_offset;

  if (alert_num <= 60) {
    if (!alert_matrix_.page0_valid) return false;
    page = alert_matrix_.page0;
    bit_offset = alert_num - 1 + 4;  // Alert 1 is at bit 4
  } else {
    if (!alert_matrix_.page1_valid) return false;
    page = alert_matrix_.page1;
    bit_offset = (alert_num - 61) + 4;  // Alert 61 is at bit 4
  }

  int byte_idx = bit_offset / 8;
  int bit_idx = bit_offset % 8;
  return (page[byte_idx] & (1 << bit_idx)) != 0;
}

uint8_t PCSController::count_alert_bits(const uint8_t* bytes) {
  uint8_t count = 0;
  // Count bits 4-7 of byte 0
  for (int i = 4; i < 8; i++) {
    if (bytes[0] & (1 << i)) count++;
  }
  // Count all bits in bytes 1-7
  for (int b = 1; b < 8; b++) {
    uint8_t byte_val = bytes[b];
    while (byte_val) {
      count += byte_val & 1;
      byte_val >>= 1;
    }
  }
  return count;
}

// ==================== CAN TIMING STATISTICS ====================

static uint32_t can_timing_last_print = 0;
static uint32_t can_timing_sum = 0;
static uint16_t can_timing_count = 0;
static uint16_t can_timing_max = 0;
static uint16_t can_timing_min = 0xFFFF;
static uint32_t can_timing_last = 0;

void PCSController::record_can_timing(uint32_t now) {
  if (can_timing_last != 0) {
    uint16_t delta = now - can_timing_last;
    can_timing_sum += delta;
    can_timing_count++;
    if (delta > can_timing_max) can_timing_max = delta;
    if (delta < can_timing_min) can_timing_min = delta;
    if (delta > 10) {
      DEBUG_SERIAL.printf("[CAN TIMING WARNING] delta exceeded 10ms: %u ms at %u ms\r\n", delta, now);
    }
  }
  can_timing_last = now;

  if (now - can_timing_last_print >= 1000 && can_timing_count > 0) {
    uint16_t avg = can_timing_sum / can_timing_count;
    DEBUG_SERIAL.printf("[CAN TIMING] avg: %u ms, min: %u ms, max: %u ms, count: %u\r\n", avg, can_timing_min, can_timing_max, can_timing_count);
    can_timing_sum = 0;
    can_timing_count = 0;
    can_timing_max = 0;
    can_timing_min = 0xFFFF;
    can_timing_last_print = now;
  }
}

// ==================== COMMAND QUEUE PROCESSING ====================

void PCSController::process_command_queue() {
  PCSCommand cmd;

  while (xQueueReceive(command_queue_, &cmd, 0) == pdTRUE) {
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

      case PCS_CMD_SET_CHARGE_TERMINATION:
        set_charge_termination_percent((uint8_t)cmd.value_u16);
        break;

      case PCS_CMD_ENABLE_PCS:
        enable_pcs(cmd.value_bool);
        break;

      case PCS_CMD_CLEAR_FAULTS:
        PCSCan::request_clear_faults();
        break;

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

      case PCS_CMD_SET_MANUAL_MODE:
        manual_override_ = cmd.value_bool;
        DEBUG_SERIAL.printf("PCS: Manual mode %s\r\n", manual_override_ ? "ENABLED" : "DISABLED");
        if (!manual_override_) {
          DEBUG_SERIAL.println("PCS: Returning to state machine control");
        }
        break;

      case PCS_CMD_MANUAL_PCS_ENABLE:
        if (manual_override_) {
          pcs_pin_enabled_ = cmd.value_bool;
          DEBUG_SERIAL.printf("PCS: Manual PCS enable = %s\r\n", pcs_pin_enabled_ ? "ON" : "OFF");
          if (pcs_pin_enabled_ && !can_enabled_) {
            can_enabled_ = true;
            DEBUG_SERIAL.println("PCS: Auto-enabling CAN (PCS enabled)");
          }
        } else {
          DEBUG_SERIAL.println("PCS: Cannot set PCS enable - manual mode not active");
        }
        break;

      case PCS_CMD_MANUAL_CHARGE_ENABLE:
        if (manual_override_) {
          charge_pin_enabled_ = cmd.value_bool;
          DEBUG_SERIAL.printf("PCS: Manual Charge enable = %s\r\n", charge_pin_enabled_ ? "ON" : "OFF");
        } else {
          DEBUG_SERIAL.println("PCS: Cannot set Charge enable - manual mode not active");
        }
        break;

      case PCS_CMD_MANUAL_DCDC_ENABLE:
        if (manual_override_) {
          dcdc_pin_enabled_ = cmd.value_bool;
          DEBUG_SERIAL.printf("PCS: Manual DCDC enable = %s\r\n", dcdc_pin_enabled_ ? "ON" : "OFF");
        } else {
          DEBUG_SERIAL.println("PCS: Cannot set DCDC enable - manual mode not active");
        }
        break;

      case PCS_CMD_MANUAL_CAN_ENABLE:
        if (manual_override_) {
          can_enabled_ = cmd.value_bool;
          DEBUG_SERIAL.printf("PCS: Manual CAN enable = %s\r\n", can_enabled_ ? "ON" : "OFF");
        } else {
          DEBUG_SERIAL.println("PCS: Cannot set CAN enable - manual mode not active");
        }
        break;
    }
  }
}

void PCSController::send_10ms_messages() {
  if (!can_enabled_) return;
  PCSCan::Msg13D();
  PCSCan::Msg20A();  // HVP_contactorState (DBC: 10ms cycle)
  PCSCan::Msg22A();
  PCSCan::Msg3B2();
}

void PCSController::send_20ms_messages() {
  if (!can_enabled_) return;
  PCSCan::Msg201();   // Unknown message (4-message cycle, ~1400ms total)
  PCSCan::Msg2E1();   // VCFRONT_status (DBC: 20ms) - simulated VCFRONT, reports PCSMia=0 DCDCNoop=0
}

void PCSController::send_50ms_messages() {
  if (!can_enabled_) return;
  PCSCan::Msg221();   // VCFRONT_LVPowerState (DBC: 50ms) - simulated VCFRONT for EV conversion
  PCSCan::Msg545();
  PCSCan::Msg261();   // VCFRONT_12VBatteryStatus (DBC: 100ms) - simulated VCFRONT for EV conversion
  PCSCan::Msg340();   // VCFRONT_alertMatrix (DBC: 100ms) - simulated VCFRONT with DCDC operational
  PCSCan::Msg3A1();   // VCFRONT_vehicleStatus (DBC: 100ms)
}

void PCSController::send_100ms_messages() {
  if (!can_enabled_) return;
  PCSCan::Msg212();   // BMS_status (DBC: 100ms)
  PCSCan::Msg21D();   // CP_evseStatus (DBC: 100ms)
  PCSCan::Msg232();
  PCSCan::Msg23D();
  PCSCan::Msg25D();   // CP_unknown (removing this causes PCS_a023_cpMia)
  PCSCan::Msg2B2(control_params_.charge_power_w);  // BMS_chargerRequest (DBC: 100ms)
  PCSCan::Msg2F1();   // VCFRONT_eFuseDebugStatus (DBC: 100ms)
}

void PCSController::send_500ms_messages() {
  if (!can_enabled_) return;

  static uint8_t ms1000_counter = 0;

  PCSCan::Msg333();   // UI_chargeRequest (DBC: 500ms)

  // Send 1000ms cycle messages every other 500ms call
  if (ms1000_counter == 0) {
    PCSCan::Msg301();   // VCFRONT_info (DBC: 1000ms)
    PCSCan::Msg321();   // VCFRONT_sensors (DBC: 1000ms)
  }
  ms1000_counter = (ms1000_counter + 1) % 2;
}

void PCSController::update_control_pins() {
  digitalWrite(pcs_enable_pin_, pcs_pin_enabled_ ? HIGH : LOW);
  digitalWrite(charge_enable_pin_, charge_pin_enabled_ ? LOW : HIGH);
  digitalWrite(dcdc_enable_pin_, dcdc_pin_enabled_ ? LOW : HIGH);
}

// ==================== DIRECT CONTROL METHODS ====================

void PCSController::set_charge_power(uint16_t power_w) {
  control_params_.charge_power_w = power_w;
  PCSCan::set_charge_power(power_w);
}

void PCSController::set_hv_voltage(uint16_t voltage_v) {
  control_params_.hv_voltage_v = voltage_v;
  PCSCan::set_hv_voltage(voltage_v);
}

void PCSController::set_dcdc_voltage(float voltage_v) {
  control_params_.dcdc_voltage_v = voltage_v;
  PCSCan::set_dcdc_voltage(voltage_v);
}

void PCSController::set_ac_current_limit(uint8_t limit_a) {
  control_params_.ac_current_limit_a = limit_a;
  PCSCan::set_ac_current_limit(limit_a);
}

void PCSController::set_charge_termination_percent(uint8_t percent) {
  control_params_.charge_termination_percent = percent;
  PCSCan::set_charge_termination_percent(percent);
}

void PCSController::set_evse_limit(uint8_t limit_a) {
  control_params_.evse_limit_a = limit_a;
  PCSCan::set_evse_limit(limit_a);
}

void PCSController::set_cable_limit(uint8_t limit) {
  control_params_.cable_limit = limit;
  PCSCan::set_cable_limit(limit);
}

void PCSController::enable_pcs(bool enable) {
  pcs_pin_enabled_ = enable;
  manual_override_ = enable;
  DEBUG_SERIAL.printf("PCS: Master enable %s (manual_override=%s)\r\n",
    enable ? "ON" : "OFF", manual_override_ ? "true" : "false");
}

// ==================== ASYNC CONTROL METHODS ====================

bool PCSController::set_charge_power_async(uint16_t power_w) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_CHARGE_POWER;
  cmd.value_u16 = power_w;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_hv_voltage_async(uint16_t voltage_v) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_HV_VOLTAGE;
  cmd.value_u16 = voltage_v;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_dcdc_voltage_async(float voltage_v) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_DCDC_VOLTAGE;
  cmd.value_float = voltage_v;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_ac_current_limit_async(uint8_t limit_a) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_AC_LIMIT;
  cmd.value_u16 = limit_a;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_charge_termination_percent_async(uint8_t percent) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_CHARGE_TERMINATION;
  cmd.value_u16 = percent;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::enable_pcs_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_ENABLE_PCS;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::clear_faults_async() {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_CLEAR_FAULTS;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_manual_mode_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_MANUAL_MODE;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::manual_pcs_enable_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_MANUAL_PCS_ENABLE;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::manual_charge_enable_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_MANUAL_CHARGE_ENABLE;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::manual_dcdc_enable_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_MANUAL_DCDC_ENABLE;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::manual_can_enable_async(bool enable) {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_MANUAL_CAN_ENABLE;
  cmd.value_bool = enable;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

// ==================== TASK FUNCTIONS ====================

void PCSController::task_loop() {
  while (true) {
    update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void PCSController::task_wrapper(void *pvParameters) {
  task_loop();
}

// ==================== STATE MACHINE ====================

void PCSController::disable_all() {
  pcs_pin_enabled_ = false;
  charge_pin_enabled_ = false;
  dcdc_pin_enabled_ = false;
  can_enabled_ = false;

  digitalWrite(pcs_enable_pin_, LOW);
  digitalWrite(charge_enable_pin_, HIGH);
  digitalWrite(dcdc_enable_pin_, HIGH);

  current_mode_ = PCS_MODE_OFF;
  PCSCan::set_mode(PCS_MODE_OFF);
  PCSCan::set_charge_enable(false);
}

void PCSController::start_charging() {
  if (current_state_ == PCS_STATE_OFF) {
    DEBUG_SERIAL.println("PCS: Starting charging sequence");
    target_state_ = PCS_STATE_CHARGING;
    current_state_ = PCS_STATE_WAITSTART;
  } else {
    DEBUG_SERIAL.printf("PCS: Cannot start charging from state %d\r\n", current_state_);
  }
}

void PCSController::start_drive_mode() {
  if (current_state_ == PCS_STATE_OFF) {
    DEBUG_SERIAL.println("PCS: Starting drive mode sequence (DCDC only)");
    target_state_ = PCS_STATE_DRIVE;
    current_state_ = PCS_STATE_WAITSTART;
  } else {
    DEBUG_SERIAL.printf("PCS: Cannot start drive mode from state %d\r\n", current_state_);
  }
}

void PCSController::stop() {
  DEBUG_SERIAL.println("PCS: Stopping (graceful shutdown)");
  current_state_ = PCS_STATE_STOP;
}

void PCSController::emergency_stop() {
  DEBUG_SERIAL.println("PCS: EMERGENCY STOP");
  disable_all();
  current_state_ = PCS_STATE_OFF;
}

bool PCSController::start_charging_async() {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_START_CHARGING;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::start_drive_mode_async() {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_START_DRIVE_MODE;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::stop_async() {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_STOP;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::emergency_stop_async() {
  if (command_queue_ == nullptr) return false;
  PCSCommand cmd;
  cmd.type = PCS_CMD_EMERGENCY_STOP;
  return xQueueSend(command_queue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

void PCSController::run_state_machine() {
  if (manual_override_) {
    update_control_pins();
    return;
  }

  uint8_t pcs_charge_status = charger_status_.main_state;

  #if DEBUG_PCS_STATE
  static PCSState last_logged_state = PCS_STATE_INIT;
  static uint32_t last_state_log = 0;

  if (current_state_ != last_logged_state) {
    DEBUG_SERIAL.printf("PCS State: %d -> %d (chg=%s)\r\n",
                        last_logged_state, current_state_,
                        charge_state_str((PCSChargeStatus)pcs_charge_status));
    last_logged_state = current_state_;
    last_state_log = millis();
  }
  else if (millis() - last_state_log > 5000) {
    DEBUG_SERIAL.printf("PCS: state=%d chg=%s mode=%d\r\n",
                        current_state_,
                        charge_state_str((PCSChargeStatus)pcs_charge_status),
                        current_mode_);
    DEBUG_SERIAL.printf("  HV: %.1fV  LV: %.1fV\r\n",
                        (float)voltage_data_.hv_v, voltage_data_.lv_v);
    DEBUG_SERIAL.printf("  AC: %.1fA %.1fV %.2fkW limit=%dA\r\n",
                        ac_status_.current_a, (float)ac_status_.voltage_v, ac_status_.power_kw, ac_status_.current_limit_a);
    DEBUG_SERIAL.printf("  DC: %.1fA (A:%.1f B:%.1f C:%.1f)\r\n",
                        dc_current_data_.total_a, dc_current_data_.phase_a_a, dc_current_data_.phase_b_a, dc_current_data_.phase_c_a);
    DEBUG_SERIAL.printf("  DCDC: %.1fA %.1fW state=%s sub=%d fault=%d\r\n",
                        dcdc_status_.current_a, dcdc_status_.power_w,
                        dcdc_state_str(dcdc_status_.main_state), dcdc_status_.sub_state, dcdc_status_.faulted);
    DEBUG_SERIAL.printf("    Prech=%d 12V=%d Disch=%d Lim=%d(%.1fA) PWM=%d\r\n",
                        dcdc_status_.precharge_status, dcdc_status_.support_12v_status, dcdc_status_.hvbus_discharge_status,
                        dcdc_status_.output_limited, dcdc_status_.max_output_current_a, dcdc_status_.pwm_enable);
    DEBUG_SERIAL.printf("  DCDC Bus (0x2B4): HV=%.1fV LV=%.1fV I=%.1fA\r\n",
                        dcdc_bus_status_.hv_bus_voltage_v, dcdc_bus_status_.lv_bus_voltage_v, dcdc_bus_status_.lv_output_current_a);
    constexpr float TEMP_INVALID = -999.0f;
    DEBUG_SERIAL.printf("  Temp: PhA=%s PhB=%s PhC=%s DCDC=%s Amb=%s\r\n",
      (temperature_data_.phase_a_c > TEMP_INVALID ? String(temperature_data_.phase_a_c, 1).c_str() : "N/A"),
      (temperature_data_.phase_b_c > TEMP_INVALID ? String(temperature_data_.phase_b_c, 1).c_str() : "N/A"),
      (temperature_data_.phase_c_c > TEMP_INVALID ? String(temperature_data_.phase_c_c, 1).c_str() : "N/A"),
      (temperature_data_.dcdc_c    > TEMP_INVALID ? String(temperature_data_.dcdc_c, 1).c_str()    : "N/A"),
      (temperature_data_.ambient_c > TEMP_INVALID ? String(temperature_data_.ambient_c, 1).c_str() : "N/A"));
    DEBUG_SERIAL.printf("  Charger: state=%s status=%d grid=%s avail=%.1f/%.1fkW PWM=%d\r\n",
                        charge_state_str(charger_status_.main_state),
                        charger_status_.charge_status,
                        grid_config_str(charger_status_.grid_config),
                        charger_status_.instant_power_available_kw, charger_status_.max_power_available_kw, charger_status_.pwm_enable);
    DEBUG_SERIAL.printf("    Phases: A=%d(%.1fA) B=%d(%.1fA) C=%d(%.1fA)\r\n",
                        charger_status_.phase_a_enabled, charger_status_.phase_a_current_request_a,
                        charger_status_.phase_b_enabled, charger_status_.phase_b_current_request_a,
                        charger_status_.phase_c_enabled, charger_status_.phase_c_current_request_a);

    if (alert_matrix_.active_count > 0) {
      DEBUG_SERIAL.printf("  Alerts (%d): [", alert_matrix_.active_count);
      bool first = true;
      for (uint8_t i = 1; i <= 120; i++) {
        if (is_alert_active(i)) {
          if (!first) DEBUG_SERIAL.print("\n");
          DEBUG_SERIAL.printf("a%03d: %s", i, get_alert_name(i));
          first = false;
        }
      }
      DEBUG_SERIAL.println("]");
    } else {
      DEBUG_SERIAL.println("  Alerts: none");
    }
    last_state_log = millis();
  }
  #endif

  switch (current_state_) {
    case PCS_STATE_INIT:
      current_state_ = PCS_STATE_OFF;
      break;

    case PCS_STATE_OFF:
      disable_all();
      precharge_timer_ = PRECHARGE_TIME_TICKS;
      pcs_wakeup_timer_ = PCS_WAKEUP_TICKS;
      powerdown_timer_ = POWERDOWN_TIME_TICKS;
      break;

    case PCS_STATE_WAITSTART:
      current_state_ = PCS_STATE_PRECHARGE;
      DEBUG_SERIAL.println("PCS: Entering precharge state");
      break;

    case PCS_STATE_PRECHARGE:
      if (!can_enabled_) {
        can_enabled_ = true;
        DEBUG_SERIAL.println("PCS: Starting CAN heartbeats");
      }

      if (!pcs_pin_enabled_) {
        pcs_pin_enabled_ = true;
        DEBUG_SERIAL.println("PCS: Enabling PCS hardware");
      }

      if (precharge_timer_ > 0) {
        precharge_timer_--;
        if (precharge_timer_ == 0) {
          DEBUG_SERIAL.println("PCS: Precharge complete, transitioning to activate");
          current_state_ = PCS_STATE_ACTIVATE;
        }
      }
      break;

    case PCS_STATE_ACTIVATE:
      print_pcs_info();

      if (target_state_ == PCS_STATE_CHARGING) {
        charge_pin_enabled_ = true;
        dcdc_pin_enabled_ = false;
        current_mode_ = PCS_MODE_CHARGE_ONLY;
        PCSCan::set_mode(PCS_MODE_CHARGE_ONLY);
        PCSCan::set_charge_enable(true);
        DEBUG_SERIAL.println("PCS: Transitioning to charging state");
      } else if (target_state_ == PCS_STATE_DRIVE) {
        charge_pin_enabled_ = false;
        dcdc_pin_enabled_ = true;
        current_mode_ = PCS_MODE_DCDC_ONLY;
        PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
        PCSCan::set_charge_enable(false);
        DEBUG_SERIAL.println("PCS: Transitioning to drive mode");
      }

      current_state_ = target_state_;
      break;

    case PCS_STATE_CHARGING:
      print_pcs_info();
      break;

    case PCS_STATE_DRIVE:
      print_pcs_info();
      charge_pin_enabled_ = false;
      dcdc_pin_enabled_ = true;
      pcs_pin_enabled_ = true;
      can_enabled_ = true;
      current_mode_ = PCS_MODE_DCDC_ONLY;
      PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
      PCSCan::set_charge_enable(false);
      break;

    case PCS_STATE_STOP:
      PCSCan::set_charge_power(0);

      if (powerdown_timer_ > 0) {
        powerdown_timer_--;
        if (powerdown_timer_ == 0) {
          DEBUG_SERIAL.println("PCS: Powerdown complete, returning to OFF state");
          disable_all();
          current_state_ = PCS_STATE_OFF;
        }
      }
      break;

    case PCS_STATE_FAULT:
      disable_all();
      break;

    default:
      DEBUG_SERIAL.printf("PCS: Unknown state %d, going to OFF\r\n", current_state_);
      current_state_ = PCS_STATE_OFF;
      break;
  }
}

// ==================== DEBUG / PRINT HELPERS ====================

const char* PCSController::get_alert_name(uint8_t alert_num) {
  if (alert_num >= 1 && alert_num <= 120) {
    return errorDescriptors[alert_num].msg;
  }
  return "Unknown alert";
}

const char* PCSController::charge_state_str(PCSChargeStatus state) {
  switch (state) {
    case PCS_STATUS_INIT:          return "INIT";
    case PCS_STATUS_IDLE:          return "IDLE";
    case PCS_STATUS_STARTUP:       return "STARTUP";
    case PCS_STATUS_WAIT_LINE:     return "WAIT_LINE";
    case PCS_STATUS_QUALIFY:       return "QUALIFY";
    case PCS_STATUS_SYSTEM_CONFIG: return "SYS_CFG";
    case PCS_STATUS_ENABLE:        return "ENABLE";
    case PCS_STATUS_SHUTDOWN:      return "SHUTDOWN";
    case PCS_STATUS_FAULTED:       return "FAULTED";
    case PCS_STATUS_CLEAR_FAULTS:  return "CLR_FAULT";
    default:                       return "UNKNOWN";
  }
}

const char* PCSController::dcdc_state_str(DCDCMainState state) {
  switch (state) {
    case DCDC_STATE_STANDBY:            return "STANDBY";
    case DCDC_STATE_12V_SUPPORT_ACTIVE: return "12V_ACTIVE";
    case DCDC_STATE_PRECHARGE_STARTUP:  return "PRECH_START";
    case DCDC_STATE_PRECHARGE_ACTIVE:   return "PRECH_ACTIVE";
    case DCDC_STATE_DIS_HVBUS_ACTIVE:   return "DISCHG_HV";
    case DCDC_STATE_SHUTDOWN:           return "SHUTDOWN";
    case DCDC_STATE_FAULTED:            return "FAULTED";
    default:                            return "UNKNOWN";
  }
}

const char* PCSController::grid_config_str(PCSGridConfig config) {
  switch (config) {
    case PCS_GRID_SNA:              return "N/A";
    case PCS_GRID_SINGLE_PHASE:     return "1P";
    case PCS_GRID_THREE_PHASE:      return "3P";
    case PCS_GRID_THREE_PHASE_DELTA: return "3P_DELTA";
    default:                        return "UNKNOWN";
  }
}

void PCSController::print_pcs_info() {
  if (!pcs_info_.info_valid) return;

  DEBUG_SERIAL.println("----------------------------------------");
  DEBUG_SERIAL.println("PCS Hardware Information:");
  DEBUG_SERIAL.printf("  Part Number:    %s\r\n", pcs_info_.part_number[0] ? pcs_info_.part_number : "(not received)");
  DEBUG_SERIAL.printf("  Hardware ID:    0x%04X\r\n", pcs_info_.hardware_id);
  DEBUG_SERIAL.printf("  Component ID:   0x%04X\r\n", pcs_info_.component_id);
  DEBUG_SERIAL.printf("  Build Type:     %d\r\n", pcs_info_.build_type);
  DEBUG_SERIAL.printf("  Build Config:   0x%04X\r\n", pcs_info_.build_config_id);
  DEBUG_SERIAL.printf("  PCBA ID:        0x%02X\r\n", pcs_info_.pcba_id);
  DEBUG_SERIAL.printf("  Assembly ID:    0x%02X\r\n", pcs_info_.assembly_id);
  DEBUG_SERIAL.printf("  Usage ID:       0x%04X\r\n", pcs_info_.usage_id);
  DEBUG_SERIAL.printf("  Sub-usage ID:   0x%04X\r\n", pcs_info_.subusage_id);
  DEBUG_SERIAL.printf("  Platform Type:  %d\r\n", pcs_info_.platform_type);
  DEBUG_SERIAL.printf("  App CRC:        0x%08lX\r\n", pcs_info_.app_crc);
  // Print 64-bit git hashes as two 32-bit parts (Arduino printf doesn't support %llX)
  DEBUG_SERIAL.printf("  App Git Hash:   0x%07lX%08lX\r\n",
                      (uint32_t)(pcs_info_.app_git_hash >> 32),
                      (uint32_t)(pcs_info_.app_git_hash & 0xFFFFFFFF));
  DEBUG_SERIAL.printf("  Boot Git Hash:  0x%07lX%08lX\r\n",
                      (uint32_t)(pcs_info_.bootloader_git_hash >> 32),
                      (uint32_t)(pcs_info_.bootloader_git_hash & 0xFFFFFFFF));
  DEBUG_SERIAL.printf("  Boot UDS Proto: %d\r\n", pcs_info_.bootloader_uds_proto_version);
  DEBUG_SERIAL.printf("  Boot CRC:       0x%08X\r\n", pcs_info_.boot_crc);
  DEBUG_SERIAL.println("----------------------------------------");
}

void PCSController::print_dcdc_status(Stream &out) {
  out.println("--- DCDC Status ---");
  out.printf("  Main State: %s (%d)\n", dcdc_state_str(dcdc_status_.main_state), dcdc_status_.main_state);
  out.printf("  Precharge: %d  12V Support: %d  HVBus Discharge: %d\n", dcdc_status_.precharge_status, dcdc_status_.support_12v_status, dcdc_status_.hvbus_discharge_status);
  out.printf("  Faulted: %d  Output Limited: %d\n", dcdc_status_.faulted, dcdc_status_.output_limited);
  out.printf("  Current: %.2f A  Power: %.1f W\n", dcdc_status_.current_a, dcdc_status_.power_w);
  out.printf("  Max Output Current: %.2f A\n", dcdc_status_.max_output_current_a);
  out.printf("  PWM Enable: %d  Fixed LV: %d\n", dcdc_status_.pwm_enable, dcdc_status_.supporting_fixed_lv);
}

void PCSController::print_ac_charge_status(Stream &out) {
  out.println("--- AC/Charge Status ---");
  out.printf("  Main State: %s (%d)\n", charge_state_str(charger_status_.main_state), charger_status_.main_state);
  out.printf("  Charge Status: %d  Grid Config: %s (%d)\n", charger_status_.charge_status, grid_config_str(charger_status_.grid_config), charger_status_.grid_config);
  out.printf("  Phases Enabled: A:%d B:%d C:%d\n", charger_status_.phase_a_enabled, charger_status_.phase_b_enabled, charger_status_.phase_c_enabled);
  out.printf("  Instant Power: %.1f kW  Max Power: %.1f kW\n", charger_status_.instant_power_available_kw, charger_status_.max_power_available_kw);
  out.printf("  Phase Currents Req: A:%.2f B:%.2f C:%.2f\n", charger_status_.phase_a_current_request_a, charger_status_.phase_b_current_request_a, charger_status_.phase_c_current_request_a);
  out.printf("  PWM Enable: %d\n", charger_status_.pwm_enable);
  out.printf("  AC Current Limit: %d A  Power: %.1f kW  Voltage: %d V  Current: %.2f A\n", ac_status_.current_limit_a, ac_status_.power_kw, ac_status_.voltage_v, ac_status_.current_a);
}

void PCSController::print_active_alerts(Stream &out) {
  out.println("--- Active Alerts ---");
  uint8_t count = 0;
  for (uint8_t i = 1; i <= 120; ++i) {
    if (is_alert_active(i)) {
      out.printf("  Alert %d: %s\n", i, get_alert_name(i));
      ++count;
    }
  }
  if (count == 0) {
    out.println("  None active.");
  } else {
    out.printf("Total active: %d\n", count);
  }
}
