#pragma once

/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 *
 * High-level controller that manages the PCS hardware pins and coordinates
 * with the low-level PCSCan communication layer.
 *
 * ARCHITECTURE:
 * -------------
 * PCSController owns all PCS state. PCSCan is a pure transport layer that
 * parses CAN messages and pushes data up via update_*() methods.
 * External code accesses state through PCSController getters only.
 *
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 *
 * STATE MACHINE & BMS COORDINATION:
 * ---------------------------------
 * The PCS owns the precharge circuit and performs the actual HV precharging
 * for ALL HV system operations (charging, drive mode, etc).
 * The BMS coordinates the overall HV system and must monitor PCS state.
 *
 * THREAD SAFETY: All BMS->PCS commands MUST use async methods (queue-based)
 *
 * Typical HV activation sequence (charging):
 *  1. BMS closes negative contactor (HV precharge begins via external circuit)
 *  2. BMS calls PCSController::start_charging_async()
 *  3. PCS enters PCS_STATE_PRECHARGE (internal initialization)
 *  4. PCS waits for initialization timer (~3 seconds for CAN startup, etc)
 *  5. PCS enters PCS_STATE_ACTIVATE, then PCS_STATE_CHARGING
 *  6. BMS polls is_precharge_complete() to verify PCS is ready
 *  7. BMS verifies precharge voltage with IVT shunt
 *  8. BMS closes positive contactor (completes HV connection)
 *  9. BMS transitions to HV_Active state
 * 10. Charging begins
 *
 * BMS coordination methods:
 *  - start_charging_async() - Request precharge + charging mode
 *  - start_drive_mode_async() - Request precharge + DCDC mode
 *  - stop_async() - Request graceful shutdown
 *  - emergency_stop_async() - Request immediate shutdown
 *
 * BMS monitoring methods:
 *  - is_precharging() - PCS is actively precharging
 *  - is_precharge_complete() - PCS ready for HV connection
 *  - is_active() - PCS in active operation (charging or drive)
 *  - is_faulted() - PCS has encountered a fault
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>
#include "../can/can.h"
#include "pcs-can.h"

// PCS state machine states
enum PCSState : uint8_t {
  PCS_STATE_INIT,
  PCS_STATE_OFF,
  PCS_STATE_WAITSTART,
  PCS_STATE_PRECHARGE,      // HV enabled, waiting for precharge timer
  PCS_STATE_ACTIVATE,       // Set charge/DCDC enables
  PCS_STATE_CHARGING,       // Active charging (EVSE activated)
  PCS_STATE_DRIVE,          // Drive mode (DCDC only)
  PCS_STATE_STOP,           // Shutdown sequence
  PCS_STATE_FAULT
};

// PCS command queue message types
enum PCSCommandType : uint8_t {
  PCS_CMD_SET_CHARGE_POWER,
  PCS_CMD_SET_HV_VOLTAGE,
  PCS_CMD_SET_DCDC_VOLTAGE,
  PCS_CMD_SET_AC_LIMIT,
  PCS_CMD_SET_CHARGE_TERMINATION,
  PCS_CMD_ENABLE_PCS,
  PCS_CMD_CLEAR_FAULTS,
  // State machine commands (for BMS coordination)
  PCS_CMD_START_CHARGING,
  PCS_CMD_START_DRIVE_MODE,
  PCS_CMD_STOP,
  PCS_CMD_EMERGENCY_STOP,
  // Manual debug commands (bypasses state machine)
  PCS_CMD_MANUAL_PCS_ENABLE,
  PCS_CMD_MANUAL_CHARGE_ENABLE,
  PCS_CMD_MANUAL_DCDC_ENABLE,
  PCS_CMD_MANUAL_CAN_ENABLE,
  PCS_CMD_SET_MANUAL_MODE
};

struct PCSCommand {
  PCSCommandType type;
  union {
    uint16_t value_u16;
    float value_float;
    bool value_bool;
  };
};

class PCSController {
public:
  // ==================== INITIALIZATION ====================

  // Initialize the PCS controller with control pins
  static void begin(CANBus *ipc_can_bus, uint8_t pcs_enable_pin, uint8_t charge_enable_pin, uint8_t dcdc_enable_pin);

  // Start the PCS FreeRTOS task
  static bool start_task();

  // Periodic update - called by task loop
  static void update();

  // ==================== STATE ACCESSORS (read-only) ====================

  // Core status data
  static const ChargerStatus& get_charger_status() { return charger_status_; }
  static const DCDCStatus& get_dcdc_status() { return dcdc_status_; }
  static const DCDCBusStatus& get_dcdc_bus_status() { return dcdc_bus_status_; }
  static const ACStatus& get_ac_status() { return ac_status_; }
  static const TemperatureData& get_temperature_data() { return temperature_data_; }
  static const VoltageData& get_voltage_data() { return voltage_data_; }
  static const DCCurrentData& get_dc_current_data() { return dc_current_data_; }
  static const ControlParams& get_control_params() { return control_params_; }

  // Logging data (from 0x2C4 / PCS_logging)
  static const PhaseLoggingData& get_phase_a_logging() { return phase_a_logging_; }
  static const PhaseLoggingData& get_phase_b_logging() { return phase_b_logging_; }
  static const PhaseLoggingData& get_phase_c_logging() { return phase_c_logging_; }
  static const ChargerLineVoltageData& get_charger_line_voltage() { return charger_line_voltage_; }
  static const ChargerFrequencyData& get_charger_frequency() { return charger_frequency_; }
  static const ChargerPhaseStateData& get_charger_phase_state() { return charger_phase_state_; }
  static const DCDCLoggingData& get_dcdc_logging() { return dcdc_logging_; }
  static const DCDCControlData& get_dcdc_control() { return dcdc_control_; }

  // Alert matrix (from 0x3A4 / PCS_alertMatrix)
  static const AlertMatrixState& get_alert_matrix() { return alert_matrix_; }
  static bool is_alert_active(uint8_t alert_num);
  static uint8_t get_active_alert_count() { return alert_matrix_.active_count; }

  // PCS hardware info (from 0x3C4 / PCS_info)
  static const PCSInfoData& get_pcs_info() { return pcs_info_; }

  // Operating mode
  static PCSMode get_mode() { return current_mode_; }

  // ==================== STATE UPDATE METHODS (called by PCSCan) ====================

  // These methods are called by PCSCan when it parses incoming CAN messages
  static void update_charger_status(const ChargerStatus& status) { charger_status_ = status; }
  static void update_dcdc_status(const DCDCStatus& status) { dcdc_status_ = status; }
  static void update_dcdc_bus_status(const DCDCBusStatus& status) { dcdc_bus_status_ = status; }
  static void update_ac_status(const ACStatus& status) { ac_status_ = status; }
  static void update_temperature_data(const TemperatureData& data) { temperature_data_ = data; }
  static void update_lv_voltage(float lv_v) { voltage_data_.lv_v = lv_v; }
  static void update_hv_voltage(uint16_t hv_v) { voltage_data_.hv_v = hv_v; }
  static void update_dc_current_data(const DCCurrentData& data) { dc_current_data_ = data; }

  // Partial updates for incremental CAN message parsing
  static void update_dcdc_current(float current_a, float lv_voltage);
  static void update_dc_phase_current(uint8_t phase, float current_a);

  // Logging data updates
  static void update_phase_logging(uint8_t phase, const PhaseLoggingData& data);
  static void update_charger_line_voltage(const ChargerLineVoltageData& data) { charger_line_voltage_ = data; }
  static void update_charger_frequency(const ChargerFrequencyData& data) { charger_frequency_ = data; }
  static void update_charger_phase_state(const ChargerPhaseStateData& data) { charger_phase_state_ = data; }
  static void update_dcdc_logging(const DCDCLoggingData& data) { dcdc_logging_ = data; }
  static void update_dcdc_control(const DCDCControlData& data) { dcdc_control_ = data; }

  // Alert matrix updates
  static void update_alert_matrix_page(uint8_t page, const uint8_t* data);

  // PCS info updates
  static void update_pcs_info(const PCSInfoData& info) { pcs_info_ = info; }

  // ==================== STATE MACHINE CONTROL ====================

  // State machine control - INTERNAL USE ONLY (called from queue processing)
  // BMS should use the async versions below for thread-safe operation
  static void start_charging();     // Initiate precharge and charging sequence
  static void start_drive_mode();   // Initiate precharge and drive mode (DCDC only)
  static void stop();               // Stop operation (graceful shutdown)
  static void emergency_stop();     // Immediate shutdown

  // Thread-safe state machine control (queue-based, safe to call from BMS task)
  static bool start_charging_async();    // Request charging mode with precharge
  static bool start_drive_mode_async();  // Request drive mode (DCDC) with precharge
  static bool stop_async();              // Request graceful shutdown
  static bool emergency_stop_async();    // Request immediate shutdown

  // ==================== CONTROL METHODS ====================

  // Direct control methods (only for PCS task internal use)
  static void set_charge_power(uint16_t power_w);
  static void set_hv_voltage(uint16_t voltage_v);
  static void set_dcdc_voltage(float voltage_v);
  static void set_ac_current_limit(uint8_t limit_a);
  static void set_charge_termination_percent(uint8_t percent);
  static void set_evse_limit(uint8_t limit_a);
  static void set_cable_limit(uint8_t limit);
  static void enable_pcs(bool enable);

  // Async control methods (queue-based, safe to call from any task)
  static bool set_charge_power_async(uint16_t power_w);
  static bool set_hv_voltage_async(uint16_t voltage_v);
  static bool set_dcdc_voltage_async(float voltage_v);
  static bool set_ac_current_limit_async(uint8_t limit_a);
  static bool set_charge_termination_percent_async(uint8_t percent);
  static bool enable_pcs_async(bool enable);
  static bool clear_faults_async();  // Request PCS to clear faults

  // Manual debug control (bypasses state machine - for fault debugging)
  static bool set_manual_mode_async(bool enable);
  static bool manual_pcs_enable_async(bool enable);
  static bool manual_charge_enable_async(bool enable);
  static bool manual_dcdc_enable_async(bool enable);
  static bool manual_can_enable_async(bool enable);
  static bool is_manual_mode() { return manual_override_; }

  // ==================== STATE QUERIES ====================

  // State machine state
  static PCSState get_state() { return current_state_; }

  // BMS coordination - check PCS readiness
  static bool is_precharging() { return current_state_ == PCS_STATE_PRECHARGE; }
  static bool is_precharge_complete() {
    return current_state_ == PCS_STATE_ACTIVATE ||
           current_state_ == PCS_STATE_CHARGING ||
           current_state_ == PCS_STATE_DRIVE;
  }
  static bool is_active() {
    return current_state_ == PCS_STATE_CHARGING ||
           current_state_ == PCS_STATE_DRIVE;
  }
  static bool is_off() { return current_state_ == PCS_STATE_OFF; }
  static bool is_faulted() { return current_state_ == PCS_STATE_FAULT; }

  // Pin state accessors
  static bool is_pcs_enabled() { return pcs_pin_enabled_; }
  static bool is_charge_enabled() { return charge_pin_enabled_; }
  static bool is_dcdc_enabled() { return dcdc_pin_enabled_; }
  static bool is_can_enabled() { return can_enabled_; }

  // Public accessor for timer_ticks for debug timing
  static uint16_t get_timer_ticks() { return timer_ticks_; }

  // ==================== DEBUG / PRINT HELPERS ====================

  static const char* get_alert_name(uint8_t alert_num);
  static const char* charge_state_str(PCSChargeStatus state);
  static const char* dcdc_state_str(DCDCMainState state);
  static const char* grid_config_str(PCSGridConfig config);

  static void print_pcs_info();
  static void print_dcdc_status(Stream &out);
  static void print_ac_charge_status(Stream &out);
  static void print_active_alerts(Stream &out);

private:
  // ==================== STATE DATA (owned by PCSController) ====================

  // Core status from CAN messages
  static ChargerStatus charger_status_;
  static DCDCStatus dcdc_status_;
  static DCDCBusStatus dcdc_bus_status_;
  static ACStatus ac_status_;
  static TemperatureData temperature_data_;
  static VoltageData voltage_data_;
  static DCCurrentData dc_current_data_;
  static ControlParams control_params_;

  // Logging data from 0x2C4 (PCS_logging)
  static PhaseLoggingData phase_a_logging_;
  static PhaseLoggingData phase_b_logging_;
  static PhaseLoggingData phase_c_logging_;
  static ChargerLineVoltageData charger_line_voltage_;
  static ChargerFrequencyData charger_frequency_;
  static ChargerPhaseStateData charger_phase_state_;
  static DCDCLoggingData dcdc_logging_;
  static DCDCControlData dcdc_control_;

  // Alert matrix from 0x3A4 (PCS_alertMatrix)
  static AlertMatrixState alert_matrix_;

  // PCS info from 0x3C4 (PCS_info)
  static PCSInfoData pcs_info_;

  // Operating mode
  static PCSMode current_mode_;

  // ==================== HARDWARE CONTROL ====================

  // Control pins
  static uint8_t pcs_enable_pin_;
  static uint8_t charge_enable_pin_;
  static uint8_t dcdc_enable_pin_;

  // Pin states
  static bool pcs_pin_enabled_;
  static bool charge_pin_enabled_;
  static bool dcdc_pin_enabled_;
  static bool can_enabled_;
  static bool manual_override_;

  // ==================== STATE MACHINE ====================

  static PCSState current_state_;
  static PCSState target_state_;

  // Precharge timing (in 100ms ticks, default 30 = 3 seconds)
  static uint8_t precharge_timer_;
  static const uint8_t PRECHARGE_TIME_TICKS = 30;

  // PCS wakeup timing (in 100ms ticks, default 2 = 200ms)
  static uint8_t pcs_wakeup_timer_;
  static const uint8_t PCS_WAKEUP_TICKS = 2;

  // Power down timing (in 100ms ticks, default 10 = 1 second)
  static uint8_t powerdown_timer_;
  static const uint8_t POWERDOWN_TIME_TICKS = 10;

  // ==================== TIMING ====================

  static HardwareTimer *message_timer_;
  static volatile bool flag_10ms_;
  static volatile bool flag_20ms_;
  static volatile bool flag_50ms_;
  static volatile bool flag_100ms_;
  static volatile bool flag_500ms_;
  static volatile uint16_t timer_ticks_;

  static void timer_callback();

  // ==================== TASK / QUEUE ====================

  static QueueHandle_t command_queue_;

  static void task_loop();
  static void task_wrapper(void *pvParameters);

  // ==================== INTERNAL HELPERS ====================

  static void process_command_queue();
  static void send_initial_messages();
  static void send_10ms_messages();
  static void send_20ms_messages();
  static void send_50ms_messages();
  static void send_100ms_messages();
  static void send_500ms_messages();
  static void update_control_pins();
  static void run_state_machine();
  static void disable_all();
  static void record_can_timing(uint32_t now);
  static uint8_t count_alert_bits(const uint8_t* bytes);
};
