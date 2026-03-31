#pragma once

/*
 * Tesla Model 3 PCS CAN Communication Layer
 *
 * PURE TRANSPORT LAYER - handles CAN message TX/RX only.
 * All state is owned by PCSController. When messages are parsed,
 * data is pushed up to PCSController via update_*() methods.
 *
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 */

#include <Arduino.h>
#include "../can/can.h"
#include "debug_serial.h"

// ==================== ENUMS ====================

// Enum for VCFRONT_12vStatusForDrive (from DBC, typical meanings)
enum VCFRONT_12vStatusForDrive : uint8_t {
  STATUS_12V_OK = 0,
  STATUS_12V_WARNING = 1,
  STATUS_12V_FAULT = 2,
  STATUS_12V_UNKNOWN = 3
};

// CP_evseStatus enums (from DBC)
enum CP_acChargeState : uint8_t {
  CP_AC_INACTIVE = 0,
  CP_AC_ACTIVE = 1,
  CP_AC_SHUTDOWN = 2
};

enum CP_cableType : uint8_t {
  CP_CABLE_IEC = 0,
  CP_CABLE_SAE = 1,
  CP_CABLE_GB_AC = 2,
  CP_CABLE_GB_DC = 3,
  CP_CABLE_SNA = 4
};

enum CP_evseChargeType : uint8_t {
  CP_EVSE_NONE = 0,
  CP_EVSE_DC = 1,
  CP_EVSE_AC = 2
};

// Enum for BMS_contactorState (from DBC)
enum BMSContactorState : uint8_t {
  BMS_CONTACTOR_SNA = 0,
  BMS_CONTACTOR_OPEN = 1,
  BMS_CONTACTOR_OPENING = 2,
  BMS_CONTACTOR_CLOSING = 3,
  BMS_CONTACTOR_CLOSED = 4,
  BMS_CONTACTOR_WELDED = 5,
  BMS_CONTACTOR_BLOCKED = 6
};

// Enum for BMS_state (from DBC)
enum BMSState : uint8_t {
  BMS_STATE_STANDBY = 0,
  BMS_STATE_DRIVE = 1,
  BMS_STATE_SUPPORT = 2,
  BMS_STATE_CHARGE = 3,
  BMS_STATE_FEIM = 4,
  BMS_STATE_CLEAR_FAULT = 5,
  BMS_STATE_FAULT = 6,
  BMS_STATE_WELD = 7,
  BMS_STATE_TEST = 8,
  BMS_STATE_SNA = 9
};

// Enum for BMS_uiChargeStatus (from DBC)
enum BMSUiChargeStatus : uint8_t {
  BMS_UI_CHARGE_DISCONNECTED = 0,
  BMS_UI_CHARGE_NO_POWER = 1,
  BMS_UI_CHARGE_ABOUT_TO_CHARGE = 2,
  BMS_UI_CHARGE_CHARGING = 3,
  BMS_UI_CHARGE_COMPLETE = 4,
  BMS_UI_CHARGE_STOPPED = 5
};

// PCS operational modes (for message 0x22A byte 2)
enum PCSMode : uint8_t {
  PCS_MODE_OFF = 0x00,           // Both charger and DCDC off
  PCS_MODE_CHARGE_ONLY = 0x05,   // Charge only, DCDC disabled (based on reference)
  PCS_MODE_DCDC_ONLY = 0x09,     // DCDC only, charger disabled
  PCS_MODE_CHARGE_DCDC = 0x0D    // Both charger and DCDC enabled
};

// PCS hardware types (from message 0x204 byte 7 bits 3-4)
enum PCSHardwareType : uint8_t {
  PCS_HW_11KW = 0,               // 48A single phase (11kW)
  PCS_HW_7_7KW = 1,              // 32A single phase (7.7kW)
  PCS_HW_3_8KW = 2               // 16A three phase (3.8kW)
};

// PCS charge main state (from message 0x204 bits 0-3 / PCS_chgMainState)
enum PCSChargeStatus : uint8_t {
  PCS_STATUS_INIT = 0,
  PCS_STATUS_IDLE = 1,
  PCS_STATUS_STARTUP = 2,
  PCS_STATUS_WAIT_LINE = 3,
  PCS_STATUS_QUALIFY = 4,
  PCS_STATUS_SYSTEM_CONFIG = 5,
  PCS_STATUS_ENABLE = 6,
  PCS_STATUS_SHUTDOWN = 7,
  PCS_STATUS_FAULTED = 8,
  PCS_STATUS_CLEAR_FAULTS = 9
};

// Grid configuration (from message 0x204 bits 6-7 / PCS_gridConfig)
enum PCSGridConfig : uint8_t {
  PCS_GRID_SNA = 0,
  PCS_GRID_SINGLE_PHASE = 1,
  PCS_GRID_THREE_PHASE = 2,
  PCS_GRID_THREE_PHASE_DELTA = 3
};

// Charge status flag (from message 0x204 bits 4-5 / PCS_chargeStatus)
enum ChargeStatusFlag : uint8_t {
  CHARGE_STATUS_IDLE = 0,
  CHARGE_STATUS_ENABLED = 1,
  CHARGE_STATUS_FAULTED = 2
};

// DCDC main state (from message 0x224 bits 6-9 / PCS_dcdcMainState)
enum DCDCMainState : uint8_t {
  DCDC_STATE_STANDBY = 0,
  DCDC_STATE_12V_SUPPORT_ACTIVE = 1,
  DCDC_STATE_PRECHARGE_STARTUP = 2,
  DCDC_STATE_PRECHARGE_ACTIVE = 3,
  DCDC_STATE_DIS_HVBUS_ACTIVE = 4,
  DCDC_STATE_SHUTDOWN = 5,
  DCDC_STATE_FAULTED = 6
};

// DCDC status flags (from message 0x224 / PCS_dcdcStatus bits 0-1, 2-3, 4-5)
enum DCDCStatusFlag : uint8_t {
  DCDC_STATUS_IDLE = 0,
  DCDC_STATUS_ACTIVE = 1,
  DCDC_STATUS_FAULTED = 2
};

// Vehicle power state (from message 0x545 bits 5-6 / VCFRONT_vehiclePowerState)
enum VehiclePowerState : uint8_t {
  VEHICLE_POWER_OFF = 0,
  VEHICLE_POWER_CONDITIONING = 1,
  VEHICLE_POWER_ACCESSORY = 2,
  VEHICLE_POWER_DRIVE = 3
};

// Generic LV state (used by all VCFRONT LV state/request signals in 0x545)
// Values: OFF=0, ON=1, GOING_DOWN=2, FAULT=3
enum LVState : uint8_t {
  LV_OFF = 0,
  LV_ON = 1,
  LV_GOING_DOWN = 2,
  LV_FAULT = 3
};

// HVP contactor state (for packContNegativeState, packContPositiveState, fcContNegativeState, fcContPositiveState)
// DBC: VAL_ 522 HVP_packContNegativeState 0 "SNA" 1 "OPEN" 2 "PRECHARGE" 3 "BLOCKED" 4 "PULLED_IN" 5 "OPENING" 6 "ECONOMIZED" 7 "WELDED"
enum HVP_ContactorState : uint8_t {
  HVP_CONTACTOR_SNA = 0,
  HVP_CONTACTOR_OPEN = 1,
  HVP_CONTACTOR_PRECHARGE = 2,
  HVP_CONTACTOR_BLOCKED = 3,
  HVP_CONTACTOR_PULLED_IN = 4,
  HVP_CONTACTOR_OPENING = 5,
  HVP_CONTACTOR_ECONOMIZED = 6,
  HVP_CONTACTOR_WELDED = 7
};

// HVP contactor set state (for packContactorSetState, fcContactorSetState)
// DBC: VAL_ 522 HVP_packContactorSetState 0 "SNA" 1 "OPEN" 2 "CLOSING" 3 "BLOCKED" 4 "OPENING" 5 "CLOSED" 6 "PARTIAL_WELD" 7 "WELDED" 8 "POSITIVE_CLOSED" 9 "NEGATIVE_CLOSED"
enum HVP_ContactorSetState : uint8_t {
  HVP_SET_SNA = 0,
  HVP_SET_OPEN = 1,
  HVP_SET_CLOSING = 2,
  HVP_SET_BLOCKED = 3,
  HVP_SET_OPENING = 4,
  HVP_SET_CLOSED = 5,
  HVP_SET_PARTIAL_WELD = 6,
  HVP_SET_WELDED = 7,
  HVP_SET_POSITIVE_CLOSED = 8,
  HVP_SET_NEGATIVE_CLOSED = 9
};

// HVP contactor request status (for packCtrsRequestStatus, fcCtrsRequestStatus)
// DBC: VAL_ 522 HVP_packCtrsRequestStatus 0 "NOT_ACTIVE" 1 "ACTIVE" 2 "COMPLETED"
enum HVP_CtrsRequestStatus : uint8_t {
  HVP_CTRS_NOT_ACTIVE = 0,
  HVP_CTRS_ACTIVE = 1,
  HVP_CTRS_COMPLETED = 2
};

// HVP HVIL status
// DBC: VAL_ 522 HVP_hvilStatus 0 "UNKNOWN" 1 "STATUS_OK" 2 "CURRENT_SOURCE_FAULT" 3 "INTERNAL_OPEN_FAULT" 4 "VEHICLE_OPEN_FAULT" 5 "PENTHOUSE_LID_OPEN_FAULT" 6 "UNKNOWN_LOCATION_OPEN_FAULT" 7 "VEHICLE_NODE_FAULT" 8 "NO_12V_SUPPLY"
enum HVP_HvilStatus : uint8_t {
  HVP_HVIL_UNKNOWN = 0,
  HVP_HVIL_STATUS_OK = 1,
  HVP_HVIL_CURRENT_SOURCE_FAULT = 2,
  HVP_HVIL_INTERNAL_OPEN_FAULT = 3,
  HVP_HVIL_VEHICLE_OPEN_FAULT = 4,
  HVP_HVIL_PENTHOUSE_LID_OPEN_FAULT = 5,
  HVP_HVIL_UNKNOWN_LOCATION_OPEN_FAULT = 6,
  HVP_HVIL_VEHICLE_NODE_FAULT = 7,
  HVP_HVIL_NO_12V_SUPPLY = 8
};

// Log message select (from message 0x2C4 / PCS_logging bits 0-4)
enum LogMessageSelect : uint8_t {
  LOG_PHA_1 = 0,
  LOG_PHB_1 = 1,
  LOG_PHC_1 = 2,
  LOG_CHG_1 = 3,
  LOG_CHG_2 = 4,
  LOG_CHG_3 = 5,
  LOG_DCDC_1 = 6,
  LOG_DCDC_2 = 7,
  LOG_DCDC_3 = 8,
  LOG_SYSTEM_1 = 9,
  LOG_PHA_2 = 10,
  LOG_PHB_2 = 11,
  LOG_PHC_2 = 12,
  LOG_CHG_4 = 13,
  LOG_DLOG_1 = 14,
  LOG_DLOG_2 = 15,
  LOG_DLOG_3 = 16,
  LOG_DLOG_4 = 17,
  LOG_DCDC_4 = 18,
  LOG_DCDC_5 = 19,
  LOG_CHG_NO_FLOW = 20,
  LOG_CHG_LINE_OFFSET = 21,
  LOG_DCDC_STATISTICS = 22,
  LOG_CHG_STATISTICS = 23,
  LOG_NUM_MSGS = 24
};

// Charger phase state (from 0x2C4 mux 5 / CHG_3)
enum ChargerPhaseState : uint8_t {
  CHG_PHASE_INIT = 0,
  CHG_PHASE_IDLE = 1,
  CHG_PHASE_PRECHARGE = 2,
  CHG_PHASE_ENABLE = 3,
  CHG_PHASE_FAULT = 4,
  CHG_PHASE_CLEAR_FAULTS = 5,
  CHG_PHASE_SHUTTING_DOWN = 6
};

// Charger phase shutdown reason (from 0x2C4 mux 5 / CHG_3)
enum ChargerShutdownReason : uint8_t {
  SHUTDOWN_REASON_NONE = 0,
  SHUTDOWN_SW_ENABLE = 1,
  SHUTDOWN_HW_ENABLE = 2,
  SHUTDOWN_SW_FAULT = 3,
  SHUTDOWN_HW_FAULT = 4,
  SHUTDOWN_PLL_NOT_LOCKED = 5,
  SHUTDOWN_INPUT_UV = 6,
  SHUTDOWN_INPUT_OV = 7,
  SHUTDOWN_OUTPUT_UV = 8,
  SHUTDOWN_OUTPUT_OV = 9,
  SHUTDOWN_PRECHARGE_TIMEOUT = 10,
  SHUTDOWN_INT_BUS_UV = 11,
  SHUTDOWN_CONTROL_REGULATION_FAULT = 12,
  SHUTDOWN_OVER_TEMPERATURE = 13,
  SHUTDOWN_TEMP_IRRATIONAL = 14,
  SHUTDOWN_SENSOR_IRRATIONAL = 15,
  SHUTDOWN_FREQ_OUT_OF_RANGE = 16,
  SHUTDOWN_LINE_TRANSIENT_FAULT = 17
};

// ==================== DATA STRUCTURES ====================

// Control parameters for TX messages
struct ControlParams {
  uint16_t hv_voltage_v;
  uint16_t charge_power_w;
  float dcdc_voltage_v;
  uint8_t ac_current_limit_a;
  uint8_t evse_limit_a;
  uint8_t cable_limit;
  uint8_t charge_termination_percent;  // Charge termination percentage (0-100)
};

// Charger status (from 0x204 / PCS_chgStatus)
struct ChargerStatus {
  PCSChargeStatus main_state;
  ChargeStatusFlag charge_status;
  PCSGridConfig grid_config;
  bool phase_a_enabled;
  bool phase_b_enabled;
  bool phase_c_enabled;
  float instant_power_available_kw;
  float max_power_available_kw;
  float phase_a_current_request_a;
  float phase_b_current_request_a;
  float phase_c_current_request_a;
  bool pwm_enable;
};

// DCDC status (from 0x224 / PCS_dcdcStatus)
struct DCDCStatus {
  float current_a;
  float power_w;
  DCDCStatusFlag precharge_status;
  DCDCStatusFlag support_12v_status;
  DCDCStatusFlag hvbus_discharge_status;
  DCDCMainState main_state;
  uint8_t sub_state;
  bool faulted;
  bool output_limited;
  float max_output_current_a;
  bool pwm_enable;
  bool supporting_fixed_lv;
};

// DCDC bus status (from 0x2B4 / PCS_dcdcBusStatus)
struct DCDCBusStatus {
  float lv_bus_voltage_v;
  float hv_bus_voltage_v;
  float lv_output_current_a;
};

// AC line status (from 0x264)
struct ACStatus {
  uint16_t current_limit_a;
  float power_kw;
  uint16_t voltage_v;
  float current_a;
};

// Temperature readings (from 0x2A4 / PCS_thermalStatus)
struct TemperatureData {
  float phase_a_c;
  float phase_b_c;
  float phase_c_c;
  float dcdc_c;
  float ambient_c;
};

// Voltage measurements
struct VoltageData {
  uint16_t hv_v;
  float lv_v;
};

// DC output current measurements
struct DCCurrentData {
  float phase_a_a;
  float phase_b_a;
  float phase_c_a;
  float total_a;
};

// Per-phase logging data (from 0x2C4 mux 0/1/2)
struct PhaseLoggingData {
  float input_current_rms_a;
  uint16_t internal_bus_voltage_v;
  uint16_t internal_bus_target_v;
  float output_current_a;
};

// Charger line voltage data (from 0x2C4 mux 3)
struct ChargerLineVoltageData {
  float l1n_voltage_v;
  float l2n_voltage_v;
  float l3n_voltage_v;
  float l1l2_voltage_v;
  uint16_t ng_voltage_v;
};

// Charger frequency data (from 0x2C4 mux 4)
struct ChargerFrequencyData {
  float l1n_frequency_hz;
  float l2n_frequency_hz;
  float l3n_frequency_hz;
  uint8_t internal_phase_config;
  float output_voltage_v;
};

// Charger phase state data (from 0x2C4 mux 5)
struct ChargerPhaseStateData {
  ChargerPhaseState phase_a_state;
  ChargerPhaseState phase_b_state;
  ChargerPhaseState phase_c_state;
  ChargerShutdownReason phase_a_shutdown_reason;
  ChargerShutdownReason phase_b_shutdown_reason;
  ChargerShutdownReason phase_c_shutdown_reason;
  uint8_t phase_a_retry_count;
  uint8_t phase_b_retry_count;
  uint8_t phase_c_retry_count;
  uint8_t charger_retry_count;
  bool l1n_pll_locked;
  bool l2n_pll_locked;
  bool l3n_pll_locked;
  bool l1l2_pll_locked;
  bool ng_pll_locked;
};

// DCDC logging data (from 0x2C4 mux 6)
struct DCDCLoggingData {
  float max_lv_output_current_a;
  float current_limit_a;
  float lv_output_temp_limit_a;
};

// Alert matrix state (from 0x3A4 / PCS_alertMatrix)
struct AlertMatrixState {
  uint8_t page0[8];
  uint8_t page1[8];
  bool page0_valid;
  bool page1_valid;
  uint8_t active_count;
};

// PCS hardware info (from 0x3C4 / PCS_info)
struct PCSInfoData {
  uint8_t build_type;
  uint16_t build_config_id; // Mux 10: Build configuration ID 16|16@1+ (1,0) [0|0] "" X
  uint16_t hardware_id;
  uint16_t component_id;
  uint8_t pcba_id;  // Mux 11: PCBA ID 16|8@1+ (1,0) [0|0] "" X
  uint8_t assembly_id; // Mux 11: Assembly ID 24|8@1+ (1,0) [0|0] "" X
  uint16_t usage_id; // Mux 11: Usage ID 32|16@1+ (1,0) [0|0] "" X
  uint16_t subusage_id; // Mux 11: Sub-usage ID 48|16@1+ (1,0) [0|0] "" X
  uint8_t platform_type;
  uint32_t app_crc; // Mux 13: Application CRC 32|32@1+ (1,0) [0|0] "" X
  uint64_t app_git_hash;             // Mux 17: Application Git hash 10|54@1+ (4,0) [0|0] "" X
  uint64_t bootloader_git_hash;      // Mux 18: Bootloader Git hash 10|54@1+ (4,0) [0|0] "" X
  uint8_t bootloader_uds_proto_version; // Mux 20: Bootloader UDS protocol version 8|8@1+ (1,0) [0|0] "" X
  uint32_t boot_crc; // Mux 20: Bootloader CRC 32|32@1+ (1,0) [0|0] "" X
  char part_number[21]; // Mux 25-27: 20 chars + null terminator
  bool info_valid;
};

// DCDC control data (from 0x2C4 mux 7)
struct DCDCControlData {
  int16_t tank_voltage_v;
  uint16_t tank_voltage_target_v;
  float switching_freq_khz;
};

// Message multiplexing state (internal to PCSCan for TX)
struct MuxState {
  bool mux_3b2;
  bool mux_545;      // VCFRONT_LVPowerState alternates mux 0/1
  uint8_t count_545; // VCFRONT_LVPowerState counter
  uint8_t count_3a1; // VCFRONT_vehicleStatus counter
  uint8_t mux_2c4;
  bool got_dci;
};

// ==================== PCS CAN TRANSPORT CLASS ====================

class PCSCan {
public:
  // Initialize the PCS CAN interface
  static void begin(CANBus *ipc_can_bus);

  // Process incoming CAN messages (polls CAN bus, calls PCSController::update_*())
  static void process_messages();

  // Process a single CAN frame
  static void process_frame(uint32_t can_id, uint32_t data[2]);

  // ==================== TX MESSAGES ====================

  static void Msg13D();   // AC current limit message
  static void Msg20A();   // Static configuration message
  static void Msg212();   // Static configuration message
  static void Msg21D();   // EVSE and cable limits
  static void Msg22A();   // Main control message (mode and voltage)
  static void Msg232();   // Static configuration message
  static void Msg23D();   // AC current limit message (alternate)
  static void Msg25D();   // Static configuration message
  static void Msg221();   // VCFRONT_LVPowerState (50ms cycle) - simulated
  static void Msg261();   // VCFRONT_12VBatteryStatus (100ms cycle) - simulated
  static void Msg2E1();   // VCFRONT_status (20ms cycle) - simulated, PCSMia=0 DCDCNoop=0
  static void Msg2B2(uint16_t charge_power_w);  // Charge power request
  static void Msg201();   // Unknown message (4-message cycle, ~1400ms total)
  static void Msg2F1();   // VCFRONT_eFuseDebugStatus (100ms cycle)
  static void Msg301();   // VCFRONT_info (1000ms cycle)
  static void Msg321();   // VCFRONT_sensors (1000ms cycle)
  static void Msg333();   // UI charge request watchdog
  static void Msg340();   // VCFRONT_alertMatrix (100ms cycle) - simulated
  static void Msg3A1();   // DCDC voltage setpoint
  static void Msg3B2();   // BMS log message (multiplexed)
  static void Msg545();   // VCFRONT_LVPowerState

  // ==================== CONTROL SETTERS (for TX messages) ====================

  static void set_mode(PCSMode mode) { current_mode = mode; }
  static void set_hv_voltage(uint16_t voltage_v) { control_params.hv_voltage_v = voltage_v; }
  static void set_charge_power(uint16_t power_w) { control_params.charge_power_w = power_w; }
  static void set_dcdc_voltage(float voltage_v) { control_params.dcdc_voltage_v = voltage_v; }
  static void set_ac_current_limit(uint8_t limit_a) { control_params.ac_current_limit_a = limit_a; }
  static void set_evse_limit(uint8_t limit_a) { control_params.evse_limit_a = limit_a; }
  static void set_cable_limit(uint8_t limit) { control_params.cable_limit = limit; }
  static void set_charge_termination_percent(uint8_t percent) { control_params.charge_termination_percent = percent; }
  static void set_charge_enable(bool enable) { charge_enable = enable; }
  static void request_clear_faults() { clear_faults_counter = 1; }

  // Mode accessor (needed by PCSController for TX)
  static PCSMode get_mode() { return current_mode; }

private:
  // ==================== RX HANDLERS ====================

  static void handle204(uint32_t data[2]);  // Charger status -> PCSController
  static void handle224(uint32_t data[2]);  // DCDC status -> PCSController
  static void handle264(uint32_t data[2]);  // AC line status -> PCSController
  static void handle2A4(uint32_t data[2]);  // Temperature -> PCSController
  static void handle2B4(uint32_t data[2]);  // DCDC bus status -> PCSController
  static void handle2C4(uint32_t data[2]);  // Logging data -> PCSController
  static void handle3A4(uint32_t data[2]);  // Alert matrix -> PCSController
  static void handle3C4(uint32_t data[2]);  // PCS info -> PCSController
  static void handle3E4(uint32_t data[2]);  // PCS2 alert matrix (log only)
  static void handle424(uint32_t data[2]);  // Alert data
  static void handle504(uint32_t data[2]);  // Boot ID
  static void handle76C(uint32_t data[2]);  // Current measurements

  // ==================== TRANSPORT STATE (for TX only) ====================

  static CANBus *can_bus;
  static PCSMode current_mode;
  static bool charge_enable;
  static uint8_t clear_faults_counter;
  static bool use_long_msg_format;
  static ControlParams control_params;
  static MuxState mux_state;

  // ==================== HELPERS ====================

  static uint8_t calc_checksum(uint8_t *data, uint16_t id);
  static float convert_temp_11bit(uint16_t raw);
};
