#pragma once

/*
 * Tesla Model 3 PCS CAN Communication Layer
 *
 * Low-level CAN message handling for the Tesla Model 3 onboard charger.
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 */

#include <Arduino.h>
#include "../can/can.h"

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

// PCS charge status (from message 0x204 byte 0 bits 0-3)
enum PCSChargeStatus : uint8_t {
  PCS_STATUS_INIT = 0,           // Initializing
  PCS_STATUS_IDLE = 1,           // Idle
  PCS_STATUS_STARTUP = 2,        // Starting up
  PCS_STATUS_WAIT_LINE = 3,      // Wait for line voltage
  PCS_STATUS_QUALIFY = 4,        // Qualify line configuration
  PCS_STATUS_ENABLE = 5,         // Enabled/charging
  PCS_STATUS_SHUTDOWN = 7,       // Shutdown
  PCS_STATUS_FAULTED = 8,        // Faulted
  PCS_STATUS_CLEAR_FAULTS = 9    // Clearing faults
};

// Grid configuration (from message 0x204 byte 0 bits 6-7)
enum PCSGridConfig : uint8_t {
  PCS_GRID_NONE = 0,             // No grid detected
  PCS_GRID_SINGLE_PHASE = 1,     // Single phase
  PCS_GRID_THREE_PHASE = 2,      // Three phase
  PCS_GRID_THREE_PHASE_DELTA = 3 // Three phase delta
};

// Control parameters set by higher-level controller
struct ControlParams {
  uint16_t hv_voltage_v;          // HV bus voltage setpoint (V)
  uint16_t charge_power_w;        // Requested charge power (W)
  float dcdc_voltage_v;           // DCDC output voltage setpoint (V)
  uint8_t ac_current_limit_a;     // AC current limit (A)
  uint8_t evse_limit_a;           // EVSE current limit (A)
  uint8_t cable_limit;            // Cable current limit (A)
  float charge_termination_pct;   // Charge termination SOC (%, used in 0x333)
};

// Charger status data (from message 0x204)
struct ChargerStatus {
  PCSHardwareType hw_type;        // Hardware type
  PCSChargeStatus status;         // Charge status
  PCSGridConfig grid_config;      // Grid configuration
  float power_available_kw;       // Available charge power (kW)
};

// DCDC status data (from message 0x224)
struct DCDCStatus {
  float current_a;                // DCDC output current (A) — from 0x2B4
  float max_current_a;            // DCDC max allowed output current (A) — from 0x224
  float power_w;                  // DCDC output power (W)
};

// AC line status (from message 0x264)
struct ACStatus {
  uint16_t current_limit_a;       // AC current limit from PCS (A)
  float power_kw;                 // AC input power (kW)
  uint16_t voltage_v;             // AC input voltage (V)
  float current_a;                // AC input current (A)
};

// Temperature readings (from message 0x2A4)
struct TemperatureData {
  int16_t local_c;                // Local temperature (°C)
  uint16_t ambient_raw;           // Ambient temperature (raw)
  uint16_t phase_a_raw;           // Phase A temperature (raw)
  uint16_t phase_b_raw;           // Phase B temperature (raw)
  uint16_t phase_c_raw;           // Phase C temperature (raw)
  uint16_t dcdc_raw;              // DCDC temperature (raw)
  float dcdc_b_c;                 // DCDC B temperature (°C)
};

// Voltage measurements (from message 0x2C4)
struct VoltageData {
  uint16_t hv_v;                  // HV bus voltage (V)
  float lv_v;                     // LV (12V) voltage (V)
};

// DC output current measurements (from messages 0x2C4 and 0x76C)
struct DCCurrentData {
  float phase_a_a;                // Phase A DC output current (A)
  float phase_b_a;                // Phase B DC output current (A)
  float phase_c_a;                // Phase C DC output current (A)
  float total_a;                  // Total DC output current (A)
};

// HVP_pcsControl (0x22A) — HVP_pcsControlRequest enum
enum HvpPcsControlRequest : uint8_t {
  HVP_PCS_CTRL_SHUTDOWN  = 0,
  HVP_PCS_CTRL_SUPPORT   = 1,
  HVP_PCS_CTRL_PRECHARGE = 2,
  HVP_PCS_CTRL_DISCHARGE = 3,
};

// HVP_contactorState (0x20A) — individual contactor state
enum HvpContactorState : uint8_t {
  CONTACTOR_STATE_SNA        = 0,
  CONTACTOR_STATE_OPEN       = 1,
  CONTACTOR_STATE_PRECHARGE  = 2,
  CONTACTOR_STATE_BLOCKED    = 3,
  CONTACTOR_STATE_PULLED_IN  = 4,
  CONTACTOR_STATE_OPENING    = 5,
  CONTACTOR_STATE_ECONOMIZED = 6,
  CONTACTOR_STATE_WELDED     = 7,
};

// HVP_contactorState (0x20A) — contactor set state
enum HvpContactorSetState : uint8_t {
  CONTACTOR_SET_STATE_SNA              = 0,
  CONTACTOR_SET_STATE_OPEN             = 1,
  CONTACTOR_SET_STATE_CLOSING          = 2,
  CONTACTOR_SET_STATE_BLOCKED          = 3,
  CONTACTOR_SET_STATE_OPENING          = 4,
  CONTACTOR_SET_STATE_CLOSED           = 5,
  CONTACTOR_SET_STATE_PARTIAL_WELD     = 6,
  CONTACTOR_SET_STATE_WELDED           = 7,
  CONTACTOR_SET_STATE_POSITIVE_CLOSED  = 8,
  CONTACTOR_SET_STATE_NEGATIVE_CLOSED  = 9,
};

// HVP_contactorState (0x20A) — HVIL status
enum HvpHvilStatus : uint8_t {
  HVIL_UNKNOWN         = 0,
  HVIL_STATUS_OK       = 1,
  HVIL_CURRENT_SOURCE_FAULT = 2,
  HVIL_INTERNAL_OPEN_FAULT  = 3,
  HVIL_VEHICLE_OPEN_FAULT   = 4,
};

// BMS_status (0x212) — BMS state / smStateRequest
enum BmsState : uint8_t {
  BMS_STANDBY    = 0,
  BMS_DRIVE      = 1,
  BMS_SUPPORT    = 2,
  BMS_CHARGE     = 3,
  BMS_FEIM       = 4,
  BMS_CLEAR_FAULT = 5,
  BMS_FAULT      = 6,
  BMS_WELD       = 7,
  BMS_TEST       = 8,
  BMS_SNA        = 9,
  BMS_DIAG       = 10,
};

// BMS_status (0x212) — contactor set state (BMS view)
enum BmsContactorState : uint8_t {
  BMS_CTRSET_SNA     = 0,
  BMS_CTRSET_OPEN    = 1,
  BMS_CTRSET_OPENING = 2,
  BMS_CTRSET_CLOSING = 3,
  BMS_CTRSET_CLOSED  = 4,
  BMS_CTRSET_WELDED  = 5,
  BMS_CTRSET_BLOCKED = 6,
};

// BMS_status (0x212) — UI charge status
enum BmsUiChargeStatus : uint8_t {
  BMS_DISCONNECTED     = 0,
  BMS_NO_POWER         = 1,
  BMS_ABOUT_TO_CHARGE  = 2,
  BMS_CHARGING         = 3,
  BMS_CHARGE_COMPLETE  = 4,
  BMS_CHARGE_STOPPED   = 5,
};

// BMS_status (0x212) — HV state
enum BmsHvState : uint8_t {
  HV_DOWN          = 0,
  HV_COMING_UP     = 1,
  HV_GOING_DOWN    = 2,
  HV_UP_FOR_DRIVE  = 3,
  HV_UP_FOR_CHARGE = 4,
  HV_UP_FOR_DC_CHARGE = 5,
  HV_UP            = 6,
};

// CP_evseStatus (0x21D) — proximity
enum CpProximity : uint8_t {
  CHG_PROXIMITY_SNA         = 0,
  CHG_PROXIMITY_DISCONNECTED = 1,
  CHG_PROXIMITY_UNLATCHED   = 2,
  CHG_PROXIMITY_LATCHED     = 3,
};

// CP_evseStatus (0x21D) — pilot state
enum CpPilot : uint8_t {
  CHG_PILOT_NONE        = 0,
  CHG_PILOT_FAULTED     = 1,
  CHG_PILOT_LINE_CHARGE = 2,
  CHG_PILOT_FAST_CHARGE = 3,
  CHG_PILOT_IDLE        = 4,
};

// CP_evseStatus (0x21D) — EVSE charge type (UI)
enum CpEvseChargeType : uint8_t {
  NO_CHARGER_PRESENT  = 0,
  DC_CHARGER_PRESENT  = 1,
  AC_CHARGER_PRESENT  = 2,
};

// CP_evseStatus (0x21D) — AC charge state
enum CpAcChargeState : uint8_t {
  AC_CHARGE_INACTIVE              = 0,
  AC_CHARGE_CONNECTED_CHARGE_BLOCKED = 1,
  AC_CHARGE_STANDBY               = 2,
  AC_CHARGE_ENABLED               = 3,
  AC_CHARGE_ONBOARD_CHARGER_SHUTDOWN = 4,
  AC_CHARGE_VEH_SHUTDOWN          = 5,
  AC_CHARGE_FAULT                 = 6,
};

// VCFRONT_vehicleStatus (0x3A1) — 12V status for drive
enum Vcfront12vStatus : uint8_t {
  NOT_READY_FOR_DRIVE_12V      = 0,
  READY_FOR_DRIVE_12V          = 1,
  EXIT_DRIVE_REQUESTED_12V     = 2,
};

// VCFRONT_sensors (0x321) — brake fluid level / washer fluid level
enum VcfrontFluidLevel : uint8_t {
  FLUID_SNA    = 0,
  FLUID_LOW    = 1,
  FLUID_NORMAL = 2,
};

// Message multiplexing state
struct MuxState {
  bool mux_3b2;                   // Message 0x3B2 multiplex toggle
  bool mux_545;                   // Message 0x545 multiplex toggle
  uint8_t count_545;              // Message 0x545 counter (0-15)
  uint8_t count_3a1;              // Message 0x3A1 vehicleStatusCounter (0-15)
  uint8_t mux_2c4;                // Message 0x2C4 current mux ID
  bool got_dci;                   // DC current received from 0x2C4 (vs 0x76C fallback)
};

class PCSCan {
public:
  // Initialize the PCS CAN interface
  static void begin(CANBus *ipc_can_bus);

  // Process incoming CAN messages (polls CAN bus directly)
  static void process_messages();

  // Process a single CAN frame
  static void process_frame(uint32_t can_id, uint32_t data[2]);

  // CAN message transmission methods (called periodically)
  static void Msg13D();   // AC current limit message
  static void Msg20A();   // Static configuration message
  static void Msg212();   // Static configuration message
  static void Msg21D();   // EVSE and cable limits
  static void Msg22A();   // Main control message (mode and voltage)
  static void Msg232();   // Static configuration message
  static void Msg23D();   // AC current limit message (alternate)
  static void Msg25D();   // Static configuration message
  static void Msg2B2(uint16_t charge_power_w);  // Charge power request
  static void Msg321();   // Static configuration message
  static void Msg333();   // UI charge request watchdog
  static void Msg3A1();   // DCDC voltage setpoint
  static void Msg3B2();   // BMS log message (multiplexed)
  static void Msg545();   // VCFront message with counter/CRC

  // CAN message reception handlers
  static void handle204(uint32_t data[2]);  // Charger status
  static void handle224(uint32_t data[2]);  // DCDC data
  static void handle264(uint32_t data[2]);  // AC line status
  static void handle2A4(uint32_t data[2]);  // Temperature data
  static void handle2B4(uint32_t data[2]);  // PCS_dcdcRailStatus (HV/LV voltages, LV current)
  static void handle2C4(uint32_t data[2]);  // PCS_logging (multiplexed)
  static void handle3A4(uint32_t data[2]);  // Alert page
  static void handle424(uint32_t data[2]);  // Alert data
  static void handle504(uint32_t data[2]);  // Boot ID
  static void handle76C(uint32_t data[2]);  // Current measurements (alternate)

  // Control parameter setters (called by higher-level PCS controller)
  static void set_mode(PCSMode mode) { current_mode = mode; }
  static void set_hv_voltage(uint16_t voltage_v) { control_params.hv_voltage_v = voltage_v; }
  static void set_charge_power(uint16_t power_w) { control_params.charge_power_w = power_w; }
  static void set_dcdc_voltage(float voltage_v) { control_params.dcdc_voltage_v = voltage_v; }
  static void set_ac_current_limit(uint8_t limit_a) { control_params.ac_current_limit_a = limit_a; }
  static void set_evse_limit(uint8_t limit_a) { control_params.evse_limit_a = limit_a; }
  static void set_cable_limit(uint8_t limit) { control_params.cable_limit = limit; }
  static void set_charge_termination_pct(float pct) { control_params.charge_termination_pct = pct; }
  static void set_charge_enable(bool enable) { charge_enable = enable; }

  // Struct-level status accessors (return references for efficient access)
  static const ControlParams& get_control_params() { return control_params; }
  static const ChargerStatus& get_charger_status() { return charger_status; }
  static const DCDCStatus& get_dcdc_status() { return dcdc_status; }
  static const ACStatus& get_ac_status() { return ac_status; }
  static const TemperatureData& get_temperature_data() { return temperature_data; }
  static const VoltageData& get_voltage_data() { return voltage_data; }
  static const DCCurrentData& get_dc_current_data() { return dc_current_data; }
  static PCSMode get_mode() { return current_mode; }

private:
  // CAN interface
  static CANBus *can_bus;

  // State tracking
  static PCSMode current_mode;
  static bool charge_enable;

  // Organized data structures
  static ControlParams control_params;
  static ChargerStatus charger_status;
  static DCDCStatus dcdc_status;
  static ACStatus ac_status;
  static TemperatureData temperature_data;
  static VoltageData voltage_data;
  static DCCurrentData dc_current_data;
  static MuxState mux_state;

  // Helper functions
  static uint8_t calc_checksum(uint8_t *data, uint16_t id);
  static int16_t process_temp(uint16_t raw_val);
};
