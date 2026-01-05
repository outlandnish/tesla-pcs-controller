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

// Charge status flag (from message 0x204 bits 4-5 / PCS_chargeStatus)
enum ChargeStatusFlag : uint8_t {
  CHARGE_STATUS_IDLE = 0,
  CHARGE_STATUS_ENABLED = 1,
  CHARGE_STATUS_FAULTED = 2
};

// DCDC main state (from message 0x224 / PCS_dcdcStatus bits 6-9)
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

// Control parameters set by higher-level controller
struct ControlParams {
  uint16_t hv_voltage_v;          // HV bus voltage setpoint (V)
  uint16_t charge_power_w;        // Requested charge power (W)
  float dcdc_voltage_v;           // DCDC output voltage setpoint (V)
  uint8_t ac_current_limit_a;     // AC current limit (A)
  uint8_t evse_limit_a;           // EVSE current limit (A)
  uint8_t cable_limit;            // Cable limit code
};

// Charger status data (from message 0x204 / PCS_chgStatus)
// DBC: Contains charger state machine status and operational flags
struct ChargerStatus {
  PCSChargeStatus main_state;           // Charge main state (bits 0-3)
  ChargeStatusFlag charge_status;       // Charge status flag (bits 4-5)
  PCSGridConfig grid_config;            // Grid configuration (bits 6-7)
  bool phase_a_enabled;                 // Phase A enabled (bit 8)
  bool phase_b_enabled;                 // Phase B enabled (bit 9)
  bool phase_c_enabled;                 // Phase C enabled (bit 10)
  float instant_power_available_kw;     // Instant AC power available (bits 16-23, scale 0.1)
  float max_power_available_kw;         // Max AC power available (bits 24-31, scale 0.1)
  float phase_a_current_request_a;      // Phase A line current request (bits 32-39, scale 0.1)
  float phase_b_current_request_a;      // Phase B line current request (bits 40-47, scale 0.1)
  float phase_c_current_request_a;      // Phase C line current request (bits 48-55, scale 0.1)
  bool pwm_enable;                      // PWM enable line (bit 56)
};

// DCDC status data (from message 0x224 / PCS_dcdcStatus)
// DBC: Contains state machine status and operational flags
struct DCDCStatus {
  float current_a;                     // DCDC output current (A) - from 0x2B4
  float power_w;                       // DCDC output power (W) - calculated
  DCDCStatusFlag precharge_status;     // Precharge status (bits 0-1)
  DCDCStatusFlag support_12v_status;   // 12V support status (bits 2-3)
  DCDCStatusFlag hvbus_discharge_status; // HV bus discharge status (bits 4-5)
  DCDCMainState main_state;            // Main state machine (bits 6-9)
  uint8_t sub_state;                   // Sub-state (bits 10-14)
  bool faulted;                        // Fault flag (bit 15)
  bool output_limited;                 // Output current limited (bit 28)
  float max_output_current_a;          // Max output current allowed (bits 29-40, scale 0.1)
  bool pwm_enable;                     // PWM enable line (bit 52)
  bool supporting_fixed_lv;            // Supporting fixed LV target (bit 53)
};

// DCDC bus status (from message 0x2B4 / PCS_dcdcBusStatus)
// DBC: 0x2B4 PCS_dcdcBusStatus contains voltage and current measurements
struct DCDCBusStatus {
  float lv_bus_voltage_v;         // LV bus voltage (V) - 10-bit, scale 0.0390625
  float hv_bus_voltage_v;         // HV bus voltage (V) - 12-bit, scale 0.146484375
  float lv_output_current_a;      // LV output current (A) - 12-bit, scale 0.1
};

// AC line status (from message 0x264)
struct ACStatus {
  uint16_t current_limit_a;       // AC current limit from PCS (A)
  float power_kw;                 // AC input power (kW)
  uint16_t voltage_v;             // AC input voltage (V)
  float current_a;                // AC input current (A)
};

// Temperature readings (from message 0x2A4 / PCS_thermalStatus)
// DBC: All temps are 11-bit signed, scale 0.1, offset +40°C
struct TemperatureData {
  float phase_a_c;                // Phase A (charger) temperature (°C)
  float phase_b_c;                // Phase B (charger) temperature (°C)
  float phase_c_c;                // Phase C (charger) temperature (°C)
  float dcdc_c;                   // DCDC temperature (°C)
  float ambient_c;                // Ambient temperature (°C)
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

// Message multiplexing state
struct MuxState {
  bool mux_3b2;                   // Message 0x3B2 multiplex toggle
  bool mux_545;                   // Message 0x545 multiplex toggle
  uint8_t count_545;              // Message 0x545 counter (0-15)
  uint8_t mux_2c4;                // Message 0x2C4 multiplex ID
  bool backup_2c4;                // Message 0x2C4 backup mode flag
  bool got_dci;                   // DC current from 0x2C4 (vs 0x76C)
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
  static void handle2B4(uint32_t data[2]);  // DCDC bus status (voltage/current)
  static void handle2C4(uint32_t data[2]);  // Voltage log (multiplexed)
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
  static void set_charge_enable(bool enable) { charge_enable = enable; }

  // Struct-level status accessors (return references for efficient access)
  static const ControlParams& get_control_params() { return control_params; }
  static const ChargerStatus& get_charger_status() { return charger_status; }
  static const DCDCStatus& get_dcdc_status() { return dcdc_status; }
  static const DCDCBusStatus& get_dcdc_bus_status() { return dcdc_bus_status; }
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
  static DCDCBusStatus dcdc_bus_status;
  static ACStatus ac_status;
  static TemperatureData temperature_data;
  static VoltageData voltage_data;
  static DCCurrentData dc_current_data;
  static MuxState mux_state;

  // Helper functions
  static uint8_t calc_checksum(uint8_t *data, uint16_t id);
  static float convert_temp_11bit(uint16_t raw);  // DBC: 11-bit signed, scale 0.1, offset +40
};
