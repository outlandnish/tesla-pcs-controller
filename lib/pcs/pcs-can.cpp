/*
 * Tesla Model 3 PCS CAN Communication Layer
 *
 * Low-level CAN message handling for the Tesla Model 3 onboard charger.
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 */

#include "pcs-can.h"
#include "pcs.h"  // For PCSController::is_pcs_enabled()
#include "param_prj.h"  // For parameter default values
#include "params.h"  // For Param::GetInt()
#include "debug_serial.h"  // Use common debug serial
#include "errormessage.h"  // For error posting

// Import debug flag from main.cpp
#ifndef DEBUG_PCS_CAN
#define DEBUG_PCS_CAN 1
#endif

#ifndef DEBUG_PCS_TX
#define DEBUG_PCS_TX 0
#endif

#ifndef DEBUG_PCS_RX
#define DEBUG_PCS_RX 0
#endif

// Initialize static members
CANBus* PCSCan::can_bus = nullptr;

// State tracking
PCSMode PCSCan::current_mode = PCS_MODE_OFF;
bool PCSCan::charge_enable = false;

// Organized data structures with default values
// Defaults reference constants defined in param_prj.h
ControlParams PCSCan::control_params = {
  .hv_voltage_v = UDCSPNT_DEFAULT,
  .charge_power_w = 0,
  .dcdc_voltage_v = UDCDC_DEFAULT,
  .ac_current_limit_a = IACLIM_DEFAULT,
  .evse_limit_a = 0,
  .cable_limit = 0
};

ChargerStatus PCSCan::charger_status = {
  .main_state = PCS_STATUS_INIT,
  .charge_status = CHARGE_STATUS_IDLE,
  .grid_config = PCS_GRID_NONE,
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

DCDCStatus PCSCan::dcdc_status = {
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

DCDCBusStatus PCSCan::dcdc_bus_status = {
  .lv_bus_voltage_v = 0.0f,
  .hv_bus_voltage_v = 0.0f,
  .lv_output_current_a = 0.0f
};

ACStatus PCSCan::ac_status = {
  .current_limit_a = 0,
  .power_kw = 0.0f,
  .voltage_v = 0,
  .current_a = 0.0f
};

TemperatureData PCSCan::temperature_data = {
  .phase_a_c = 0.0f,
  .phase_b_c = 0.0f,
  .phase_c_c = 0.0f,
  .dcdc_c = 0.0f,
  .ambient_c = 0.0f
};

VoltageData PCSCan::voltage_data = {
  .hv_v = 0,
  .lv_v = 0.0f
};

DCCurrentData PCSCan::dc_current_data = {
  .phase_a_a = 0.0f,
  .phase_b_a = 0.0f,
  .phase_c_a = 0.0f,
  .total_a = 0.0f
};

// Mux state is populated from PCS CAN messages
MuxState PCSCan::mux_state = {
  .mux_3b2 = false,
  .mux_545 = false,
  .count_545 = 0,
  .mux_2c4 = 0,
  .backup_2c4 = false,
  .got_dci = false
};

// Helper function to log RX messages
static void log_rx_message(uint32_t id, const uint32_t data[2]) {
  #if DEBUG_PCS_RX
  const uint8_t* bytes = (const uint8_t*)data;
  DEBUG_SERIAL.print("IPC RX: 0x");
  if (id < 0x100) DEBUG_SERIAL.print("0");
  if (id < 0x10) DEBUG_SERIAL.print("0");
  DEBUG_SERIAL.print(id, HEX);
  DEBUG_SERIAL.print(" [8] ");
  for (uint8_t i = 0; i < 8; i++) {
    if (bytes[i] < 0x10) DEBUG_SERIAL.print("0");
    DEBUG_SERIAL.print(bytes[i], HEX);
    DEBUG_SERIAL.print(" ");
  }
  DEBUG_SERIAL.println();
  #endif
}

void PCSCan::begin(CANBus *ipc_can_bus) {
  can_bus = ipc_can_bus;
  DEBUG_SERIAL.println("PCSCan: Low-level CAN initialized");
}

void PCSCan::process_messages() {
  if (can_bus == nullptr) return;
  
  uint32_t can_id;
  uint8_t data[8];
  uint8_t len;

  while (can_bus->receiveMessage(can_id, data, len)) {
    
    uint32_t data_words[2];
    memcpy(data_words, data, 8);
    process_frame(can_id, data_words);
  }
}

void PCSCan::process_frame(uint32_t can_id, uint32_t data[2]) {
  log_rx_message(can_id, data);

  switch (can_id) {
    case 0x204: handle204(data); break;
    case 0x224: handle224(data); break;
    case 0x264: handle264(data); break;
    case 0x2A4: handle2A4(data); break;
    case 0x2B4: handle2B4(data); break;
    case 0x2C4: handle2C4(data); break;
    case 0x3A4: handle3A4(data); break;
    case 0x424: handle424(data); break;
    case 0x504: handle504(data); break;
    case 0x76C: handle76C(data); break;
    default: break;
  }
}

// CAN Message Reception Handlers (from reference implementation)

void PCSCan::handle204(uint32_t data[2]) {
  // DBC: PCS_chgStatus (0x204 / 516)
  // Contains charger state machine status and operational flags
  uint8_t* bytes = (uint8_t*)data;

  // PCS_chgMainState (bits 0-3)
  charger_status.main_state = (PCSChargeStatus)(bytes[0] & 0x0F);

  // PCS_chargeStatus (bits 4-5)
  charger_status.charge_status = (ChargeStatusFlag)((bytes[0] >> 4) & 0x03);

  // PCS_gridConfig (bits 6-7)
  charger_status.grid_config = (PCSGridConfig)((bytes[0] >> 6) & 0x03);

  // PCS_chgPHAEnable (bit 8)
  charger_status.phase_a_enabled = (bytes[1] >> 0) & 0x01;

  // PCS_chgPHBEnable (bit 9)
  charger_status.phase_b_enabled = (bytes[1] >> 1) & 0x01;

  // PCS_chgPHCEnable (bit 10)
  charger_status.phase_c_enabled = (bytes[1] >> 2) & 0x01;

  // PCS_chgInstantAcPowerAvailable (bits 16-23, scale 0.1 kW)
  charger_status.instant_power_available_kw = bytes[2] * 0.1f;

  // PCS_chgMaxAcPowerAvailable (bits 24-31, scale 0.1 kW)
  charger_status.max_power_available_kw = bytes[3] * 0.1f;

  // PCS_chgPHALineCurrentRequest (bits 32-39, scale 0.1 A)
  charger_status.phase_a_current_request_a = bytes[4] * 0.1f;

  // PCS_chgPHBLineCurrentRequest (bits 40-47, scale 0.1 A)
  charger_status.phase_b_current_request_a = bytes[5] * 0.1f;

  // PCS_chgPHCLineCurrentRequest (bits 48-55, scale 0.1 A)
  charger_status.phase_c_current_request_a = bytes[6] * 0.1f;

  // PCS_chgPwmEnableLine (bit 56)
  charger_status.pwm_enable = (bytes[7] >> 0) & 0x01;
}

void PCSCan::handle224(uint32_t data[2]) {
  // DBC: PCS_dcdcStatus (0x224 / 548)
  // Contains DCDC state machine status and operational flags
  // 
  // NOTE: DCDC current/voltage is read from 0x2B4 (PCS_dcdcBusStatus)
  // which provides accurate readings on 2017 hardware.
  uint8_t* bytes = (uint8_t*)data;
  
  // Parse status flags per DBC
  // PCS_dcdcPrechargeStatus (bits 0-1)
  dcdc_status.precharge_status = (DCDCStatusFlag)(bytes[0] & 0x03);
  
  // PCS_dcdc12VSupportStatus (bits 2-3)
  dcdc_status.support_12v_status = (DCDCStatusFlag)((bytes[0] >> 2) & 0x03);
  
  // PCS_dcdcHvBusDischargeStatus (bits 4-5)
  dcdc_status.hvbus_discharge_status = (DCDCStatusFlag)((bytes[0] >> 4) & 0x03);
  
  // PCS_dcdcMainState (bits 6-9)
  dcdc_status.main_state = (DCDCMainState)((bytes[1] << 2 | bytes[0] >> 6) & 0x0F);
  
  // PCS_dcdcSubState (bits 10-14)
  dcdc_status.sub_state = (bytes[1] >> 2) & 0x1F;
  
  // PCS_dcdcFaulted (bit 15)
  dcdc_status.faulted = (bytes[1] >> 7) & 0x01;
  
  // PCS_dcdcOutputIsLimited (bit 28)
  dcdc_status.output_limited = (bytes[3] >> 4) & 0x01;
  
  // PCS_dcdcMaxOutputCurrentAllowed (bits 29-40, scale 0.1)
  uint16_t raw_max_current = ((bytes[5] << 8 | bytes[4]) >> 5) & 0xFFF;
  dcdc_status.max_output_current_a = raw_max_current * 0.1f;
  
  // PCS_dcdcPwmEnableLine (bit 52)
  dcdc_status.pwm_enable = (bytes[6] >> 4) & 0x01;
  
  // PCS_dcdcSupportingFixedLvTarget (bit 53)
  dcdc_status.supporting_fixed_lv = (bytes[6] >> 5) & 0x01;
}

void PCSCan::handle264(uint32_t data[2]) {
  // DBC: PCS_chgLineStatus (0x264 / 612)
  // Contains AC line measurements and limits
  uint8_t* bytes = (uint8_t*)data;

  // PCS_chgInputVoltage (bits 0-13, 14 bits, scale 0.033)
  ac_status.voltage_v = ((bytes[1] << 8 | bytes[0]) & 0x3FFF) * 0.033f;

  // PCS_chgLineCurrent (bits 14-22, 9 bits, scale 0.1)
  ac_status.current_a = (((bytes[2] << 8 | bytes[1]) >> 6) & 0x1FF) * 0.1f;

  // PCS_chgInputPower (bits 24-31, 8 bits, scale 0.1)
  ac_status.power_kw = bytes[3] * 0.1f;

  // PCS_chgAcCurrentLimit (bits 32-41, 10 bits, scale 0.1)
  ac_status.current_limit_a = ((bytes[5] << 8 | bytes[4]) & 0x3FF) * 0.1f;
}

void PCSCan::handle2A4(uint32_t data[2]) {
  // DBC: PCS_thermalStatus (0x2A4 / 676)
  // All temperatures: 11-bit signed, scale 0.1, offset +40°C
  // Formula: temp_C = raw * 0.1 + 40
  uint8_t* bytes = (uint8_t*)data;

  // Extract 11-bit values per DBC bit positions
  // PCS_chgPhATemp:  bits 0-10
  // PCS_chgPhBTemp:  bits 11-21
  // PCS_chgPhCTemp:  bits 24-34 (note: gap at 22-23)
  // PCS_dcdcTemp:    bits 35-45
  // PCS_ambientTemp: bits 48-58

  uint16_t raw_a = (bytes[1] << 8 | bytes[0]) & 0x7FF;
  uint16_t raw_b = ((bytes[2] << 8 | bytes[1]) >> 3) & 0x7FF;
  uint16_t raw_c = ((bytes[4] << 8 | bytes[3]) >> 0) & 0x7FF;
  uint16_t raw_dcdc = ((bytes[5] << 8 | bytes[4]) >> 3) & 0x7FF;
  uint16_t raw_ambient = ((bytes[7] << 8 | bytes[6]) >> 0) & 0x7FF;

  // Convert to °C using DBC formula
  temperature_data.phase_a_c = convert_temp_11bit(raw_a);
  temperature_data.phase_b_c = convert_temp_11bit(raw_b);
  temperature_data.phase_c_c = convert_temp_11bit(raw_c);
  temperature_data.dcdc_c = convert_temp_11bit(raw_dcdc);
  temperature_data.ambient_c = convert_temp_11bit(raw_ambient);
}

void PCSCan::handle2B4(uint32_t data[2]) {
  // DBC: PCS_dcdcBusStatus (0x2B4 / 692)
  // PCS_dcdcLvBusVolt:      bits 0-9 (10 bits), scale 0.0390625
  // PCS_dcdcHvBusVolt:      bits 10-21 (12 bits), scale 0.146484375
  // PCS_dcdcLvOutputCurrent: bits 24-35 (12 bits), scale 0.1
  // 
  // NOTE: This message provides accurate voltage/current on 2017 PCS
  // where 0x2C4 (mux 0xE6/0xC6) returns 0.0V for LV voltage.
  // We update both dcdc_bus_status (for debugging) and the primary
  // voltage_data/dcdc_status structs (for parameters).
  uint8_t* bytes = (uint8_t*)data;

  // Extract 10-bit LV voltage (bits 0-9)
  uint16_t raw_lv_volt = (bytes[1] << 8 | bytes[0]) & 0x3FF;
  float lv_voltage = raw_lv_volt * 0.0390625f;
  dcdc_bus_status.lv_bus_voltage_v = lv_voltage;
  voltage_data.lv_v = lv_voltage;  // Update primary LV voltage

  // Extract 12-bit HV voltage (bits 10-21)
  uint16_t raw_hv_volt = ((bytes[2] << 8 | bytes[1]) >> 2) & 0xFFF;
  float hv_voltage = raw_hv_volt * 0.146484375f;
  dcdc_bus_status.hv_bus_voltage_v = hv_voltage;
  voltage_data.hv_v = (uint16_t)hv_voltage;  // Update primary HV voltage

  // Extract 12-bit LV current (bits 24-35)
  uint16_t raw_lv_current = ((bytes[4] << 8 | bytes[3]) >> 0) & 0xFFF;
  float lv_current = raw_lv_current * 0.1f;
  dcdc_bus_status.lv_output_current_a = lv_current;
  dcdc_status.current_a = lv_current;  // Update primary DCDC current
  dcdc_status.power_w = lv_current * lv_voltage;  // Update power calculation
}

void PCSCan::handle2C4(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  mux_state.mux_2c4 = (bytes[0]);

  if ((mux_state.mux_2c4 == 0xE6) || (mux_state.mux_2c4 == 0xC6)) {
    voltage_data.hv_v = (((bytes[3] << 8 | bytes[2]) & 0xFFF) * 0.146484f);
    voltage_data.lv_v = ((((bytes[1] << 9 | bytes[0]) >> 6)) * 0.0390625f);
    mux_state.backup_2c4 = false;
  }
  else if ((mux_state.mux_2c4 == 0x04) && (mux_state.backup_2c4)) {
    voltage_data.hv_v = ((((bytes[7] << 8 | bytes[6]) >> 3) & 0xFFF) * 0.146484f);
  }

  mux_state.mux_2c4 = (bytes[0] & 0x1F);
  if (mux_state.mux_2c4 == 0x00) {
    dc_current_data.phase_a_a = ((bytes[4]) * 0.1f);
    mux_state.got_dci = true;
  }
  else if (mux_state.mux_2c4 == 0x01) {
    dc_current_data.phase_b_a = ((bytes[4]) * 0.1f);
    mux_state.got_dci = true;
  }
  else if (mux_state.mux_2c4 == 0x02) {
    dc_current_data.phase_c_a = ((bytes[4]) * 0.1f);
    mux_state.got_dci = true;
  }
  dc_current_data.total_a = dc_current_data.phase_a_a + dc_current_data.phase_b_a + dc_current_data.phase_c_a;
}

void PCSCan::handle3A4(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  // Alert page (not heavily used in reference)
}

void PCSCan::handle424(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t alert_id = bytes[0];

  #if DEBUG_PCS_CAN
  DEBUG_SERIAL.printf("PCS Alert 0x%02X received\r\n", alert_id);
  #endif

  // Special handling for CAN rationality alert (0x1E = alert 30)
  // Extract which CAN message caused the error and what the error was
  if (alert_id == 0x1E) {
    uint8_t error_detail = bytes[2] & 0x07;
    uint16_t error_can_id = (bytes[4] << 8) | bytes[3];
    DEBUG_SERIAL.printf("  CAN Rationality: ID=0x%03X err=%d", error_can_id, error_detail);
    if (error_detail == 0x01) DEBUG_SERIAL.println(" (msg too long)");
    else if (error_detail == 0x02) DEBUG_SERIAL.println(" (msg too short)");
    else if (error_detail == 0x03) DEBUG_SERIAL.println(" (msg missing)");
    else DEBUG_SERIAL.println();

    // Adaptive message format handling (matches old firmware ProcessCANRat)
    if (error_can_id == 0x2B2) {
      if (error_detail == 0x01) {
        // Message too long - switch to short (3-byte) format
        DEBUG_SERIAL.println("  Switching 0x2B2 to short format");
        Param::SetInt(Param::pcstype, 0);
      } else if (error_detail == 0x02) {
        // Message too short - switch to long (5-byte) format
        DEBUG_SERIAL.println("  Switching 0x2B2 to long format");
        Param::SetInt(Param::pcstype, 1);
      }
    }
  }

  // Map PCS alert ID to error message and post it
  // PCS alerts are numbered 0x01-0x6C (1-108)
  ERROR_MESSAGE_NUM error = ERROR_NONE;
  if (alert_id >= 0x01 && alert_id <= 0x6C) {
    error = (ERROR_MESSAGE_NUM)(ERR_PCS_a001_chgHwInputOc + (alert_id - 0x01));
    ErrorMessage::Post(error);
  }
  else {
    DEBUG_SERIAL.printf("  Unknown PCS alert ID: 0x%02X\r\n", alert_id);
  }
}

void PCSCan::handle504(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  // Boot ID counter in bytes[7] - not currently used
}

void PCSCan::handle76C(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t mux = (bytes[0]);

  if (!mux_state.got_dci) {
    if (mux == 0x0C) {
      dc_current_data.phase_a_a = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }
    else if (mux == 0x16) {
      dc_current_data.phase_b_a = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }
    else if (mux == 0x20) {
      dc_current_data.phase_c_a = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }
    dc_current_data.total_a = dc_current_data.phase_a_a + dc_current_data.phase_b_a + dc_current_data.phase_c_a;
  }
}

// Helper function to log TX messages
static void log_tx_message(uint16_t id, const uint8_t* data, uint8_t len) {
  #if DEBUG_PCS_TX
  DEBUG_SERIAL.print("IPC TX: 0x");
  if (id < 0x100) DEBUG_SERIAL.print("0");
  if (id < 0x10) DEBUG_SERIAL.print("0");
  DEBUG_SERIAL.print(id, HEX);
  DEBUG_SERIAL.print(" [");
  DEBUG_SERIAL.print(len);
  DEBUG_SERIAL.print("] ");
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) DEBUG_SERIAL.print("0");
    DEBUG_SERIAL.print(data[i], HEX);
    DEBUG_SERIAL.print(" ");
  }
  DEBUG_SERIAL.println();
  #endif
}

// CAN Message Transmission Methods (from reference implementation)

void PCSCan::Msg13D() {
  if (can_bus == nullptr) return;

  uint8_t bytes[6];
  // 0x05 = charger enabled, 0x0A = charger disabled (per old firmware comments)
  if (charge_enable) bytes[0] = 0x05;
  if (!charge_enable) bytes[0] = 0x0A;
  bytes[1] = control_params.ac_current_limit_a * 2;
  bytes[2] = 0xAA;
  bytes[3] = 0x1A;
  bytes[4] = 0xFF;
  bytes[5] = 0x02;
  if (!can_bus->sendMessage(0x13D, bytes, 6)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x13D");
  }
}

void PCSCan::Msg20A() {
  if (can_bus == nullptr) return;

  uint8_t bytes[6];
  bytes[0] = 0xF6;
  bytes[1] = 0x15;
  bytes[2] = 0x09;
  bytes[3] = 0x82;
  bytes[4] = 0x18;
  bytes[5] = 0x01;
  if (!can_bus->sendMessage(0x20A, bytes, 6)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x20A");
  }
}

void PCSCan::Msg212() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  bytes[0] = 0xB9;
  bytes[1] = 0x1C;
  bytes[2] = 0x94;
  bytes[3] = 0xAD;
  bytes[4] = 0xC3;
  bytes[5] = 0x15;
  bytes[6] = 0x06;
  bytes[7] = 0x63;
  log_tx_message(0x212, bytes, 8);
  if (!can_bus->sendMessage(0x212, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x212");
  }
}

void PCSCan::Msg21D() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  bytes[0] = 0x2D;
  bytes[1] = control_params.evse_limit_a * 2;
  bytes[2] = 0x00;
  bytes[3] = control_params.cable_limit;
  bytes[4] = 0x80;
  bytes[5] = 0x00;
  bytes[6] = 0x60;
  bytes[7] = 0x10;
  if (!can_bus->sendMessage(0x21D, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x21D");
  }
}

void PCSCan::Msg22A() {
  if (can_bus == nullptr) return;

  uint8_t bytes[4];
  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = (control_params.hv_voltage_v & 0xF) << 4 | current_mode;
  bytes[3] = (control_params.hv_voltage_v >> 4) & 0xFF;
  log_tx_message(0x22A, bytes, 4);
  if (!can_bus->sendMessage(0x22A, bytes, 4)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x22A");
  }
}

void PCSCan::Msg232() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  bytes[0] = 0x0A;
  bytes[1] = 0x02;
  bytes[2] = 0xD5;
  bytes[3] = 0x09;
  bytes[4] = 0xCB;
  bytes[5] = 0x04;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  if (!can_bus->sendMessage(0x232, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x232");
  }
}

void PCSCan::Msg23D() {
  if (can_bus == nullptr) return;

  uint8_t bytes[4];
  // 0x05 = charger enabled, 0x0A = charger disabled (per old firmware comments)
  if (charge_enable) bytes[0] = 0x05;
  if (!charge_enable) bytes[0] = 0x0A;
  bytes[1] = control_params.ac_current_limit_a * 2;
  bytes[2] = 0xFF;
  bytes[3] = 0x0F;
  if (!can_bus->sendMessage(0x23D, bytes, 4)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x23D");
  }
}

void PCSCan::Msg25D() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  bytes[0] = 0xD9;
  bytes[1] = 0x8C;
  bytes[2] = 0x01;
  bytes[3] = 0xB5;
  bytes[4] = 0x4A;
  bytes[5] = 0xC1;
  bytes[6] = 0x0A;
  bytes[7] = 0xE0;
  log_tx_message(0x25D, bytes, 8);
  if (!can_bus->sendMessage(0x25D, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x25D");
  }
}

void PCSCan::Msg2B2(uint16_t charge_power_w) {
  if (can_bus == nullptr) return;

  // Check pcstype parameter: 0=US (3-byte), 1=EU (5-byte)
  bool use_long_format = (Param::GetInt(Param::pcstype) == 1);

  if (use_long_format) {
    // 5-byte variant (EU version - newer firmware)
    uint8_t bytes[5];
    bytes[0] = charge_power_w & 0xFF;
    bytes[1] = charge_power_w >> 8;
    bytes[2] = charge_enable ? 0x02 : 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    log_tx_message(0x2B2, bytes, 5);
    if (!can_bus->sendMessage(0x2B2, bytes, 5)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x2B2");
    }
  } else {
    // 3-byte variant (US version - older firmware)
    uint8_t bytes[3];
    bytes[0] = charge_power_w & 0xFF;
    bytes[1] = charge_power_w >> 8;
    bytes[2] = charge_enable ? 0x02 : 0x00;
    log_tx_message(0x2B2, bytes, 3);
    if (!can_bus->sendMessage(0x2B2, bytes, 3)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x2B2");
    }
  }
}

void PCSCan::Msg321() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  bytes[0] = 0x2C;
  bytes[1] = 0xB6;
  bytes[2] = 0xA8;
  bytes[3] = 0x7F;
  bytes[4] = 0x02;
  bytes[5] = 0x7F;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  log_tx_message(0x321, bytes, 8);
  if (!can_bus->sendMessage(0x321, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x321");
  }
}

void PCSCan::Msg333() {
  if (can_bus == nullptr) return;

  uint8_t bytes[4];
  bytes[0] = 0x04;
  bytes[1] = 0x30;
  bytes[2] = 0x29;
  bytes[3] = 0x07;
  log_tx_message(0x333, bytes, 4);
  if (!can_bus->sendMessage(0x333, bytes, 4)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x333");
  }
}

void PCSCan::Msg3A1() {
  if (can_bus == nullptr) return;

  uint16_t dcdc_spnt = control_params.dcdc_voltage_v * 100.0f;
  uint8_t bytes[8];
  bytes[0] = 0x09;
  bytes[1] = 0x62;
  bytes[2] = dcdc_spnt & 0xFF;
  bytes[3] = ((dcdc_spnt >> 8) | 0x99);
  bytes[4] = 0x08;
  bytes[5] = 0x2C;
  bytes[6] = 0x12;
  bytes[7] = 0x5A;
  log_tx_message(0x3A1, bytes, 8);
  if (!can_bus->sendMessage(0x3A1, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x3A1");
  }
}

void PCSCan::Msg3B2() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  if (mux_state.mux_3b2) {
    bytes[0] = 0xE5;
    bytes[1] = 0x0D;
    bytes[2] = 0xEB;
    bytes[3] = 0xFF;
    bytes[4] = 0x0C;
    bytes[5] = 0x66;
    bytes[6] = 0xBB;
    bytes[7] = 0x11;
    if (!can_bus->sendMessage(0x3B2, bytes, 8)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x3B2");
    }
    mux_state.mux_3b2 = false;
  }
  else {
    bytes[0] = 0xE3;
    bytes[1] = 0x5D;
    bytes[2] = 0xFB;
    bytes[3] = 0xFF;
    bytes[4] = 0x0C;
    bytes[5] = 0x66;
    bytes[6] = 0xBB;
    bytes[7] = 0x06;
    if (!can_bus->sendMessage(0x3B2, bytes, 8)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x3B2");
    }
    mux_state.mux_3b2 = true;
  }
}

void PCSCan::Msg545() {
  if (can_bus == nullptr) return;

  uint8_t bytes[8];
  if (mux_state.mux_545) {
    bytes[0] = 0x14;
    bytes[1] = 0x00;
    bytes[2] = 0x3F;
    bytes[3] = 0x70;
    bytes[4] = 0x9F;
    bytes[5] = 0x01;
    bytes[6] = (mux_state.count_545 << 4) | 0xA;
    bytes[7] = calc_checksum(bytes, 0x545);
    if (!can_bus->sendMessage(0x545, bytes, 8)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x545");
    }
    mux_state.mux_545 = false;
  }
  else {
    bytes[0] = 0x03;
    bytes[1] = 0x19;
    bytes[2] = 0x64;
    bytes[3] = 0x32;
    bytes[4] = 0x19;
    bytes[5] = 0x00;
    bytes[6] = (mux_state.count_545 << 4);
    bytes[7] = calc_checksum(bytes, 0x545);
    if (!can_bus->sendMessage(0x545, bytes, 8)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x545");
    }
    mux_state.mux_545 = true;
  }
  mux_state.count_545++;
  if (mux_state.count_545 > 0x0F) mux_state.count_545 = 0;
}

// Helper Functions

uint8_t PCSCan::calc_checksum(uint8_t *bytes, uint16_t id) {
  uint16_t checksum_calc = 0;
  for (int b = 0; b < 7; b++) {
    checksum_calc = checksum_calc + bytes[b];
  }
  checksum_calc += id + (id >> 8);
  checksum_calc &= 0xFF;
  return (uint8_t)checksum_calc;
}

// Convert 11-bit signed raw value to temperature in °C
// DBC: scale 0.1, offset +40
// Formula: temp_C = raw * 0.1 + 40
// Special value 1023 (0x3FF) = SNA (Signal Not Available)
float PCSCan::convert_temp_11bit(uint16_t raw) {
  // Check for SNA value
  if ((raw & 0x7FF) == 0x3FF) {
    return -999.0f;  // Return invalid marker
  }

  // Sign extend 11-bit to 16-bit (2's complement)
  int16_t signed_val = raw & 0x7FF;
  if (signed_val & 0x400) {
    signed_val |= 0xF800;  // Sign extend
  }

  // Apply DBC conversion: scale 0.1, offset +40
  return (signed_val * 0.1f) + 40.0f;
}
