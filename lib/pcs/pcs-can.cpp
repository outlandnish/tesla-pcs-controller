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
  .hw_type = PCS_HW_11KW,
  .status = PCS_STATUS_INIT,
  .grid_config = PCS_GRID_NONE,
  .power_available_kw = 0.0f
};

DCDCStatus PCSCan::dcdc_status = {
  .current_a = 0.0f,
  .power_w = 0.0f
};

ACStatus PCSCan::ac_status = {
  .current_limit_a = 0,
  .power_kw = 0.0f,
  .voltage_v = 0,
  .current_a = 0.0f
};

TemperatureData PCSCan::temperature_data = {
  .local_c = 0,
  .ambient_raw = 0,
  .phase_a_raw = 0,
  .phase_b_raw = 0,
  .phase_c_raw = 0,
  .dcdc_raw = 0,
  .dcdc_b_c = 0.0f
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

  static uint32_t lastDebugTime = 0;
  static uint32_t messageCount = 0;
  
  uint32_t can_id;
  uint8_t data[8];
  uint8_t len;

  while (can_bus->receiveMessage(can_id, data, len)) {
    messageCount++;
    
    uint32_t data_words[2];
    memcpy(data_words, data, 8);
    process_frame(can_id, data_words);
  }
  
  // Debug: Print message count and error stats every 5 seconds
  #if DEBUG_PCS_CAN
  if (millis() - lastDebugTime > 5000) {
    DEBUG_SERIAL.printf("CAN1 (IPC): PCS_Enable=%s\n", PCSController::is_pcs_enabled() ? "true" : "false");
    
    // Get error counters
    uint8_t txErrors, rxErrors;
    can_bus->getErrorCounters(txErrors, rxErrors);
    
    if (messageCount > 0) {
      DEBUG_SERIAL.printf("CAN1 (IPC): Received %lu messages in last 5 sec (TX_ERR=%d, RX_ERR=%d)\n", 
        messageCount, txErrors, rxErrors);
    } else {
      DEBUG_SERIAL.printf("CAN1 (IPC): No messages received (TX_ERR=%d, RX_ERR=%d)\n", txErrors, rxErrors);
      
      // Show detailed status if errors detected or no messages
      if (txErrors > 0 || rxErrors > 0) {
        DEBUG_SERIAL.println("CAN1 (IPC) Status:");
        can_bus->printStatus();
      } else {
        DEBUG_SERIAL.println("  PCS may be offline or not transmitting");
      }
    }
    
    // Reset counters
    messageCount = 0;
    lastDebugTime = millis();
  }
  #endif
}

void PCSCan::process_frame(uint32_t can_id, uint32_t data[2]) {
  log_rx_message(can_id, data);

  switch (can_id) {
    case 0x204: handle204(data); break;
    case 0x224: handle224(data); break;
    case 0x264: handle264(data); break;
    case 0x2A4: handle2A4(data); break;
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
  uint8_t* bytes = (uint8_t*)data;
  charger_status.hw_type = (PCSHardwareType)((bytes[7] >> 3) & 0x03);
  charger_status.status = (PCSChargeStatus)((bytes[0]) & 0x0f);
  charger_status.power_available_kw = bytes[3] * 0.1f;
  charger_status.grid_config = (PCSGridConfig)((bytes[0] >> 6) & 0x3);
}

void PCSCan::handle224(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  dcdc_status.current_a = (((bytes[3] << 8 | bytes[2]) & 0xFFF) * 0.1f);
  dcdc_status.power_w = dcdc_status.current_a * voltage_data.lv_v;
}

void PCSCan::handle264(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  ac_status.current_limit_a = (((bytes[5] << 8 | bytes[4]) & 0x3ff) * 0.1f);
  ac_status.power_kw = ((bytes[3]) * 0.1f);
  ac_status.voltage_v = (((bytes[1] << 8 | bytes[0]) & 0x3FFF) * 0.033f);
  ac_status.current_a = (((bytes[2] << 9 | bytes[1]) >> 7) * 0.1f);
}

void PCSCan::handle2A4(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  temperature_data.phase_a_raw = ((bytes[1] << 8 | bytes[0]));
  temperature_data.phase_b_raw = ((bytes[2] << 8 | bytes[1]) >> 3);
  temperature_data.phase_c_raw = ((bytes[4] << 15 | bytes[3] << 7 | bytes[2] >> 1) >> 5);
  temperature_data.dcdc_raw = ((bytes[5] << 8 | bytes[4]) >> 1);
  temperature_data.dcdc_b_c = (((bytes[7] << 8 | bytes[6]) >> 7) & 0x1FF) * 0.293542f;
  temperature_data.ambient_raw = ((bytes[6] << 8 | bytes[5]) >> 4);

  // Process temperatures
  temperature_data.local_c = process_temp(temperature_data.phase_a_raw);
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

int16_t PCSCan::process_temp(uint16_t in_val) {
  int16_t value = in_val & 0x3ff;
  if (in_val & 0x400) value -= 0x3ff;
  value = value * 0.1f + 40;
  return value;
}
