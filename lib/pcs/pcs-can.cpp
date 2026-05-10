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
  .cable_limit = 0,
  .charge_termination_pct = 80.0f
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
  // PCS_dcdcMaxOutputCurrentAllowed: pos=29, w=12, scale=0.1
  // bits29-40: byte3 bits5-7, byte4 bits0-7, byte5 bit0
  dcdc_status.current_a = (((bytes[5] << 16 | bytes[4] << 8 | bytes[3]) >> 5) & 0xFFF) * 0.1f;
  dcdc_status.power_w = dcdc_status.current_a * voltage_data.lv_v;
}

void PCSCan::handle264(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  ac_status.current_limit_a = (((bytes[5] << 8 | bytes[4]) & 0x3ff) * 0.1f);
  ac_status.power_kw = ((bytes[3]) * 0.1f);
  ac_status.voltage_v = (((bytes[1] << 8 | bytes[0]) & 0x3FFF) * 0.033f);
  // PCS_chgLineCurrent: pos=14, w=9, scale=0.1 — bits14-22: byte1 bits6-7, byte2 bits0-6
  ac_status.current_a = (((bytes[2] << 2 | bytes[1] >> 6) & 0x1FF) * 0.1f);
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

  // HVP_contactorState — signal layout from Model3_ETH.compact.json
  // Contactor states: neg@0(3b), pos@3(3b), packSetState@8(4b),
  //   packCtrsClosingAllowed@35(1b), dcLinkAllowedToEnergize@36(1b), hvilStatus@40(4b)
  uint64_t word = 0;
  word |= (uint64_t)CONTACTOR_STATE_ECONOMIZED << 0;   // packContNegativeState
  word |= (uint64_t)CONTACTOR_STATE_ECONOMIZED << 3;   // packContPositiveState
  word |= (uint64_t)CONTACTOR_SET_STATE_CLOSED << 8;   // packContactorSetState
  word |= (uint64_t)1 << 35;  // packCtrsClosingAllowed
  word |= (uint64_t)1 << 36;  // dcLinkAllowedToEnergize
  word |= (uint64_t)HVIL_STATUS_OK << 40;              // hvilStatus
  uint8_t bytes[6];
  for (int i = 0; i < 6; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  if (!can_bus->sendMessage(0x20A, bytes, 6)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x20A");
  }
}

void PCSCan::Msg212() {
  if (can_bus == nullptr) return;

  // BMS_status — signal layout from Model3_ETH.compact.json
  // hvacPowerRequest@0, updateAllowed@4, pcsPwmEnabled@7,
  // contactorState@8(3b), uiChargeStatus@11(3b), hvState@16(3b),
  // chargeRequest@29, state@32(4b), smStateRequest@56(4b)
  uint64_t word = 0;
  word |= (uint64_t)1 << 0;                           // hvacPowerRequest
  word |= (uint64_t)1 << 4;                           // updateAllowed
  word |= (uint64_t)1 << 7;                           // pcsPwmEnabled
  word |= (uint64_t)BMS_CTRSET_CLOSED << 8;           // contactorState
  word |= (uint64_t)BMS_CHARGING << 11;               // uiChargeStatus
  word |= (uint64_t)HV_UP_FOR_CHARGE << 16;           // hvState
  word |= (uint64_t)1 << 29;                          // chargeRequest
  word |= (uint64_t)BMS_CHARGE << 32;                 // state
  word |= (uint64_t)BMS_CHARGE << 56;                 // smStateRequest
  uint8_t bytes[8];
  for (int i = 0; i < 8; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  log_tx_message(0x212, bytes, 8);
  if (!can_bus->sendMessage(0x212, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x212");
  }
}

void PCSCan::Msg21D() {
  if (can_bus == nullptr) return;

  // CP_evseStatus — signal layout from Model3_ETH.compact.json
  // evseAccept@0, proximity@2(2b), pilot@4(3b), pilotCurrent@8(8b,scale=0.5),
  // cableCurrentLimit@24(7b), evseChargeType_UI@38(2b), acChargeState@53(3b)
  uint64_t word = 0;
  word |= (uint64_t)1 << 0;                                           // evseAccept
  word |= (uint64_t)CHG_PROXIMITY_LATCHED << 2;                       // proximity
  word |= (uint64_t)CHG_PILOT_LINE_CHARGE << 4;                       // pilot
  word |= (uint64_t)(uint8_t)(control_params.evse_limit_a / 0.5f) << 8;  // pilotCurrent
  word |= (uint64_t)(control_params.cable_limit & 0x7F) << 24;        // cableCurrentLimit
  word |= (uint64_t)AC_CHARGER_PRESENT << 38;                         // evseChargeType_UI
  word |= (uint64_t)AC_CHARGE_ENABLED << 53;                          // acChargeState
  uint8_t bytes[8];
  for (int i = 0; i < 8; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  if (!can_bus->sendMessage(0x21D, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x21D");
  }
}

void PCSCan::Msg22A() {
  if (can_bus == nullptr) return;

  // HVP_pcsControl — signal layout from Model3_ETH.compact.json
  // dcLinkVoltageRequest@0(16b,scale=0.1,signed), pcsControlRequest@16(2b),
  // pcsChargeHwEnabled@18(1b), pcsDcdcHwEnabled@19(1b)
  bool charge_hw = (current_mode == PCS_MODE_CHARGE_ONLY || current_mode == PCS_MODE_CHARGE_DCDC);
  bool dcdc_hw   = (current_mode == PCS_MODE_DCDC_ONLY   || current_mode == PCS_MODE_CHARGE_DCDC);
  HvpPcsControlRequest ctrl = (current_mode == PCS_MODE_OFF) ? HVP_PCS_CTRL_SHUTDOWN : HVP_PCS_CTRL_SUPPORT;
  int16_t v_raw = (int16_t)(control_params.hv_voltage_v / 0.1f);
  uint32_t word = (uint16_t)v_raw;
  word |= (uint32_t)ctrl << 16;
  word |= (uint32_t)(charge_hw ? 1 : 0) << 18;
  word |= (uint32_t)(dcdc_hw ? 1 : 0) << 19;
  uint8_t bytes[4];
  for (int i = 0; i < 4; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
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

  // CP_chargeStatus — not in Model3_ETH.compact.json; layout from pcs_send.py (confirmed working):
  // hvChargeStatus@0(3b): 5=CP_CHARGE_ENABLED, acChargeCurrentLimit@8(8b,scale=0.5)
  uint32_t word = 0;
  word |= 5 << 0;  // hvChargeStatus=CP_CHARGE_ENABLED
  word |= (uint32_t)(uint8_t)(control_params.ac_current_limit_a / 0.5f) << 8;
  uint8_t bytes[4];
  for (int i = 0; i < 4; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  if (!can_bus->sendMessage(0x23D, bytes, 4)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x23D");
  }
}

void PCSCan::Msg25D() {
  if (can_bus == nullptr) return;

  // Fixed payload confirmed working in pcs_send.py
  uint8_t bytes[8] = { 0xD8, 0x8C, 0x01, 0xB5, 0x4A, 0xC1, 0x0A, 0xE0 };
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

  // UI_chargeRequest — signal layout from Model3_ETH.compact.json
  // chargeEnableRequest@2(1b), acChargeCurrentLimit@8(7b,amps integer),
  // chargeTerminationPct@16(10b,scale=0.1)
  uint16_t term_raw = (uint16_t)(control_params.charge_termination_pct / 0.1f);
  uint32_t word = 0;
  word |= (uint32_t)1 << 2;                                          // chargeEnableRequest
  word |= (uint32_t)(control_params.ac_current_limit_a & 0x7F) << 8; // acChargeCurrentLimit
  word |= (uint32_t)(term_raw & 0x3FF) << 16;                        // chargeTerminationPct
  uint8_t bytes[4];
  for (int i = 0; i < 4; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  log_tx_message(0x333, bytes, 4);
  if (!can_bus->sendMessage(0x333, bytes, 4)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x333");
  }
}

void PCSCan::Msg3A1() {
  if (can_bus == nullptr) return;

  // VCFRONT_vehicleStatus — signal layout from Model3_ETH.compact.json
  // bmsHvChargeEnable@0, inAccessoryPlus(accPlusAvailable)@9, 12vStatusForDrive@14(2b),
  // pcs12vVoltageTarget@16(11b,scale=0.01), vehicleStatusCounter@52(4b),
  // vehicleStatusChecksum@56(8b) = sum(bytes[0:7]) & 0xFF
  uint16_t v_raw = (uint16_t)(control_params.dcdc_voltage_v / 0.01f) & 0x7FF;
  uint64_t word = 0;
  word |= (uint64_t)1 << 0;                                    // bmsHvChargeEnable
  word |= (uint64_t)1 << 9;                                    // accPlusAvailable
  word |= (uint64_t)READY_FOR_DRIVE_12V << 14;                 // 12vStatusForDrive
  word |= (uint64_t)v_raw << 16;                               // pcs12vVoltageTarget
  word |= (uint64_t)(mux_state.count_3a1 & 0xF) << 52;        // vehicleStatusCounter
  uint8_t bytes[8];
  for (int i = 0; i < 7; i++) bytes[i] = (word >> (8 * i)) & 0xFF;
  bytes[7] = 0;
  uint8_t csum = 0;
  for (int i = 0; i < 7; i++) csum += bytes[i];
  bytes[7] = csum;
  mux_state.count_3a1 = (mux_state.count_3a1 + 1) & 0xF;
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
  // 11-bit signed, scale=0.1, offset=40 (PCS_thermalStatus signals)
  int16_t value = in_val & 0x3FF;
  if (in_val & 0x400) value -= 0x400;
  return (int16_t)(value * 0.1f + 40);
}
