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

#include "pcs-can.h"
#include "pcs.h"  // For PCSController update methods
#include "param_prj.h"
#include "params.h"
#include "debug_serial.h"
#include "errormessage.h"

#ifndef DEBUG_PCS_CAN
#define DEBUG_PCS_CAN 1
#endif

#ifndef DEBUG_PCS_TX
#define DEBUG_PCS_TX 0
#endif

#ifndef DEBUG_PCS_RX
#define DEBUG_PCS_RX 0
#endif

// ==================== STATIC MEMBER DEFINITIONS (transport only) ====================

CANBus* PCSCan::can_bus = nullptr;
PCSMode PCSCan::current_mode = PCS_MODE_DCDC_ONLY;  // CRITICAL: DCDC must be enabled for interface to respond
bool PCSCan::charge_enable = false;
uint8_t PCSCan::clear_faults_counter = 0;
bool PCSCan::use_long_msg_format = true;

ControlParams PCSCan::control_params = {
  .hv_voltage_v = UDCSPNT_DEFAULT,
  .charge_power_w = 0,
  .dcdc_voltage_v = UDCDC_DEFAULT,
  .ac_current_limit_a = IACLIM_DEFAULT,
  .evse_limit_a = 0,
  .cable_limit = 0,
  .charge_termination_percent = CHGTERMN_DEFAULT
};

MuxState PCSCan::mux_state = {
  .mux_3b2 = false,
  .mux_545 = false,
  .count_545 = 0,
  .count_3a1 = 0,
  .mux_2c4 = 0,
  .got_dci = false
};

// ==================== HELPER FUNCTIONS ====================

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

// ==================== INITIALIZATION ====================

void PCSCan::begin(CANBus *ipc_can_bus) {
  can_bus = ipc_can_bus;
  DEBUG_SERIAL.println("PCSCan: Low-level CAN initialized");
}

// ==================== MESSAGE PROCESSING ====================

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
    case 0x3C4: handle3C4(data); break;
    case 0x3E4: handle3E4(data); break;
    case 0x424: handle424(data); break;
    case 0x504: handle504(data); break;
    case 0x76C: handle76C(data); break;
    default: break;
  }
}

// ==================== RX HANDLERS (push data to PCSController) ====================

void PCSCan::handle204(uint32_t data[2]) {
  // DBC: PCS_chgStatus (0x204 / 516)
  uint8_t* bytes = (uint8_t*)data;

  ChargerStatus status;
  status.main_state = (PCSChargeStatus)(bytes[0] & 0x0F);
  status.charge_status = (ChargeStatusFlag)((bytes[0] >> 4) & 0x03);
  status.grid_config = (PCSGridConfig)((bytes[0] >> 6) & 0x03);
  status.phase_a_enabled = (bytes[1] >> 0) & 0x01;
  status.phase_b_enabled = (bytes[1] >> 1) & 0x01;
  status.phase_c_enabled = (bytes[1] >> 2) & 0x01;
  status.instant_power_available_kw = bytes[2] * 0.1f;
  status.max_power_available_kw = bytes[3] * 0.1f;
  status.phase_a_current_request_a = bytes[4] * 0.1f;
  status.phase_b_current_request_a = bytes[5] * 0.1f;
  status.phase_c_current_request_a = bytes[6] * 0.1f;
  status.pwm_enable = (bytes[7] >> 0) & 0x01;

  PCSController::update_charger_status(status);
}

void PCSCan::handle224(uint32_t data[2]) {
  // DBC: PCS_dcdcStatus (0x224 / 548)
  uint8_t* bytes = (uint8_t*)data;

  DCDCStatus status;
  status.current_a = 0.0f;  // Updated from 0x2B4
  status.power_w = 0.0f;    // Updated from 0x2B4
  status.precharge_status = (DCDCStatusFlag)(bytes[0] & 0x03);
  status.support_12v_status = (DCDCStatusFlag)((bytes[0] >> 2) & 0x03);
  status.hvbus_discharge_status = (DCDCStatusFlag)((bytes[0] >> 4) & 0x03);
  status.main_state = (DCDCMainState)((bytes[1] << 2 | bytes[0] >> 6) & 0x0F);
  status.sub_state = (bytes[1] >> 2) & 0x1F;
  status.faulted = (bytes[1] >> 7) & 0x01;
  status.output_limited = (bytes[3] >> 4) & 0x01;
  uint16_t raw_max_current = ((bytes[5] << 8 | bytes[4]) >> 5) & 0xFFF;
  status.max_output_current_a = raw_max_current * 0.1f;
  status.pwm_enable = (bytes[6] >> 4) & 0x01;
  status.supporting_fixed_lv = (bytes[6] >> 5) & 0x01;

  PCSController::update_dcdc_status(status);
}

void PCSCan::handle264(uint32_t data[2]) {
  // DBC: PCS_chgLineStatus (0x264 / 612)
  uint8_t* bytes = (uint8_t*)data;

  ACStatus status;
  status.voltage_v = ((bytes[1] << 8 | bytes[0]) & 0x3FFF) * 0.033f;
  status.current_a = (((bytes[2] << 8 | bytes[1]) >> 6) & 0x1FF) * 0.1f;
  status.power_kw = bytes[3] * 0.1f;
  status.current_limit_a = ((bytes[5] << 8 | bytes[4]) & 0x3FF) * 0.1f;

  PCSController::update_ac_status(status);
}

void PCSCan::handle2A4(uint32_t data[2]) {
  // DBC: PCS_thermalStatus (0x2A4 / 676)
  uint8_t* bytes = (uint8_t*)data;

  uint16_t raw_a = (bytes[1] << 8 | bytes[0]) & 0x7FF;
  uint16_t raw_b = ((bytes[2] << 8 | bytes[1]) >> 3) & 0x7FF;
  uint16_t raw_c = ((bytes[4] << 8 | bytes[3]) >> 0) & 0x7FF;
  uint16_t raw_dcdc = ((bytes[5] << 8 | bytes[4]) >> 3) & 0x7FF;
  uint16_t raw_ambient = ((bytes[7] << 8 | bytes[6]) >> 0) & 0x7FF;

  TemperatureData temp;
  temp.phase_a_c = convert_temp_11bit(raw_a);
  temp.phase_b_c = convert_temp_11bit(raw_b);
  temp.phase_c_c = convert_temp_11bit(raw_c);
  temp.dcdc_c = convert_temp_11bit(raw_dcdc);
  temp.ambient_c = convert_temp_11bit(raw_ambient);

  PCSController::update_temperature_data(temp);
}

void PCSCan::handle2B4(uint32_t data[2]) {
  // JSON: PCS_dcdcRailStatus (0x2B4) - 6 bytes
  // PCS_dcdcLvBusVolt:      bit 0,  13-bit unsigned, scale=0.01 V
  // PCS_dcdcLvOutputCurrent: bit 32, 13-bit signed,   scale=0.1  A
  uint8_t* bytes = (uint8_t*)data;

  uint16_t raw_lv_volt = ((uint16_t)bytes[1] << 8 | bytes[0]) & 0x1FFF;
  float lv_voltage = raw_lv_volt * 0.01f;

  int16_t raw_lv_current = ((uint16_t)bytes[5] << 8 | bytes[4]) & 0x1FFF;
  if (raw_lv_current & 0x1000) raw_lv_current |= (int16_t)0xE000;  // sign-extend 13→16 bit
  float lv_current = raw_lv_current * 0.1f;

  DCDCBusStatus bus_status;
  bus_status.lv_bus_voltage_v = lv_voltage;
  bus_status.hv_bus_voltage_v = 0.0f;  // not in this message
  bus_status.lv_output_current_a = lv_current;
  PCSController::update_dcdc_bus_status(bus_status);

  PCSController::update_lv_voltage(lv_voltage);

  PCSController::update_dcdc_current(lv_current, lv_voltage);
}

void PCSCan::handle2C4(uint32_t data[2]) {
  // DBC: PCS_logging (0x2C4 / 708) - Multiplexed
  uint8_t* bytes = (uint8_t*)data;
  mux_state.mux_2c4 = bytes[0] & 0x1F;

  switch (mux_state.mux_2c4) {
    case LOG_PHA_1:
    case LOG_PHB_1:
    case LOG_PHC_1:
    {
      uint16_t raw_input_irms = ((bytes[1] << 8 | bytes[0]) >> 5) & 0x1FF;
      uint16_t raw_int_bus_v = ((bytes[2] << 8 | bytes[1]) >> 6) & 0x1FF;
      uint16_t raw_int_bus_target = ((bytes[3] << 8 | bytes[2]) >> 7) & 0x1FF;
      uint8_t raw_output_i = bytes[4];

      PhaseLoggingData phase_data;
      phase_data.input_current_rms_a = raw_input_irms * 0.1f;
      phase_data.internal_bus_voltage_v = raw_int_bus_v;
      phase_data.internal_bus_target_v = raw_int_bus_target;
      phase_data.output_current_a = raw_output_i * 0.1f;

      uint8_t phase = mux_state.mux_2c4;  // 0, 1, or 2
      PCSController::update_phase_logging(phase, phase_data);
      mux_state.got_dci = true;
      break;
    }

    case LOG_CHG_1:
    {
      uint16_t raw_l1n = ((bytes[2] << 16 | bytes[1] << 8 | bytes[0]) >> 5) & 0xFFF;
      uint16_t raw_l2n = ((bytes[3] << 16 | bytes[2] << 8 | bytes[1]) >> 9) & 0xFFF;
      uint16_t raw_l3n = ((bytes[5] << 16 | bytes[4] << 8 | bytes[3]) >> 5) & 0xFFF;
      uint16_t raw_l1l2 = ((bytes[6] << 16 | bytes[5] << 8 | bytes[4]) >> 9) & 0xFFF;
      uint16_t raw_ng = ((bytes[7] << 8 | bytes[6]) >> 5) & 0x1FF;

      ChargerLineVoltageData line_voltage;
      line_voltage.l1n_voltage_v = raw_l1n * 0.2f;
      line_voltage.l2n_voltage_v = raw_l2n * 0.2f;
      line_voltage.l3n_voltage_v = raw_l3n * 0.2f;
      line_voltage.l1l2_voltage_v = raw_l1l2 * 0.2f;
      line_voltage.ng_voltage_v = raw_ng;

      PCSController::update_charger_line_voltage(line_voltage);
      break;
    }

    case LOG_CHG_2:
    {
      uint16_t raw_freq_l1n = ((bytes[2] << 8 | bytes[1]) >> 0) & 0xFFF;
      uint16_t raw_freq_l2n = ((bytes[3] << 8 | bytes[2]) >> 4) & 0xFFF;
      uint16_t raw_freq_l3n = ((bytes[5] << 8 | bytes[4]) >> 0) & 0xFFF;
      uint8_t raw_phase_config = (bytes[5] >> 4) & 0x07;
      uint16_t raw_output_v = ((bytes[7] << 8 | bytes[6]) >> 0) & 0xFFF;

      ChargerFrequencyData freq_data;
      freq_data.l1n_frequency_hz = raw_freq_l1n * 0.01f + 40.0f;
      freq_data.l2n_frequency_hz = raw_freq_l2n * 0.01f + 40.0f;
      freq_data.l3n_frequency_hz = raw_freq_l3n * 0.01f + 40.0f;
      freq_data.internal_phase_config = raw_phase_config;
      freq_data.output_voltage_v = raw_output_v * 0.146484f;

      PCSController::update_charger_frequency(freq_data);
      break;
    }

    case LOG_CHG_3:
    {
      ChargerPhaseStateData phase_state;
      phase_state.phase_a_state = (ChargerPhaseState)((bytes[0] >> 5) | ((bytes[1] & 0x03) << 3));
      phase_state.phase_b_state = (ChargerPhaseState)((bytes[1] >> 1) & 0x0F);
      phase_state.phase_c_state = (ChargerPhaseState)((bytes[1] >> 5) | ((bytes[2] & 0x01) << 3));
      phase_state.phase_a_shutdown_reason = (ChargerShutdownReason)((bytes[2] >> 1) & 0x1F);
      phase_state.phase_b_shutdown_reason = (ChargerShutdownReason)((bytes[2] >> 6) | ((bytes[3] & 0x07) << 2));
      phase_state.phase_c_shutdown_reason = (ChargerShutdownReason)((bytes[3] >> 3) & 0x1F);
      phase_state.phase_a_retry_count = bytes[4] & 0x07;
      phase_state.phase_b_retry_count = (bytes[4] >> 3) & 0x07;
      phase_state.phase_c_retry_count = ((bytes[4] >> 6) | ((bytes[5] & 0x01) << 2));
      phase_state.charger_retry_count = (bytes[5] >> 1) & 0x07;
      phase_state.l1n_pll_locked = (bytes[6] >> 6) & 0x01;
      phase_state.l2n_pll_locked = (bytes[6] >> 7) & 0x01;
      phase_state.l3n_pll_locked = (bytes[7] >> 0) & 0x01;
      phase_state.l1l2_pll_locked = (bytes[7] >> 1) & 0x01;
      phase_state.ng_pll_locked = (bytes[7] >> 2) & 0x01;

      PCSController::update_charger_phase_state(phase_state);
      break;
    }

    case LOG_DCDC_1:
    {
      uint16_t raw_max_output = ((bytes[4] << 8 | bytes[3]) >> 4) & 0xFFF;
      uint16_t raw_current_limit = ((bytes[6] << 8 | bytes[5]) >> 0) & 0xFFF;
      uint16_t raw_temp_limit = ((bytes[7] << 8 | bytes[6]) >> 4) & 0xFFF;

      DCDCLoggingData dcdc_log;
      dcdc_log.max_lv_output_current_a = raw_max_output * 0.1f;
      dcdc_log.current_limit_a = raw_current_limit * 0.1f;
      dcdc_log.lv_output_temp_limit_a = raw_temp_limit * 0.1f;

      PCSController::update_dcdc_logging(dcdc_log);
      break;
    }

    case LOG_DCDC_2:
    {
      int16_t raw_tank_v = ((bytes[4] << 8 | bytes[3]) >> 2) & 0x7FF;
      if (raw_tank_v & 0x400) raw_tank_v |= 0xF800;  // Sign extend

      uint16_t raw_target_v = ((bytes[5] << 8 | bytes[4]) >> 5) & 0x3FF;
      uint16_t raw_freq = ((bytes[7] << 8 | bytes[6]) >> 0) & 0xFFF;

      DCDCControlData dcdc_ctrl;
      dcdc_ctrl.tank_voltage_v = raw_tank_v;
      dcdc_ctrl.tank_voltage_target_v = raw_target_v;
      dcdc_ctrl.switching_freq_khz = raw_freq * 0.0976563f;

      PCSController::update_dcdc_control(dcdc_ctrl);
      break;
    }

    default:
      break;
  }
}

void PCSCan::handle3A4(uint32_t data[2]) {
  // Explorer: PCS_alertMatrix (0x3A4) - PCS_matrixIndex is 4 bits (bits 0-3),
  // alert bits start at bit 4. Page 0: alerts 1-60, Page 1: alerts 61-116.
  uint8_t* bytes = (uint8_t*)data;
  uint8_t matrix_index = bytes[0] & 0x0F;  // 4-bit index

  PCSController::update_alert_matrix_page(matrix_index, bytes);

  #if DEBUG_PCS_CAN
  // Log alert changes
  static uint8_t last_page0[8] = {};
  static uint8_t last_page1[8] = {};
  uint8_t* last_page = (matrix_index == 0) ? last_page0 : last_page1;

  bool changed = memcmp(last_page, bytes, 8) != 0;
  memcpy(last_page, bytes, 8);

  if (changed) {
    uint8_t page_count = 0;
    for (int i = 4; i < 8; i++) {  // Alert bits start at bit 4
      if (bytes[0] & (1 << i)) page_count++;
    }
    for (int b = 1; b < 8; b++) {
      uint8_t byte_val = bytes[b];
      while (byte_val) {
        page_count += byte_val & 1;
        byte_val >>= 1;
      }
    }

    if (page_count > 0) {
      DEBUG_SERIAL.printf("0x3A4 page %d: %d alerts active [", matrix_index, page_count);
      bool first = true;
      for (int i = 0; i < 60; i++) {
        int bit_pos = i + 4;  // Alert bits start at bit 4
        int byte_idx = bit_pos / 8;
        int bit_idx = bit_pos % 8;
        if (bytes[byte_idx] & (1 << bit_idx)) {
          int alert_num = (matrix_index == 0) ? (i + 1) : (i + 61);
          if (!first) DEBUG_SERIAL.print(",");
          DEBUG_SERIAL.printf("a%03d", alert_num);
          first = false;
        }
      }
      DEBUG_SERIAL.println("]");
    }
  }
  #endif
}

void PCSCan::handle3E4(uint32_t data[2]) {
  // Explorer: PCS2_alertMatrix (0x3E4) - PCS2_matrixIndex is 4 bits (bits 0-3),
  // alert bits start at bit 4. Page 0: alerts 1-60, Page 1: 61-120, Page 2: 121-175.
  // PCS2 is a different hardware generation - alerts logged for diagnostics only.
  #if DEBUG_PCS_CAN
  uint8_t* bytes = (uint8_t*)data;
  uint8_t matrix_index = bytes[0] & 0x0F;  // 4-bit index

  static uint8_t last_page[3][8] = {};
  if (matrix_index < 3) {
    bool changed = memcmp(last_page[matrix_index], bytes, 8) != 0;
    memcpy(last_page[matrix_index], bytes, 8);

    if (changed) {
      uint8_t alert_count = 0;
      for (int i = 4; i < 8; i++) {
        if (bytes[0] & (1 << i)) alert_count++;
      }
      for (int b = 1; b < 8; b++) {
        uint8_t v = bytes[b];
        while (v) { alert_count += v & 1; v >>= 1; }
      }

      if (alert_count > 0) {
        DEBUG_SERIAL.printf("0x3E4 PCS2 page %d: %d alerts active [", matrix_index, alert_count);
        bool first = true;
        for (int i = 0; i < 60; i++) {
          int bit_pos = i + 4;
          int byte_idx = bit_pos / 8;
          int bit_idx = bit_pos % 8;
          if (bytes[byte_idx] & (1 << bit_idx)) {
            int alert_num = matrix_index * 60 + i + 1;
            if (!first) DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.printf("a%03d", alert_num);
            first = false;
          }
        }
        DEBUG_SERIAL.println("]");
      }
    }
  }
  #endif
}

void PCSCan::handle3C4(uint32_t data[2]) {
  // DBC: PCS_info (0x3C4 / 964)
  uint8_t* bytes = (uint8_t*)data;
  uint8_t mux = bytes[0];

  // Build up PCS info incrementally
  static PCSInfoData pcs_info = {};

  switch (mux) {
    case 10:
      // DBC: PCS_buildType 8|8@1+, PCS_buildConfigId 16|16@1+, PCS_hardwareId 32|16@1+, PCS_componentId 48|16@1+
      pcs_info.build_type = bytes[1];
      pcs_info.build_config_id = (bytes[3] << 8) | bytes[2];
      pcs_info.hardware_id = (bytes[5] << 8) | bytes[4];
      pcs_info.component_id = (bytes[7] << 8) | bytes[6];

      #if DEBUG_PCS_CAN
      if (!pcs_info.info_valid) {
        DEBUG_SERIAL.printf("PCS Info: build=%d cfg=0x%04X hw=0x%04X comp=0x%04X\r\n",
                            pcs_info.build_type, pcs_info.build_config_id,
                            pcs_info.hardware_id, pcs_info.component_id);
      }
      #endif
      pcs_info.info_valid = true;
      PCSController::update_pcs_info(pcs_info);
      break;

    case 11:
      // DBC: PCS_pcbaId 16|8@1+, PCS_assemblyId 24|8@1+, PCS_usageId 32|16@1+, PCS_subUsageId 48|16@1+
      pcs_info.pcba_id = bytes[2];
      pcs_info.assembly_id = bytes[3];
      pcs_info.usage_id = (bytes[5] << 8) | bytes[4];
      pcs_info.subusage_id = (bytes[7] << 8) | bytes[6];
      PCSController::update_pcs_info(pcs_info);
      break;

    case 13:
      // DBC: PCS_platformType 8|8@1+, PCS_appCrc 32|32@1+
      pcs_info.platform_type = bytes[1];
      pcs_info.app_crc = ((uint32_t)bytes[7] << 24) | ((uint32_t)bytes[6] << 16) |
                         ((uint32_t)bytes[5] << 8) | bytes[4];
      PCSController::update_pcs_info(pcs_info);
      break;

    case 17: {
      // DBC: PCS_appGitHash 10|54@1+ (4,0) - 54 bits starting at bit 10
      uint64_t raw = ((uint64_t)bytes[7] << 56) | ((uint64_t)bytes[6] << 48) |
                     ((uint64_t)bytes[5] << 40) | ((uint64_t)bytes[4] << 32) |
                     ((uint64_t)bytes[3] << 24) | ((uint64_t)bytes[2] << 16) |
                     ((uint64_t)bytes[1] << 8) | bytes[0];
      pcs_info.app_git_hash = (raw >> 10) & 0x3FFFFFFFFFFFFFULL; // 54 bits
      PCSController::update_pcs_info(pcs_info);
      break;
    }

    case 18: {
      // DBC: PCS_bootGitHash 10|54@1+ (4,0) - 54 bits starting at bit 10
      uint64_t raw = ((uint64_t)bytes[7] << 56) | ((uint64_t)bytes[6] << 48) |
                     ((uint64_t)bytes[5] << 40) | ((uint64_t)bytes[4] << 32) |
                     ((uint64_t)bytes[3] << 24) | ((uint64_t)bytes[2] << 16) |
                     ((uint64_t)bytes[1] << 8) | bytes[0];
      pcs_info.bootloader_git_hash = (raw >> 10) & 0x3FFFFFFFFFFFFFULL; // 54 bits
      PCSController::update_pcs_info(pcs_info);
      break;
    }

    case 20:
      // DBC: PCS_bootUdsProtoVersion 8|8@1+, PCS_bootCrc 32|32@1+
      pcs_info.bootloader_uds_proto_version = bytes[1];
      pcs_info.boot_crc = ((uint32_t)bytes[7] << 24) | ((uint32_t)bytes[6] << 16) |
                          ((uint32_t)bytes[5] << 8) | bytes[4];
      PCSController::update_pcs_info(pcs_info);
      break;

    case 25:
      // DBC: PCS_partNumChar01-07 at bits 8, 16, 24, 32, 40, 48, 56 (bytes 1-7)
      for (int i = 0; i < 7; i++) {
        pcs_info.part_number[i] = bytes[i + 1];
      }
      break;

    case 26:
      // DBC: PCS_partNumChar08-14 at bits 8, 16, 24, 32, 40, 48, 56 (bytes 1-7)
      for (int i = 0; i < 7; i++) {
        pcs_info.part_number[7 + i] = bytes[i + 1];
      }
      break;

    case 27:
      // DBC: PCS_partNumChar15-20 at bits 8, 16, 24, 32, 40, 48 (bytes 1-6)
      for (int i = 0; i < 6; i++) {
        pcs_info.part_number[14 + i] = bytes[i + 1];
      }
      pcs_info.part_number[20] = '\0';
      PCSController::update_pcs_info(pcs_info);

      #if DEBUG_PCS_CAN
      static bool part_logged = false;
      if (!part_logged && pcs_info.part_number[0] != '\0') {
        DEBUG_SERIAL.printf("PCS Part#: %s\r\n", pcs_info.part_number);
        part_logged = true;
      }
      #endif
      break;

    default:
      break;
  }
}

void PCSCan::handle424(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t alert_id = bytes[0];

  #if DEBUG_PCS_CAN
  DEBUG_SERIAL.printf("PCS Alert 0x%02X received\r\n", alert_id);
  #endif

  // Handle CAN rationality alert for auto message format detection
  if (alert_id == 0x1E) {
    // Debug: dump raw bytes to verify parsing
    DEBUG_SERIAL.printf("  Raw alert bytes: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
      bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]);

    uint8_t error_detail = bytes[2] & 0x07;
    uint16_t error_can_id = (bytes[4] << 8) | bytes[3];
    DEBUG_SERIAL.printf("  CAN Rationality: ID=0x%03X err=%d\r\n", error_can_id, error_detail);

    if (error_can_id == 0x2B2) {
      if (error_detail == 0x01) {
        DEBUG_SERIAL.println("  Auto-switching 0x2B2 to short (3-byte) format");
        use_long_msg_format = false;
      } else if (error_detail == 0x02) {
        DEBUG_SERIAL.println("  Auto-switching 0x2B2 to long (5-byte) format");
        use_long_msg_format = true;
      }
    }
  }

  // Post error message (a001-a109 + d110-d116, IDs 1-116 = 0x01-0x74)
  if (alert_id >= 0x01 && alert_id <= 0x74) {
    ERROR_MESSAGE_NUM error = (ERROR_MESSAGE_NUM)(ERR_PCS_a001_chgHwInputOc + (alert_id - 0x01));
    ErrorMessage::Post(error);
  }
  else {
    DEBUG_SERIAL.printf("  Unknown PCS alert ID: 0x%02X\r\n", alert_id);
  }
}

void PCSCan::handle504(uint32_t data[2]) {
  // Boot ID counter - not currently used
}

void PCSCan::handle76C(uint32_t data[2]) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t mux = bytes[0];

  // Fallback DC current measurement (used if 0x2C4 not available)
  if (!mux_state.got_dci) {
    float current = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    uint8_t phase = 255;

    if (mux == 0x0C) phase = 0;
    else if (mux == 0x16) phase = 1;
    else if (mux == 0x20) phase = 2;

    if (phase < 3) {
      PCSController::update_dc_phase_current(phase, current);
    }
  }
}

// ==================== TX MESSAGES ====================

void PCSCan::Msg13D() {
  if (can_bus == nullptr) return;
  uint8_t bytes[6];
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
  // DBC: BO_ 522 HVP_contactorState: 6 VEH (decimal 522 = 0x20A hex) - 10ms cycle
  //  SG_ HVP_packContNegativeState: 0|3@1+ - pack negative contactor state
  //  SG_ HVP_packContPositiveState: 3|3@1+ - pack positive contactor state
  //  SG_ HVP_packContactorSetState: 8|4@1+ - pack contactor set state
  //  SG_ HVP_fcContNegativeState: 12|3@1+ - fast charge negative contactor state
  //  SG_ HVP_fcContPositiveState: 16|3@1+ - fast charge positive contactor state
  //  SG_ HVP_fcContactorSetState: 19|4@1+ - fast charge contactor set state
  //  SG_ HVP_fcCtrsRequestStatus: 24|2@1+ - fast charge contactors request status
  //  SG_ HVP_fcCtrsClosingAllowed: 29|1@1+ - fast charge contactors closing allowed
  //  SG_ HVP_fcLinkAllowedToEnergizeAc: 30|1@1+ - FC link allowed to energize AC
  //  SG_ HVP_fcLinkAllowedToEnergizeDc: 31|1@1+ - FC link allowed to energize DC
  //  SG_ HVP_packCtrsRequestStatus: 32|2@1+ - pack contactors request status
  //  SG_ HVP_packCtrsClosingAllowed: 37|1@1+ - pack contactors closing allowed
  //  SG_ HVP_dcLinkAllowedToEnergize: 38|1@1+ - DC link allowed to energize
  //  SG_ HVP_hvilStatus: 40|4@1+ - HVIL status
  if (can_bus == nullptr) return;

  // Pack contactor states - indicate contactors are closed and ready
  HVP_ContactorState pack_cont_neg = HVP_CONTACTOR_OPEN;
  HVP_ContactorState pack_cont_pos = HVP_CONTACTOR_OPEN;
  HVP_ContactorSetState pack_set_state = HVP_SET_OPEN;

  // Fast charge contactor states - indicate FC contactors are open (not DC charging)
  HVP_ContactorState fc_cont_neg = HVP_CONTACTOR_OPEN;
  HVP_ContactorState fc_cont_pos = HVP_CONTACTOR_OPEN;
  HVP_ContactorSetState fc_set_state = HVP_SET_OPEN;

  // Request statuses
  HVP_CtrsRequestStatus fc_request_status = HVP_CTRS_NOT_ACTIVE;
  HVP_CtrsRequestStatus pack_request_status = HVP_CTRS_NOT_ACTIVE;

  // Control flags
  uint8_t fc_closing_allowed = 1;       // Allow FC contactors to close
  uint8_t fc_link_energize_ac = 0;      // FC link AC energize (not used for AC charging)
  uint8_t fc_link_energize_dc = 0;      // FC link DC energize (not DC charging)
  uint8_t pack_closing_allowed = 1;     // Allow pack contactors to close
  uint8_t dc_link_energize = 1;         // DC link allowed to energize

  // HVIL status
  HVP_HvilStatus hvil_status = HVP_HVIL_STATUS_OK;

  uint8_t bytes[6] = {0};

  // Byte 0 (bits 0-7): packContNegativeState (0-2), packContPositiveState (3-5)
  bytes[0] = (pack_cont_neg & 0x07) | ((pack_cont_pos & 0x07) << 3);

  // Byte 1 (bits 8-15): packContactorSetState (8-11), fcContNegativeState (12-14)
  bytes[1] = (pack_set_state & 0x0F) | ((fc_cont_neg & 0x07) << 4);

  // Byte 2 (bits 16-23): fcContPositiveState (16-18), fcContactorSetState (19-22)
  bytes[2] = (fc_cont_pos & 0x07) | ((fc_set_state & 0x0F) << 3);

  // Byte 3 (bits 24-31): fcCtrsRequestStatus (24-25), flags (26-31)
  bytes[3] = (fc_request_status & 0x03)
           | (fc_closing_allowed << 5)
           | (fc_link_energize_ac << 6)
           | (fc_link_energize_dc << 7);

  // Byte 4 (bits 32-39): packCtrsRequestStatus (32-33), flags (34-39)
  bytes[4] = (pack_request_status & 0x03)
           | (pack_closing_allowed << 5)
           | (dc_link_energize << 6);

  // Byte 5 (bits 40-47): hvilStatus (40-43)
  bytes[5] = (hvil_status & 0x0F);

  log_tx_message(0x20A, bytes, 6);
  if (!can_bus->sendMessage(0x20A, bytes, 6)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x20A");
  }
}

void PCSCan::Msg212() {
  if (can_bus == nullptr) return;
  uint8_t bytes[8] = {0};

  // JSON: BMS_userChargeStatus at bits 11-13 (byte 1 bits 3-5), 3-bit
  BMSUiChargeStatus ui_charge_status = BMS_UI_CHARGE_DISCONNECTED;
  if (PCSController::is_active()) {
    ui_charge_status = BMS_UI_CHARGE_CHARGING;
  } else if (PCSController::is_faulted()) {
    ui_charge_status = BMS_UI_CHARGE_STOPPED;
  }
  bytes[1] |= (ui_charge_status & 0x07) << 3;

  // JSON: BMS_state at bits 32-35 (byte 4 bits 0-3), 4-bit
  BMSState bms_state = BMS_STATE_STANDBY;
  if (PCSController::is_active()) {
    bms_state = BMS_STATE_CHARGE;
  }
  bytes[4] |= (bms_state & 0x0F);

  // JSON: BMS_contactorState at bits 8-10 (byte 1 bits 0-2), 3-bit
  BMSContactorState contactor_state = BMS_CONTACTOR_OPEN;
  if (PCSController::is_active()) {
    contactor_state = BMS_CONTACTOR_CLOSED;
  }
  bytes[1] |= (contactor_state & 0x07);

  // Example: Set BMS_chargeRequest (bit 29, byte 3 bit 5)
  bytes[3] |= (charge_enable ? 1 : 0) << 5;

  // Example: Set BMS_chgPowerAvailable (bits 38-48, byte 4 bits 6-7, byte 5 bits 0-7, byte 6 bit 0)
  uint16_t chg_power_available = 0; // TODO: set from PCSController or logic
  bytes[4] |= ((chg_power_available & 0x03) << 6);
  bytes[5] |= ((chg_power_available >> 2) & 0xFF);
  bytes[6] |= ((chg_power_available >> 10) & 0x01);

  // ... Add other fields as needed, following the DBC bit layout

  log_tx_message(0x212, bytes, 8);
  if (!can_bus->sendMessage(0x212, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x212");
  }
}

void PCSCan::Msg21D() {
  if (can_bus == nullptr) return;
  uint8_t bytes[8] = {0};

  // CP_evseAccept (bit 0)
  bool evse_accept = PCSController::is_charge_enabled();
  bytes[0] |= (evse_accept ? 1 : 0);

  // CP_evseRequest (bit 1)
  bool evse_request = PCSController::is_charge_enabled();
  bytes[0] |= (evse_request ? 1 : 0) << 1;

  // CP_lineVoltageRequested (bit 2)
  bool line_voltage_requested = (PCSController::get_charger_status().grid_config == PCS_GRID_SINGLE_PHASE || PCSController::get_charger_status().grid_config == PCS_GRID_THREE_PHASE);
  bytes[0] |= (line_voltage_requested ? 1 : 0) << 2;

  // CP_pilot (bits 5-7)
  uint8_t pilot = PCSController::get_charger_status().phase_a_enabled + PCSController::get_charger_status().phase_b_enabled + PCSController::get_charger_status().phase_c_enabled;
  bytes[0] |= (pilot & 0x07) << 5;

  // CP_pilotCurrent (byte 1, bits 0-7)
  uint8_t pilot_current = control_params.evse_limit_a * 2; // DBC: (0.5,0) scaling
  bytes[1] = pilot_current;

  // CP_cableType (byte 2, bits 0-2)
  CP_cableType cable_type = CP_CABLE_IEC;
  // Example: set cable type based on cable_limit
  if (control_params.cable_limit > 32) cable_type = CP_CABLE_GB_DC;
  else if (control_params.cable_limit > 16) cable_type = CP_CABLE_SAE;
  bytes[2] |= (cable_type & 0x07);

  // CP_cableCurrentLimit (byte 3, bits 0-6)
  uint8_t cable_limit = control_params.cable_limit; // DBC: (1,0) scaling
  bytes[3] |= (cable_limit & 0x7F);

  // CP_acChargeState (byte 7, bits 4-5)
  CP_acChargeState ac_charge_state = CP_AC_INACTIVE;
  if (PCSController::is_charge_enabled()) ac_charge_state = CP_AC_ACTIVE;
  if (PCSController::is_faulted()) ac_charge_state = CP_AC_SHUTDOWN;
  bytes[7] |= (ac_charge_state & 0x03) << 4;

  // CP_evseChargeType (byte 4, bits 6-7)
  CP_evseChargeType evse_charge_type = CP_EVSE_NONE;
  if (PCSController::is_charge_enabled()) evse_charge_type = CP_EVSE_AC;
  bytes[4] |= (evse_charge_type & 0x03) << 6;

  // ... Add other fields as needed from DBC

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

  uint8_t control_byte = 0;
  if (clear_faults_counter > 0) {
    control_byte = 0x01;
    clear_faults_counter--;
    DEBUG_SERIAL.println("PCS: Sending fault clear request");
  } else if (charge_enable) {
    control_byte = 0x02;
  }

  if (use_long_msg_format) {
    uint8_t bytes[5];
    bytes[0] = charge_power_w & 0xFF;
    bytes[1] = charge_power_w >> 8;
    bytes[2] = control_byte;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    log_tx_message(0x2B2, bytes, 5);
    if (!can_bus->sendMessage(0x2B2, bytes, 5)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x2B2");
    }
  } else {
    uint8_t bytes[3];
    bytes[0] = charge_power_w & 0xFF;
    bytes[1] = charge_power_w >> 8;
    bytes[2] = control_byte;
    log_tx_message(0x2B2, bytes, 3);
    if (!can_bus->sendMessage(0x2B2, bytes, 3)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x2B2");
    }
  }
}

void PCSCan::Msg333() {
  // DBC: BO_ 819 UI_chargeRequest: 4 VEH
  //  SG_ UI_openChargePortDoorRequest: 0|1@1+ (1,0) [0|0] "" X
  //  SG_ UI_closeChargePortDoorRequest: 1|1@1+ (1,0) [0|0] "" X
  //  SG_ UI_chargeEnableRequest: 2|1@1+ (1,0) [0|0] "" X
  //  SG_ UI_acChargeCurrentLimit: 8|7@1+ (1,0) [0|0] "A" X
  //  SG_ UI_chargeTerminationPct: 16|10@1+ (0.1,0) [0|0] "%" X
  if (can_bus == nullptr) return;

  uint8_t open_charge_port = 0;     // bit 0
  uint8_t close_charge_port = 0;    // bit 1
  uint8_t charge_enable_flag = charge_enable ? 1 : 0; // bit 2
  uint8_t ac_current_limit = control_params.ac_current_limit_a;  // bits 8-14
  // DBC scaling: (0.1, 0) means raw value = percentage * 10
  uint16_t charge_termination_raw = control_params.charge_termination_percent * 10;  // bits 16-25

  uint8_t bytes[5] = {0};
  bytes[0] = (open_charge_port & 0x01)
           | ((close_charge_port & 0x01) << 1)
           | ((charge_enable_flag & 0x01) << 2);
  bytes[1] = ac_current_limit & 0x7F;
  bytes[2] = charge_termination_raw & 0xFF;
  bytes[3] = (charge_termination_raw >> 8) & 0x03;
  bytes[4] = 0x00;  // not sure -> logs show this as zero during normal operation

  log_tx_message(0x333, bytes, 5);
  if (!can_bus->sendMessage(0x333, bytes, 5)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x333");
  }
}

void PCSCan::Msg2F1() {
  // DBC: BO_ 753 VCFRONT_eFuseDebugStatus: 8 VEH (decimal 753 = 0x2F1 hex) - 100ms cycle
  //  SG_ VCFRONT_eFuseDebugStatusIndex M: 0|5@1+ (1,0) [0|0] "" X
  //  Multiplexed: mux 0=right controller, 1=left controller, 2=PCS, 3=iBooster
  //  Contains temperature, voltage, current readings for various controllers
  if (can_bus == nullptr) return;

  static uint8_t mux_index = 0;
  uint8_t bytes[8] = {0};

  // Mux between different controllers (0-3)
  bytes[0] = mux_index & 0x1F;

  // For mux 2 (PCS), echo back real PCS data. For others, use default values
  uint16_t temp_raw, volt_raw;
  int16_t curr_raw;
  uint8_t state, fault;

  if (mux_index == 2) {
    // Echo back real PCS data from PCSController
    const DCDCStatus& dcdc = PCSController::get_dcdc_status();
    const TemperatureData& temps = PCSController::get_temperature_data();
    const DCDCBusStatus& bus = PCSController::get_dcdc_bus_status();

    state = dcdc.main_state & 0x03;
    fault = dcdc.faulted ? 1 : 0;
    // DBC (0x2F1 mux 2): Temperature scale 0.125, offset -40
    // Decode: temp_c = raw * 0.125 - 40; Encode: raw = (temp_c + 40) / 0.125
    temp_raw = (uint16_t)((temps.dcdc_c + 40.0f) / 0.125f);
    // DBC (0x2F1 mux 2): Voltage scale 0.1, offset 0
    // Decode: volt_v = raw * 0.1; Encode: raw = volt_v / 0.1
    volt_raw = (uint16_t)(bus.lv_bus_voltage_v / 0.1f);
    // DBC (0x2F1 mux 2): Current scale 0.1, offset 0 (signed)
    // Decode: curr_a = raw * 0.1; Encode: raw = curr_a / 0.1
    curr_raw = (int16_t)(bus.lv_output_current_a / 0.1f);
  } else {
    // Default values for other controllers
    state = 0;
    fault = 0;
    temp_raw = 400;      // ~50°C
    volt_raw = 138;      // ~13.8V
    curr_raw = 500;      // ~50A
  }

  // Pack state and fault into byte 1
  bytes[1] = ((state & 0x03) << 0) | ((fault & 0x01) << 2);

  // Temperature: 11-bit little-endian starting at bit 15 (byte 1 bit 7, then byte 2, then byte 3 bits 0-1)
  bytes[1] |= ((temp_raw & 0x01) << 7);
  bytes[2] = (temp_raw >> 1) & 0xFF;
  bytes[3] = (temp_raw >> 9) & 0x03;

  // Voltage: 16-bit starting at bit 31 (byte 3 bits 2-7, then byte 4, then byte 5 bits 0-3)
  bytes[3] |= ((volt_raw & 0x0F) << 2);
  bytes[4] = (volt_raw >> 4) & 0xFF;
  bytes[5] = (volt_raw >> 12) & 0x0F;

  // Current: 16-bit signed starting at bit 47 (byte 5 bits 4-7, then byte 6, then byte 7 bits 0-3)
  bytes[5] |= ((curr_raw & 0x0F) << 4);
  bytes[6] = (curr_raw >> 4) & 0xFF;
  bytes[7] = (curr_raw >> 12) & 0x0F;

  // Rotate mux for next message
  mux_index = (mux_index + 1) % 4;

  log_tx_message(0x2F1, bytes, 8);
  if (!can_bus->sendMessage(0x2F1, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x2F1");
  }
}

void PCSCan::Msg301() {
  // DBC: BO_ 769 VCFRONT_info: 8 VEH (decimal 769 = 0x301 hex) - 1000ms cycle
  //  SG_ VCFRONT_infoIndex M: 0|8@1+ (1,0) [0|0] "" X
  //  Multiplexed: Contains build info, hardware IDs, CRC checksums, git hashes
  //  Mostly static/informational, varying through different mux pages
  if (can_bus == nullptr) return;

  static uint8_t mux_index = 0x10;  // Start with a valid mux value
  uint8_t bytes[8] = {0};

  bytes[0] = mux_index;

  // For most mux values, send reasonable defaults
  // Example: mux 0x10 (build type info)
  if (mux_index == 0x10) {
    bytes[1] = 0x02;  // Build type = MFG
    bytes[2] = 0x01;  // Build config ID (low byte)
    bytes[3] = 0x00;  // Build config ID (high byte)
    bytes[4] = 0x01;  // Hardware ID (low byte)
    bytes[5] = 0x00;  // Hardware ID (high byte)
  } else {
    // For other mux values, fill with reasonable defaults
    for (int i = 1; i < 8; i++) {
      bytes[i] = 0x00;
    }
  }

  // Rotate through different info pages
  mux_index++;
  if (mux_index > 0x1F) mux_index = 0x10;

  log_tx_message(0x301, bytes, 8);
  if (!can_bus->sendMessage(0x301, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x301");
  }
}

void PCSCan::Msg321() {
  // DBC: BO_ 801 VCFRONT_sensors: 8 VEH (decimal 801 = 0x321 hex) - 1000ms cycle
  //  SG_ VCFRONT_tempCoolantBatInlet: 0|10@1+ (0.125,-40) "degC"
  //  SG_ VCFRONT_tempCoolantPTInlet: 10|11@1+ (0.125,-40) "degC"
  //  SG_ VCFRONT_brakeFluidLevel: 22|2@1+ (1,0) ""
  //  SG_ VCFRONT_coolantLevel: 21|1@1+ (1,0) ""
  //  SG_ VCFRONT_tempAmbient: 24|8@1+ (0.5,-40) "degC"
  //  SG_ VCFRONT_washerFluidLevel: 32|2@1+ (1,0) ""
  //  SG_ VCFRONT_tempAmbientFiltered: 40|8@1+ (0.5,-40) "degC"
  //  SG_ VCFRONT_battSensorIrrational: 48|1@1+ (1,0) ""
  //  SG_ VCFRONT_ptSensorIrrational: 49|1@1+ (1,0) ""
  if (can_bus == nullptr) return;

  uint8_t bytes[8] = {0};

  // Use default values from real trace: 37 DA A8 90 01 8A D0 C8
  // Coolant Bat Inlet: 30.9°C = 567 raw = 0x237
  uint16_t coolant_bat = 567;
  bytes[0] = coolant_bat & 0xFF;
  bytes[1] = (bytes[1] & 0xFC) | ((coolant_bat >> 8) & 0x03);

  // Coolant PT Inlet: 0.8°C = 326 raw = 0x146
  uint16_t coolant_pt = 326;
  bytes[1] |= ((coolant_pt & 0x07) << 2);
  bytes[2] = (coolant_pt >> 3) & 0xFF;

  // Coolant level: bit 21 = 1 (full)
  bytes[2] |= (1 << 5);

  // Brake fluid level: bits 22-23 = 2
  bytes[2] |= (2 << 6);

  // Ambient temp: 32°C = 144 raw (scale 0.5: (32+40)*2 = 144)
  bytes[3] = 144;

  // Washer fluid: bits 32-33 = 1
  bytes[4] = (1 << 0);

  // Ambient filtered: 29°C = 138 raw (scale 0.5: (29+40)*2 = 138)
  bytes[5] = 138;

  // Sensor flags: bits 48-49 = 0 (both OK)
  bytes[6] = 0x00;

  // Checksum/counter byte (varies in trace, use fixed value)
  bytes[7] = 0xC8;

  log_tx_message(0x321, bytes, 8);
  if (!can_bus->sendMessage(0x321, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x321");
  }
}

void PCSCan::Msg3A1() {
  // DBC: BO_ 929 VCFRONT_vehicleStatus: 8 VEH (decimal 929 = 0x3A1 hex) - 100ms cycle
  //  SG_ VCFRONT_pcs12vVoltageTarget: 16|11@1+ (0.01,0) "V" - DCDC target voltage
  //  SG_ VCFRONT_pcsEFuseVoltage: 42|10@1+ (0.1,0) "V" - eFuse measured voltage
  //  SG_ VCFRONT_vehicleStatusCounter: 52|4@1+ - message counter
  //  SG_ VCFRONT_vehicleStatusChecksum: 56|8@1+ - checksum
  if (can_bus == nullptr) return;

  // Calculate voltage values - both should match DCDC target
  uint16_t voltage_target_raw = (uint16_t)(control_params.dcdc_voltage_v * 100.0f);  // scale 0.01V: 14V -> 1400
  uint16_t efuse_voltage_raw = (uint16_t)(control_params.dcdc_voltage_v * 10.0f);    // scale 0.1V: 14V -> 140

  uint8_t bytes[8] = {0};

  // --- 12V and HV/charging system bits ---
  // VCFRONT_bmsHvChargeEnable: bit 0
  bool hv_charge_enable = PCSController::is_charge_enabled();
  bytes[0] |= (hv_charge_enable ? 1 : 0);

  // VCFRONT_preconditionRequest: bit 1
  bool precondition_request = false; // TODO: set from logic
  bytes[0] |= (precondition_request ? 1 : 0) << 1;

  // VCFRONT_is12VBatterySupported: bit 5
  bool is_12v_supported = true;
  bytes[0] |= (is_12v_supported ? 1 : 0) << 5;

  // VCFRONT_standbySupplySupported: bit 6
  bool standby_supply_supported = true; // TODO: set from logic
  bytes[0] |= (standby_supply_supported ? 1 : 0) << 6;

  // VCFRONT_12vStatusForDrive: bits 14-15 (byte 1 bits 6-7)
  VCFRONT_12vStatusForDrive status_12v_drive = STATUS_12V_OK;
  // Example logic: set FAULT if DCDC is faulted, WARNING if output limited, else OK
  // if (PCSController::get_dcdc_status().faulted) {
  //   status_12v_drive = STATUS_12V_FAULT;
  // } else if (PCSController::get_dcdc_status().output_limited) {
  //   status_12v_drive = STATUS_12V_WARNING;
  // }
  bytes[1] |= (status_12v_drive & 0x03) << 6;

  // Bytes 2-3: VCFRONT_pcs12vVoltageTarget at bits 16-26 (11 bits)
  bytes[2] = voltage_target_raw & 0xFF;                 // bits 16-23
  bytes[3] = (voltage_target_raw >> 8) & 0x07;          // bits 24-26

  // VCFRONT_batterySupportRequest: bit 27 (byte 3 bit 3) — battery available for support
  bytes[3] |= (1 << 3);

  // Byte 4: Other signals
  bytes[4] = 0x00;

  // Bytes 5-6: VCFRONT_pcsEFuseVoltage at bits 42-51 (10 bits)
  // Bit 42 = byte 5 bit 2, so efuse spans byte 5 bits 2-7 (6 bits) and byte 6 bits 0-3 (4 bits)
  bytes[5] = (efuse_voltage_raw & 0x3F) << 2;              // bits 42-47 (low 6 bits shifted to bits 2-7)
  bytes[6] = ((efuse_voltage_raw >> 6) & 0x0F);            // bits 48-51 (high 4 bits in bits 0-3)

  // Byte 6 bits 4-7: VCFRONT_vehicleStatusCounter
  bytes[6] |= (mux_state.count_3a1 << 4);

  // Byte 7: Checksum
  bytes[7] = calc_checksum(bytes, 0x3A1);

  log_tx_message(0x3A1, bytes, 8);
  if (!can_bus->sendMessage(0x3A1, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x3A1");
  }

  // Increment counter (4-bit, wraps at 15)
  mux_state.count_3a1++;
  if (mux_state.count_3a1 > 0x0F) mux_state.count_3a1 = 0;
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
    log_tx_message(0x3B2, bytes, 8);
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
    log_tx_message(0x3B2, bytes, 8);
    if (!can_bus->sendMessage(0x3B2, bytes, 8)) {
      DEBUG_SERIAL.println("ERROR: Failed to send 0x3B2");
    }
    mux_state.mux_3b2 = true;
  }
}

void PCSCan::Msg221() {
  // DBC: BO_ 545 VCFRONT_LVPowerState: 8 VEH (decimal 545 = 0x221 hex) - 50ms cycle
  // Multiplexed message with 2 pages (mux 0 and 1)
  // CRITICAL: This message signals LV power availability to PCS - needed for DCDC regulation
  // Real trace shows fixed payloads for mux 0/1 with separate rotating counters
  if (can_bus == nullptr) return;

  static uint8_t counter_mux0 = 0;
  static uint8_t counter_mux1 = 0;
  static uint8_t mux_221 = 0;
  uint8_t bytes[8] = {0};

  if (mux_221 == 0) {
    // Mux 0: Real trace payload = 40 41 05 15 00 50
    bytes[0] = 0x40;  // Mux 0, vehicle power state OFF
    bytes[1] = 0x41;
    bytes[2] = 0x05;
    bytes[3] = 0x15;
    bytes[4] = 0x00;
    bytes[5] = 0x50;
    // Counter at bits 52-55 (byte 6, upper nibble)
    bytes[6] = ((counter_mux0 & 0x0F) << 4);
    counter_mux0 = (counter_mux0 + 1) & 0x0F;
  } else {
    // Mux 1: Real trace payload = 41 01 55 51 01 02
    // Bit 16 (byte 2, bits 0-1) = pcsLVState signals LV power available to PCS
    bytes[0] = 0x41;  // Mux 1, vehicle power state = 2 (ACCESSORY)
    bytes[1] = 0x01;
    bytes[2] = 0x55;
    bytes[3] = 0x51;
    bytes[4] = 0x01;
    bytes[5] = 0x02;
    // Counter at bits 52-55 (byte 6, upper nibble)
    bytes[6] = ((counter_mux1 & 0x0F) << 4);
    counter_mux1 = (counter_mux1 + 1) & 0x0F;
  }

  bytes[7] = calc_checksum(bytes, 0x221);
  log_tx_message(0x221, bytes, 8);

  if (!can_bus->sendMessage(0x221, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x221");
  }

  mux_221 = !mux_221;  // Toggle between mux 0 and 1
}

void PCSCan::Msg201() {
  // Unknown message (0x201 / 513 decimal)
  // Pattern observed in real Model 3 traces: 4-message repeating cycle
  // Appears ~1,762 times in charging trace (roughly every 25-30ms)
  //
  // Cycle pattern:
  //   Pattern 0: 02 00 00 00 00 00 00 00
  //   Pattern 1: 03 00 00 00 00 00 00 00
  //   Pattern 2: 00 XX XX XX 00 E4 FF 00  (XX varies, sensor data?)
  //   Pattern 3: 59 38 98 00 01 00 00 00
  //
  // Repeats every 4 messages (~100-120ms cycle)
  if (can_bus == nullptr) return;

  static uint8_t cycle_201 = 0;
  uint8_t bytes[8] = {0};

  switch (cycle_201) {
    case 0:
      bytes[0] = 0x02;
      bytes[1] = 0x00;
      bytes[2] = 0x00;
      bytes[3] = 0x00;
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;
      break;

    case 1:
      bytes[0] = 0x03;
      bytes[1] = 0x00;
      bytes[2] = 0x00;
      bytes[3] = 0x00;
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;
      break;

    case 2:
      // Pattern with variable bytes 1-3 (sensor data, using median values from trace)
      bytes[0] = 0x00;
      bytes[1] = 0x32;  // Varies in range 0x30-0x33
      bytes[2] = 0x50;  // Varies in range 0x4F-0x51
      bytes[3] = 0x1A;  // Varies in range 0x19-0x1A
      bytes[4] = 0x00;
      bytes[5] = 0xE4;
      bytes[6] = 0xFF;
      bytes[7] = 0x00;
      break;

    case 3:
      bytes[0] = 0x59;
      bytes[1] = 0x38;
      bytes[2] = 0x98;
      bytes[3] = 0x00;
      bytes[4] = 0x01;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;
      break;
  }

  log_tx_message(0x201, bytes, 8);
  if (!can_bus->sendMessage(0x201, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x201");
  }

  // Advance to next pattern in cycle
  cycle_201 = (cycle_201 + 1) % 4;
}

void PCSCan::Msg2E1() {
  // DBC: BO_ 737 VCFRONT_status: 8 ETH (decimal 737 = 0x2E1 hex) - 20ms cycle
  // Multiplexed on VCFRONT_statusIndex (3-bit, bits 0-2), pages 0-4.
  // Page 4 carries VCFRONT_PCSMia (bit 38) and VCFRONT_DCDCNoop (bit 39).
  // Critical: PCSMia=0 (PCS is alive), DCDCNoop=0 (DCDC is active)
  if (can_bus == nullptr) return;

  static uint8_t page = 0;
  uint8_t bytes[8] = {0};
  bytes[0] = page & 0x07;  // VCFRONT_statusIndex bits 0-2

  // For page 4, explicitly ensure PCSMia and DCDCNoop are set correctly
  if (page == 4) {
    // bit 38 = byte 4 bit 6: VCFRONT_PCSMia = 0 (PCS is alive)
    // bit 39 = byte 4 bit 7: VCFRONT_DCDCNoop = 0 (DCDC is operational)
    bytes[4] = 0x00;  // Both bits 6 and 7 = 0 (correct values)
  }

  log_tx_message(0x2E1, bytes, 8);
  if (!can_bus->sendMessage(0x2E1, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x2E1");
  }

  if (++page > 4) page = 0;
}

void PCSCan::Msg261() {
  // DBC: BO_ 609 VCFRONT_12VBatteryStatus: 8 VEH (decimal 609 = 0x261 hex) - 100ms cycle
  // VCFRONT simulated response (since no real VCFront in EV conversion)
  if (can_bus == nullptr) return;

  static uint8_t mux_261 = 0;
  uint8_t bytes[8] = {0};

  // Byte 0: VCFRONT_12VBatteryIndex (bits 0-1)
  bytes[0] = (mux_261 & 0x03);

  switch (mux_261) {
    case 0:  // m0: IBSCurrent
    {
      int16_t ibs_current_raw = 0;  // 0A
      bytes[2] = ibs_current_raw & 0xFF;
      bytes[3] = ((ibs_current_raw >> 8) & 0x0F);
      break;
    }

    case 1:  // m1: IBSCurrentRaw and IBSVoltage
    {
      int16_t ibs_current_raw = 0;  // 0A
      bytes[2] = ibs_current_raw & 0xFF;
      bytes[3] = ((ibs_current_raw >> 8) & 0xFF);

      uint16_t ibs_voltage_raw = 2425;  // 13.2V
      bytes[4] = ibs_voltage_raw & 0xFF;
      bytes[5] = ((ibs_voltage_raw >> 8) & 0x0F);
      break;
    }

    case 2:  // m2: IBSAmpHours and IBSTemperature
    {
      int16_t ibs_amp_hours_raw = 5000;  // 50Ah
      bytes[0] |= ((ibs_amp_hours_raw & 0x03) << 2);
      bytes[1] = ((ibs_amp_hours_raw >> 2) & 0xFF);
      bytes[2] = ((ibs_amp_hours_raw >> 10) & 0x0F);

      int16_t ibs_temp_raw = 250;  // 25°C
      bytes[1] |= ((ibs_temp_raw & 0x01) << 7);
      bytes[2] |= ((ibs_temp_raw >> 1) & 0x7F);
      bytes[3] = ((ibs_temp_raw >> 8) & 0xFF);
      break;
    }
  }

  log_tx_message(0x261, bytes, 8);
  if (!can_bus->sendMessage(0x261, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x261");
  }

  mux_261++;
  if (mux_261 > 2) mux_261 = 0;
}

void PCSCan::Msg340() {
  // DBC: BO_ 832 VCFRONT_alertMatrix: 8 VEH (decimal 832 = 0x340 hex) - 100ms cycle
  // VCFRONT simulated response with DCDC operational (since no real VCFront in EV conversion)
  // Index 3 contains VCFRONT_a190_DCDCNotOperational at byte 1 bit 5
  if (can_bus == nullptr) return;

  uint8_t bytes[8] = {0};

  // Always send index 3 (contains DCDC status)
  uint8_t matrix_index = 3;
  bytes[0] = (matrix_index & 0x0F);

  // Set all alert bits to 0 (no alerts)
  // VCFRONT_a190_DCDCNotOperational at byte 1 bit 5 = 0 means DCDC is operational
  bytes[1] = 0;  // No alerts - DCDC is operational
  bytes[2] = 0;  // No alerts
  bytes[3] = 0;  // No alerts
  bytes[4] = 0;  // No alerts
  bytes[5] = 0;  // No alerts
  bytes[6] = 0;  // No alerts
  bytes[7] = 0;  // Placeholder (no checksum in DBC)

  log_tx_message(0x340, bytes, 8);
  if (!can_bus->sendMessage(0x340, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x340");
  }
}

void PCSCan::Msg545() {
  // VCFRONT_systemStatus (0x545) - 50ms cycle
  // Replicated from real Model 3 CAN traces showing dynamic system status
  //
  // Pattern analysis from 4 real charging/driving traces:
  // Type A: 01 00 00 00 00 10 XX YY (most common, ~445+ occurrences per trace)
  // Type B: 02 00 01-0C 00 00 00 XX YY (load shedding indicator, varies by state)
  // Type C: Various patterns with different byte 0 and byte 2 values
  //
  // For idle/normal operation: send Type A pattern
  // Byte 6: lower nibble = 0 for type A; upper nibble = rolling counter
  // Byte 7: checksum = (sum of bytes 0-6 + 0x45 + 0x05) & 0xFF
  if (can_bus == nullptr) return;

  uint8_t bytes[8] = {0};

  // Send the most common pattern from real traces (Type A - normal operation)
  // This appears 100+ times per trace and seems to be the baseline state
  bytes[0] = 0x01;  // System status type A
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x10;
  bytes[6] = 0x00 | (mux_state.count_545 << 4);  // Counter in upper nibble

  bytes[7] = calc_checksum(bytes, 0x545);

  log_tx_message(0x545, bytes, 8);

  if (!can_bus->sendMessage(0x545, bytes, 8)) {
    DEBUG_SERIAL.println("ERROR: Failed to send 0x545");
  }

  // Increment counter for next message (wraps at 16)
  mux_state.count_545++;
  if (mux_state.count_545 > 0x0F) mux_state.count_545 = 0;
}


// ==================== HELPERS ====================

uint8_t PCSCan::calc_checksum(uint8_t *bytes, uint16_t id) {
  uint16_t checksum_calc = 0;
  for (int b = 0; b < 7; b++) {
    checksum_calc = checksum_calc + bytes[b];
  }
  checksum_calc += id + (id >> 8);
  checksum_calc &= 0xFF;
  return (uint8_t)checksum_calc;
}

float PCSCan::convert_temp_11bit(uint16_t raw) {
  if ((raw & 0x7FF) == 0x3FF) {
    return -999.0f;  // SNA
  }

  int16_t signed_val = raw & 0x7FF;
  if (signed_val & 0x400) {
    signed_val |= 0xF800;
  }

  return (signed_val * 0.1f) + 40.0f;
}
