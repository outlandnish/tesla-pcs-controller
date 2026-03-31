# DBC Exploration Tasks

This document tracks potential CAN message updates based on the Tesla Model 3 DBC file from [onyx-m2-dbc](../model3.dbc).

---

## Completed

### 1. Temperature Parsing (0x2A4 / PCS_thermalStatus)

- [x] Updated to 11-bit signed values
- [x] Applied DBC formula: `raw * 0.1 + 40`
- [x] Updated bit positions per DBC
- [ ] **TODO:** DCDC shows big jumps / fluctuations periodically

---

## Pending Tasks

### 2. Add 0x2B4 Handler (PCS_dcdcBusStatus) - COMPLETED ✓

- [x] Added handler for 0x2B4 (PCS_dcdcBusStatus)
- [x] Created DCDCBusStatus struct with lv_bus_voltage_v, hv_bus_voltage_v, lv_output_current_a
- [x] Implemented handle2B4() with proper bit extraction per DBC
- [x] Added accessor methods to PCSCan and PCSController
- [x] Added debug output to DEBUG_PCS_STATE
- [x] **Switched to 0x2B4 as primary source** for voltage_data and dcdc_status

DBC shows this message contains HV/LV voltage and DCDC current:

```
BO_ 692 PCS_dcdcBusStatus: 5 VEH
 SG_ PCS_dcdcLvBusVolt:      0|10@1+  (0.0390625,0)    "V"
 SG_ PCS_dcdcHvBusVolt:      10|12@1+ (0.146484375,0)  "V"
 SG_ PCS_dcdcLvOutputCurrent: 24|12@1+ (0.1,0)         "A"
```

**✅ VERIFIED on 2017 PCS:** Message 0x2B4 works correctly and provides accurate readings!

- LV voltage now reads correctly (was 0.0V from 0x2C4)
- 0x2B4 is now the primary source for:
  - `voltage_data.lv_v` (LV voltage)
  - `voltage_data.hv_v` (HV voltage)
  - `dcdc_status.current_a` (DCDC current)
  - `dcdc_status.power_w` (DCDC power)
- All existing OpenInverter parameters (ulv, udc, idcdc, powerdcdc) now use 0x2B4 data
- 0x224 handler disabled (current parsing) - appears to be status flags only per DBC

---

### 3. Fix 0x224 Handler (PCS_dcdcStatus) - COMPLETED ✓

- [x] Removed current parsing (now using 0x2B4)
- [x] Added DCDCMainState and DCDCStatusFlag enums
- [x] Expanded DCDCStatus struct with all status flags
- [x] Implemented complete status flag parsing per DBC
- [x] Added DCDC state to debug output

**DBC shows 0x224 contains status flags:**

```
SG_ PCS_dcdcPrechargeStatus:        0|2@1+   - IDLE/ACTIVE/FAULTED
SG_ PCS_dcdc12VSupportStatus:       2|2@1+   - IDLE/ACTIVE/FAULTED
SG_ PCS_dcdcHvBusDischargeStatus:   4|2@1+   - IDLE/ACTIVE/FAULTED
SG_ PCS_dcdcMainState:              6|4@1+   - STANDBY/12V_SUPPORT/PRECHARGE/etc.
SG_ PCS_dcdcSubState:               10|5@1+  - Sub-state number
SG_ PCS_dcdcFaulted:                15|1@1+  - Fault flag
SG_ PCS_dcdcOutputIsLimited:        28|1@1+  - Current limiting active
SG_ PCS_dcdcMaxOutputCurrentAllowed: 29|12@1+ (0.1) "A" - Max current limit
SG_ PCS_dcdcPwmEnableLine:          52|1@1+  - PWM enable
SG_ PCS_dcdcSupportingFixedLvTarget: 53|1@1+ - Fixed LV target mode
```

**✅ IMPLEMENTED:** Full status flag parsing, DCDC current now from 0x2B4.

---

### 4. Update 0x204 Handler (PCS_chgStatus) - COMPLETED ✓

- [x] Added ChargeStatusFlag enum for PCS_chargeStatus
- [x] Expanded ChargerStatus struct with all DBC signals
- [x] Updated handle204() to parse all signals per DBC
- [x] Removed hw_type (not in DBC, was reverse-engineered)
- [x] Updated references in main.cpp and pcs.cpp

**DBC signals now parsed:**

```
SG_ PCS_chgMainState:        0|4@1+  - Main state (bits 0-3)
SG_ PCS_chargeStatus:        4|2@1+  - IDLE/ENABLED/FAULTED (bits 4-5)
SG_ PCS_gridConfig:          6|2@1+  - Grid config (bits 6-7)
SG_ PCS_chgPHAEnable:        8|1@1+  - Phase A enabled (bit 8)
SG_ PCS_chgPHBEnable:        9|1@1+  - Phase B enabled (bit 9)
SG_ PCS_chgPHCEnable:        10|1@1+ - Phase C enabled (bit 10)
SG_ PCS_chgInstantAcPowerAvailable: 16|8@1+ (0.1) "kW" (bits 16-23)
SG_ PCS_chgMaxAcPowerAvailable:     24|8@1+ (0.1) "kW" (bits 24-31)
SG_ PCS_chgPHALineCurrentRequest: 32|8@1+ (0.1) "A" (bits 32-39)
SG_ PCS_chgPHBLineCurrentRequest: 40|8@1+ (0.1) "A" (bits 40-47)
SG_ PCS_chgPHCLineCurrentRequest: 48|8@1+ (0.1) "A" (bits 48-55)
SG_ PCS_chgPwmEnableLine:    56|1@1+ (bit 56)
```

---

### 5. Update 0x264 Handler (PCS_chgLineStatus) - COMPLETED ✓

- [x] Fixed current_a bit extraction bug (was using wrong shift/mask)
- [x] Added DBC comments to all signal extractions
- [x] Verified all signals match DBC

**DBC definition:**

```
SG_ PCS_chgInputVoltage:   0|14@1+  (0.033,0)  "V"  - bits 0-13 ✓
SG_ PCS_chgLineCurrent:    14|9@1+  (0.1,0)    "A"  - bits 14-22 ✓ (FIXED)
SG_ PCS_chgInputPower:     24|8@1+  (0.1,0)    "kW" - bits 24-31 ✓
SG_ PCS_chgAcCurrentLimit: 32|10@1+ (0.1,0)    "A"  - bits 32-41 ✓
```

**Bug fixed in current_a extraction:**

```cpp
// OLD (buggy): ((bytes[2] << 9 | bytes[1]) >> 7) * 0.1f
// NEW (correct): (((bytes[2] << 8 | bytes[1]) >> 6) & 0x1FF) * 0.1f
```

---

### 6. Update 0x2C4 Handler (PCS_logging) - COMPLETED ✓

- [x] Added LogMessageSelect enum (mux values 0-24)
- [x] Removed obsolete 0xE6/0xC6 voltage extraction (now using 0x2B4)
- [x] Added PhaseLoggingData struct for per-phase data (mux 0/1/2)
- [x] Added ChargerLineVoltageData struct (mux 3 / CHG_1)
- [x] Added ChargerFrequencyData struct (mux 4 / CHG_2)
- [x] Added ChargerPhaseStateData struct (mux 5 / CHG_3)
- [x] Added DCDCLoggingData struct (mux 6 / DCDC_1)
- [x] Updated handle2C4 to parse all implemented mux values per DBC
- [x] Added accessor methods for all new logging data

**DBC shows 5-bit mux (0-24):**

```
SG_ PCS_logMessageSelect M: 0|5@1+

VAL_ 708 PCS_logMessageSelect:
  0 "PHA_1"    1 "PHB_1"    2 "PHC_1"
  3 "CHG_1"    4 "CHG_2"    5 "CHG_3"
  6 "DCDC_1"   7 "DCDC_2"   8 "DCDC_3"
  9 "SYSTEM_1" ...
```

**Implemented mux handlers:**

- Mux 0/1/2 (PHA_1/PHB_1/PHC_1): Per-phase AC input current, internal bus voltage, DC output current
- Mux 3 (CHG_1): Line voltages (L1N, L2N, L3N, L1L2, NG)
- Mux 4 (CHG_2): Line frequencies and charger output voltage
- Mux 5 (CHG_3): Phase states, shutdown reasons, PLL lock status
- Mux 6 (DCDC_1): DCDC current limits

**NOTE:** Old 0xE6/0xC6 voltage extraction removed since 0x2B4 now provides accurate HV/LV voltage readings.

---

### 7. Update State Machine Enums - COMPLETED ✓

- [x] Fixed PCSChargeStatus enum: added SYSTEM_CONFIG (value 5), moved ENABLE to value 6
- [x] Updated PCSGridConfig enum: renamed NONE to SNA per DBC
- [x] Verified DCDCMainState enum matches DBC (already correct)
- [x] Added DBC reference comments to all state enums

**PCS_chgMainState (PCSChargeStatus):**

```
VAL_ 516 PCS_chgMainState:
  0 "INIT"
  1 "IDLE"
  2 "STARTUP"
  3 "WAIT_FOR_LINE_VOLTAGE"
  4 "QUALIFY_LINE_CONFIG"
  5 "SYSTEM_CONFIG"        ← was missing, now added
  6 "ENABLE"               ← was incorrectly at 5, now fixed
  7 "SHUTDOWN"
  8 "FAULTED"
  9 "CLEAR_FAULTS"
```

**PCS_dcdcMainState (DCDCMainState):** ✓ Already correct

```
VAL_ 548 PCS_dcdcMainState:
  0 "STANDBY"
  1 "12V_SUPPORT_ACTIVE"
  2 "PRECHARGE_STARTUP"
  3 "PRECHARGE_ACTIVE"
  4 "DIS_HVBUS_ACTIVE"
  5 "SHUTDOWN"
  6 "FAULTED"
```

**PCS_gridConfig (PCSGridConfig):**

```
VAL_ 516 PCS_gridConfig:
  0 "SNA"                  ← renamed from "NONE"
  1 "SINGLE_PHASE"
  2 "THREE_PHASE"
  3 "THREE_PHASE_DELTA"
```

---

### 8. Alert Matrix (0x3A4 / PCS_alertMatrix) - COMPLETED ✓

- [x] Added debug logging to investigate 0x3A4
- [x] Confirmed 2017 PCS sends BOTH 0x3A4 (matrix) and 0x424 (individual)
- [x] Added AlertMatrixState struct to track all active alerts
- [x] Implemented handle3A4() with state tracking and change detection
- [x] Added is_alert_active(alert_num) query function
- [x] Added get_active_alert_count() for total fault count
- [x] Reduced debug verbosity (only logs on change)

**DBC structure:**

```
BO_ 932 PCS_alertMatrix: 8 VEH
 SG_ PCS_matrixIndex M: 0|4@1+
 SG_ PCS_a001_chgHwInputOc m0: 4|1@1+
 SG_ PCS_a002_chgHwOutputOc m0: 5|1@1+
 ... (60 alerts per page, 2 pages)
```

**✅ VERIFIED on 2017 PCS:** Both message types work:

- **0x3A4 (Alert Matrix)**: Continuously broadcasts bitmap of ALL active alerts
  - Alternates between page 0 (alerts 1-60) and page 1 (alerts 61-120)
  - Useful for polling current fault state
- **0x424 (Individual Alert)**: Fires once when each alert triggers
  - Includes extra detail for some alerts (e.g., CAN rationality shows offending message ID)
  - Used for event-driven error posting

**API for querying alerts:**

```cpp
PCSCan::is_alert_active(8);       // Check if a008 (chgPhaseOverTemp) active
PCSCan::get_active_alert_count(); // Total active alerts (both pages)
PCSCan::get_alert_matrix();       // Full AlertMatrixState struct
```

---

### 9. BMS_chargerRequest Fault Clear (0x2B2) - COMPLETED ✓

- [x] Added BMS_pcsClearFaultRequest signal (bit 16)
- [x] Added `PCSCan::request_clear_faults()` one-shot method
- [x] Added `PCSController::clear_faults_async()` for thread-safe use
- [x] DBC comments added to Msg2B2

**DBC definition:**

```
BO_ 690 BMS_chargerRequest: 3 VEH
 SG_ BMS_acChargePowerRequest: 0|16@1+ (0.001,0) "kW"   // bits 0-15
 SG_ BMS_pcsClearFaultRequest: 16|1@1+ (1,0) ""         // bit 16
 SG_ BMS_acChargeEnable: 17|1@1+ (1,0) ""               // bit 17
```

**Usage:**

```cpp
PCSController::clear_faults_async();  // Thread-safe from any task
PCSCan::request_clear_faults();       // Direct call (one-shot)
```

---

### 10. Add 0x3C4 Handler (PCS_info) - COMPLETED ✓

- [x] Added PCSInfoData struct with build_type, hardware_id, component_id, platform_type, app_crc, part_number
- [x] Implemented handle3C4() with mux parsing for pages 10, 13, 25, 26
- [x] Added accessor method get_pcs_info()
- [x] Added 0x3C4 to process_frame switch

**DBC structure (multiplexed):**

```
BO_ 964 PCS_info: 8 VEH
 SG_ PCS_infoIndex M: 0|8@1+
 SG_ PCS_buildType m10: 8|8@1+            // UNKNOWN/PLATFORM/LOCAL/CI/MFG
 SG_ PCS_hardwareId m10: 32|16@1+
 SG_ PCS_platformType m13: 8|8@1+
 SG_ PCS_appCrc m13: 32|32@1+
 SG_ PCS_partNumChar01-14 m25-27: 8|8@1+  // Part number string
```

**API:**

```cpp
PCSCan::get_pcs_info();  // Returns PCSInfoData with hardware ID, part number, etc.
```

---

### 11. Add 0x2C4 Mux 7 Handler (DCDC_2) - COMPLETED ✓

- [x] Added DCDCControlData struct with tank_voltage_v, tank_voltage_target_v, switching_freq_khz
- [x] Added LOG_DCDC_2 case to handle2C4() with proper bit extraction
- [x] Added accessor method get_dcdc_control()

**DBC signals:**

```
SG_ PCS_dcdcTankVoltage m7:       26|11@1- (1,0) "V"      // 11-bit signed
SG_ PCS_dcdcTankVoltageTarget m7: 37|10@1+ (1,0) "V"
SG_ PCS_dcdcClaCurrentFreq m7:    48|12@1+ (0.0976563,0) "kHz"
```

**API:**

```cpp
PCSCan::get_dcdc_control();  // Returns DCDCControlData with tank voltage, switching frequency
```

---

### 12. Verify BMS_acChargePowerRequest Scale (0x2B2)

DBC shows power in kW with scale 0.001, but current code sends watts.

**DBC definition:**

```
SG_ BMS_acChargePowerRequest: 0|16@1+ (0.001,0) "kW"
```

**Current code:** Sends raw watts (e.g., 7000 for 7kW)
**DBC expects:** kW × 1000 (e.g., 7000 for 7kW)

**Note:** These happen to be equivalent! 7000W = 7kW × 1000 = 7000
The math works out, but we should verify the PCS interprets it correctly.

**Action:** Test and document whether current implementation is correct.

---

## Debug Tasks

### Add Raw Message Logging

For messages with suspected format differences, add temporary debug output:

```cpp
DEBUG_SERIAL.printf("0x2A4 raw: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    bytes[0], bytes[1], bytes[2], bytes[3],
                    bytes[4], bytes[5], bytes[6], bytes[7]);
```

Messages to capture:

- [ ] 0x2A4 - Temperature (DCDC showing 139°C)
- [ ] 0x2C4 - Logging (different mux scheme)
- [ ] 0x2B4 - Check if 2017 PCS sends this message

---

## Reference: Full DBC Definitions

### PCS_chgStatus (0x204 / 516)

```
BO_ 516 PCS_chgStatus: 8 VEH
 SG_ PCS_chargeStatus: 4|2@1+ (1,0) ""
 SG_ PCS_chgInstantAcPowerAvailable: 16|8@1+ (0.1,0) "kW"
 SG_ PCS_chgMainState: 0|4@1+ (1,0) ""
 SG_ PCS_chgMaxAcPowerAvailable: 24|8@1+ (0.1,0) "kW"
 SG_ PCS_chgPHAEnable: 8|1@1+ (1,0) ""
 SG_ PCS_chgPHALineCurrentRequest: 32|8@1+ (0.1,0) "A"
 SG_ PCS_chgPHBEnable: 9|1@1+ (1,0) ""
 SG_ PCS_chgPHBLineCurrentRequest: 40|8@1+ (0.1,0) "A"
 SG_ PCS_chgPHCEnable: 10|1@1+ (1,0) ""
 SG_ PCS_chgPHCLineCurrentRequest: 48|8@1+ (0.1,0) "A"
 SG_ PCS_chgPwmEnableLine: 56|1@1+ (1,0) ""
 SG_ PCS_gridConfig: 6|2@1+ (1,0) ""
```

### PCS_dcdcStatus (0x224 / 548)

```
BO_ 548 PCS_dcdcStatus: 8 VEH
 SG_ PCS_dcdc12VSupportRtyCnt: 44|4@1+ (1,0) ""
 SG_ PCS_dcdc12VSupportStatus: 2|2@1+ (1,0) ""
 SG_ PCS_dcdcDischargeRtyCnt: 48|4@1+ (1,0) ""
 SG_ PCS_dcdcFaulted: 15|1@1+ (1,0) ""
 SG_ PCS_dcdcHvBusDischargeStatus: 4|2@1+ (1,0) ""
 SG_ PCS_dcdcInitialPrechargeSubState: 59|5@1+ (1,0) ""
 SG_ PCS_dcdcMainState: 6|4@1+ (1,0) ""
 SG_ PCS_dcdcMaxOutputCurrentAllowed: 29|12@1+ (0.1,0) "A"
 SG_ PCS_dcdcOutputIsLimited: 28|1@1+ (1,0) ""
 SG_ PCS_dcdcPrechargeRestartCnt: 56|3@1+ (1,0) ""
 SG_ PCS_dcdcPrechargeRtyCnt: 41|3@1+ (1,0) ""
 SG_ PCS_dcdcPrechargeStatus: 0|2@1+ (1,0) ""
 SG_ PCS_dcdcPwmEnableLine: 52|1@1+ (1,0) ""
 SG_ PCS_dcdcSubState: 10|5@1+ (1,0) ""
 SG_ PCS_dcdcSupportingFixedLvTarget: 53|1@1+ (1,0) ""
 SG_ PCS_ecuLogUploadRequest: 54|2@1+ (1,0) ""
```

### PCS_chgLineStatus (0x264 / 612)

```
BO_ 612 PCS_chgLineStatus: 6 VEH
 SG_ PCS_chgAcCurrentLimit: 32|10@1+ (0.1,0) "A"
 SG_ PCS_chgInputPower: 24|8@1+ (0.1,0) "kW"
 SG_ PCS_chgInputVoltage: 0|14@1+ (0.033,0) "V"
 SG_ PCS_chgLineCurrent: 14|9@1+ (0.1,0) "A"
```

### PCS_thermalStatus (0x2A4 / 676)

```
BO_ 676 PCS_thermalStatus: 8 VEH
 SG_ PCS_ambientTemp: 48|11@1- (0.1,40) "C"
 SG_ PCS_chgPhATemp: 0|11@1- (0.1,40) "C"
 SG_ PCS_chgPhBTemp: 11|11@1- (0.1,40) "C"
 SG_ PCS_chgPhCTemp: 24|11@1- (0.1,40) "C"
 SG_ PCS_dcdcTemp: 35|11@1- (0.1,40) "C"
```

### PCS_dcdcBusStatus (0x2B4 / 692) - NOT CURRENTLY HANDLED

```
BO_ 692 PCS_dcdcBusStatus: 5 VEH
 SG_ PCS_dcdcHvBusVolt: 10|12@1+ (0.146484375,0) "V"
 SG_ PCS_dcdcLvBusVolt: 0|10@1+ (0.0390625,0) "V"
 SG_ PCS_dcdcLvOutputCurrent: 24|12@1+ (0.1,0) "A"
```

### BMS_chargerRequest (0x2B2 / 690) - TX Message

```
BO_ 690 BMS_chargerRequest: 3 VEH
 SG_ BMS_acChargePowerRequest: 0|16@1+ (0.001,0) "kW"
 SG_ BMS_pcsClearFaultRequest: 16|1@1+ (1,0) ""
 SG_ BMS_acChargeEnable: 17|1@1+ (1,0) ""
```

Note: Current code sends power in watts (scale 1), DBC shows kW (scale 0.001).
This may explain the 3-byte vs 5-byte format difference.
