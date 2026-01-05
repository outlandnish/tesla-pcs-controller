/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 * Arduino/STM32F413CG implementation using libopeninv-arduino
 *
 * Based on original firmware by Damien Maguire
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 *
 * CAN Bus Configuration:
 *   CAN1 (PA11/PA12): IPC CAN to PCS - Bidirectional control communication
 *   CAN2 (PB5/PB13):  CP_CAN - Charge Port CAN bus monitoring (PCS <-> CP ECU)
 *   CAN3 (PB3/PA15):  OpenInverter protocol - SDO/parameter access
 */

// Debug configuration - set to 1 to enable verbose output
#define DEBUG_PCS_STATE 1      // Enable PCS state/voltage debug output
#define DEBUG_CAN_TRAFFIC 0    // Enable CAN RX/TX message logging (CAN2 & CAN3)
#define DEBUG_SDO_PARAMS 0     // Enable SDO parameter read/write logging
#define DEBUG_PCS_CAN 1        // Enable PCS CAN RX message statistics (CAN1)
#define DEBUG_PCS_TX 0         // Enable PCS CAN TX message logging (CAN1)
#define DEBUG_PCS_RX 0         // Enable PCS CAN RX message logging (CAN1)
#define DEBUG_CP_CAN 0         // Enable CP_CAN message logging (CAN2)

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "hwdefs.h"
#include "serial_config.h"  // Common serial configuration
#include "debug_serial.h"   // DEBUG_SERIAL macro
#include "param_prj.h"
#include "params.h"
#include "param_save.h"
#include "my_string.h"
#include "my_fp.h"
#include "errormessage.h"
#include "printf.h"
#include "canhardware_arduino.h"
#include "canmap.h"
#include "cansdo.h"
#include "pcs.h"
#include "pcs-can.h"

// Define Serial1 instance (declared extern in serial_config.h)
HardwareSerial Serial1(USART1);

// CAN interfaces
// CAN1: Tesla Model 3 PCS IPC communication (PA11/PA12)
CANBus ipcCan(PIN_CANRX0, PIN_CANTX0);

// CAN2: CP_CAN for PCS and Charge Port ECU (PB5/PB13)
CANBus cpCan(PIN_CANRX1, PIN_CANTX1);

// CAN3: OpenInverter protocol (PB3/PA15)
CANBus openinvCan(PIN_CANRX2, PIN_CANTX2);

// OpenInverter CAN hardware handler
CanHardwareArduino* canHardware;
CanMap* canMap;
CanSdo* canSdo;

// Timing variables
static uint32_t lastMillis = 0;
static uint32_t uptime = 0;

// Forward declarations
void setupPins();
void readInputs();
void updateParameters();
void openinv_task(void *pvParameters);
void buildParameterJson();

// Pre-generated parameter JSON for web interface
String parameterJson;

// Data source callback for SDO data transfer
// Used for JSON (0x5001)
// Returns byte at offset or -1 if out of range
int getDataByte(uint32_t offset) {
  if (offset >= parameterJson.length()) {
    return -1;  // End of data
  }
  return (int)(uint8_t)parameterJson[offset];
}

// Generate parameter JSON
void buildParameterJson() {
  // Clear any existing content
  parameterJson = String();
  
  JsonDocument doc;

  for (int i = 0; i < Param::PARAM_LAST; i++) {
    const Param::Attributes* attr = Param::GetAttrib((Param::PARAM_NUM)i);
    if (!attr) continue;

    JsonObject param = doc[attr->name].to<JsonObject>();
    param["unit"] = attr->unit;
    param["name"] = attr->displayName;
    param["category"] = attr->category;
    param["minimum"] = attr->min;
    param["maximum"] = attr->max;
    param["default"] = attr->def;
    param["id"] = attr->id;
    param["isparam"] = (Param::GetType((Param::PARAM_NUM)i) == Param::TYPE_PARAM) ? 1 : 0;
    param["value"] = Param::GetFloat((Param::PARAM_NUM)i);
  }

  serializeJson(doc, parameterJson);
  
  // Update CanSdo with actual JSON size (needed for SDO size query)
  if (canSdo != nullptr) {
    canSdo->SetJsonSize(parameterJson.length());
  }
}

void setup() {
  // Initialize serial for debug (USART1 on PB6/PB7)
  // Must use ALT pin definitions for correct alternate function mapping
  Serial.setRx(PB7_ALT0);  // USART1 RX
  Serial.setTx(PB6_ALT1);  // USART1 TX
  Serial.begin(115200);
  
  Serial.println("\n\n========================================");
  Serial.println("Tesla Model 3 PCS Controller " VER_STR);
  Serial.println("========================================");

  // Create parameter mutex for thread-safe access
  // Initialize parameter system (creates mutex)
  Param::Init();

  // Load default parameters
  Param::LoadDefaults();

  // Load saved parameters from Flash
  parm_load();
  
  // Initialize all spot value parameters to 0 to prevent garbage values
  Param::SetInt(Param::opmode, 0);
  Param::SetInt(Param::state, 0);
  Param::SetInt(Param::lasterr, 0);
  Param::SetInt(Param::version, VER_NUM);
  Param::SetFloat(Param::uaux, 0);
  Param::SetFloat(Param::hwaclim, 0);
  Param::SetFloat(Param::cablelim, 0);
  Param::SetFloat(Param::evselim, 0);
  Param::SetFloat(Param::powerac, 0);
  Param::SetFloat(Param::powerdcdc, 0);
  Param::SetFloat(Param::udc, 0);
  Param::SetFloat(Param::ulv, 0);
  Param::SetFloat(Param::uac, 0);
  Param::SetFloat(Param::iac, 0);
  Param::SetFloat(Param::idc, 0);
  Param::SetFloat(Param::idcdc, 0);
  Param::SetFloat(Param::ChgACLim, 0);
  Param::SetInt(Param::PCS_Type, 0);
  Param::SetInt(Param::proximity, 0);
  Param::SetInt(Param::enable, 0);
  Param::SetInt(Param::cpuload, 0);
  Param::SetInt(Param::CHG_STAT, 0);
  Param::SetFloat(Param::CHGPAvail, 0);
  Param::SetInt(Param::GridCFG, 0);
  Param::SetFloat(Param::ChgATemp, 0);
  Param::SetFloat(Param::ChgBTemp, 0);
  Param::SetFloat(Param::ChgCTemp, 0);
  Param::SetFloat(Param::DCDCTemp, 0);
  Param::SetInt(Param::Drive_En, 0);
  Param::SetInt(Param::uptime, 0);
  
  // Debug: Print some key parameter defaults to verify they loaded
  Serial.printf("Parameter defaults loaded:\n");
  Serial.printf("  pacspnt (Power) = %d kW\n", Param::GetInt(Param::pacspnt));
  Serial.printf("  iaclim (AC Limit) = %d A\n", Param::GetInt(Param::iaclim));
  Serial.printf("  udcspnt (DC Voltage) = %d V\n", Param::GetInt(Param::udcspnt));
  Serial.printf("  version = %d (%s)\n", Param::GetInt(Param::version), VER_STR);

  // Configure pins
  setupPins();

  // Initialize CAN1 for Tesla PCS IPC (500 kbps)
  Serial.println("Initializing CAN1 (PA11/PA12)...");
  if (!ipcCan.begin(CAN_BPS_500K)) {
    Serial.println("ERROR: Failed to initialize CAN1 (Tesla PCS IPC)!");
    while(1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
  Serial.println("CAN1 (Tesla PCS IPC) initialized at 500 kbps");

  // Run loopback test to verify CAN hardware
  Serial.println("Running CAN1 loopback test...");
  if (ipcCan.runLoopbackTest(0x123, 100)) {
    Serial.println("CAN1 loopback PASSED - hardware OK");
  } else {
    Serial.println("CAN1 loopback FAILED - check CAN hardware");
  }

  // Initialize CAN2 for CP_CAN monitoring (500 kbps)
  Serial.println("Initializing CAN2 (PB5/PB13)...");
  if (!cpCan.begin(CAN_BPS_500K)) {
    Serial.println("ERROR: Failed to initialize CAN2 (CP_CAN)!");
    while(1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(150);
    }
  }
  Serial.println("CAN2 (CP_CAN) initialized at 500 kbps");

  // Initialize CAN3 for OpenInverter protocol (500 kbps)
  Serial.println("Initializing CAN3 (PB3/PA15)...");
  if (!openinvCan.begin(CAN_BPS_500K)) {
    Serial.println("ERROR: Failed to initialize CAN3 (OpenInverter)!");
    while(1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(200);
    }
  }
  Serial.println("CAN3 (OpenInverter) initialized at 500 kbps");

  // Initialize OpenInverter CAN handler
  Serial.println("Initializing CanHardware...");
  canHardware = new CanHardwareArduino(&openinvCan);
  Serial.println("OpenInverter CAN hardware initialized");

  // Initialize CAN mapping and SDO handlers
  Serial.println("Initializing CanMap and CanSdo...");
  canMap = new CanMap(canHardware);
  canSdo = new CanSdo(canHardware, canMap);

  uint8_t canNodeId = 2;  // Change to 2 if you're sending to 0x602
  canSdo->SetNodeId(canNodeId);
  Serial.printf("CAN Node ID set to %d (listening on 0x%03X)\r\n", canNodeId, 0x600 + canNodeId);

  // Callbacks will be registered in openinv_task to avoid threading issues
  Serial.println("CanSdo initialized - callbacks will be set in task");

  // Initialize PCS controller
  Serial.println("Initializing PCS controller...");
  PCSController::begin(&ipcCan, PIN_PCS_ENABLE, PIN_CHARGE_ENABLE, PIN_DCDC_ENABLE);

  // Start PCS FreeRTOS task
  Serial.println("Starting PCS task...");
  if (!PCSController::start_task()) {
    Serial.println("ERROR: Failed to start PCS task!");
    while(1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
  Serial.println("PCS task started");

  // Start OpenInverter FreeRTOS task
  Serial.println("Starting OpenInverter task...");
  BaseType_t result = xTaskCreate(
    openinv_task,
    "OpenInv",
    2048,  // Stack size
    NULL,
    2,     // Priority (same as PCS task)
    NULL
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to start OpenInverter task!");
    while(1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(150);
    }
  }
  Serial.println("OpenInverter task started");

  Serial.println("\n*** Initialization complete - Starting RTOS ***\n");

  // Start FreeRTOS scheduler (this should never return)
  vTaskStartScheduler();

  // If we get here, scheduler failed to start
  Serial.println("FATAL: FreeRTOS scheduler failed to start!");
  while(1) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(50);  // Very fast blink = scheduler failure
  }

  lastMillis = millis();
}

void loop() {
  // Note: With FreeRTOS scheduler running, loop() may not execute
  // or executes as the idle task. Keep minimal processing here.

  // Small delay
  delay(10);
}

void setupPins() {
  // Configure digital outputs
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PCS_ENABLE, OUTPUT);
  pinMode(PIN_CHARGE_ENABLE, OUTPUT);
  pinMode(PIN_DCDC_ENABLE, OUTPUT);
  pinMode(PIN_EVSE_SWITCH, OUTPUT);

  // Configure digital inputs
  pinMode(PB4, INPUT);  // Drive mode input

  // Set initial states (all disabled)
  digitalWrite(PIN_PCS_ENABLE, LOW);
  digitalWrite(PIN_CHARGE_ENABLE, HIGH);  // Active low
  digitalWrite(PIN_DCDC_ENABLE, HIGH);    // Active low
  digitalWrite(PIN_EVSE_SWITCH, LOW);
  digitalWrite(PIN_LED, LOW);
}

void readInputs() {
  // Read drive enable input
  bool driveEnable = digitalRead(PB4);
  Param::SetInt(Param::Drive_En, driveEnable ? 1 : 0);

  // Read auxiliary voltage (12V monitoring)
  // Scale factor: assuming voltage divider for 0-16V range on 3.3V ADC
  int auxRaw = analogRead(PIN_PILOT_SENSE);
  float uaux = (auxRaw / 4095.0f) * 3.3f * 5.0f;  // Adjust multiplier based on actual divider
  Param::SetFloat(Param::uaux, uaux);

  // TODO: Read and decode EVSE pilot signal for cable limit and proximity
  // This requires PWM decoding on TIM3 (similar to original firmware)
}

void updateParameters() {
  static uint32_t lastDebug = 0;
  static bool debugPrinted = false;
  
  // Disable interrupts briefly to prevent race conditions with SDO reads
  // This is safe because parameter updates are fast (~few microseconds)
  taskENTER_CRITICAL();
  
  // Get PCS state
  PCSState state = PCSController::get_state();
  Param::SetInt(Param::state, (int)state);

  // Debug: Print state once every 5 seconds
  #if DEBUG_PCS_STATE
  if (millis() - lastDebug > 5000 && !debugPrinted) {
    Serial.printf("PCS State: %d\n", (int)state);
    debugPrinted = true;
  }
  if (millis() - lastDebug > 5000) {
    lastDebug = millis();
    debugPrinted = false;
  }
  #endif

  // Update operation mode based on state
  int opmode = 0;  // Off
  if (state == PCS_STATE_CHARGING) {
    opmode = 1;  // Run (charging)
  } else if (state == PCS_STATE_DRIVE) {
    opmode = 2;  // Drive
  }
  Param::SetInt(Param::opmode, opmode);

  // Get charger status from PCS
  const ChargerStatus& chgStatus = PCSController::get_charger_status();
  Param::SetInt(Param::CHG_STAT, chgStatus.main_state);
  Param::SetInt(Param::GridCFG, chgStatus.grid_config);
  Param::SetFloat(Param::CHGPAvail, chgStatus.max_power_available_kw);

  // Get AC status
  const ACStatus& acStatus = PCSController::get_ac_status();
  Param::SetFloat(Param::powerac, acStatus.power_kw);
  Param::SetFloat(Param::uac, acStatus.voltage_v);
  Param::SetFloat(Param::iac, acStatus.current_a);
  Param::SetFloat(Param::ChgACLim, acStatus.current_limit_a);

  // Get DCDC status
  const DCDCStatus& dcdcStatus = PCSController::get_dcdc_status();
  Param::SetFloat(Param::idcdc, dcdcStatus.current_a);
  Param::SetFloat(Param::powerdcdc, dcdcStatus.power_w);

  // Get voltage data
  const VoltageData& voltage = PCSController::get_voltage_data();
  Param::SetFloat(Param::udc, voltage.hv_v);
  Param::SetFloat(Param::ulv, voltage.lv_v);

  // Get DC current data
  const DCCurrentData& dcCurrent = PCSController::get_dc_current_data();
  Param::SetFloat(Param::idc, dcCurrent.total_a);

  // Get temperature data (already converted to °C per DBC)
  const TemperatureData& temps = PCSController::get_temperature_data();
  Param::SetFloat(Param::ChgATemp, temps.phase_a_c);
  Param::SetFloat(Param::ChgBTemp, temps.phase_b_c);
  Param::SetFloat(Param::ChgCTemp, temps.phase_c_c);
  Param::SetFloat(Param::DCDCTemp, temps.dcdc_c);
  
  // Re-enable interrupts
  taskEXIT_CRITICAL();
}

// OpenInverter FreeRTOS task
void openinv_task(void *pvParameters) {
  // Register callbacks INSIDE the task to ensure proper thread initialization
  canSdo->SetDataSource(getDataByte);
  canSdo->SetJsonRebuildCallback(buildParameterJson);
  
  // Build initial JSON
  buildParameterJson();
  
  Serial.println("OpenInverter task running");
  
  // Sync loaded parameters to PCS hardware (SetFixed doesn't trigger Change callbacks)
  // Must be done here after command queue is created, not during init
  Serial.println("Syncing parameters to PCS controller...");
  
  // Parameters that control PCS behavior via IPC CAN messages:
  // - pacspnt (0x2B2) - charge power setpoint in watts
  // - udcspnt (0x22A) - HV voltage setpoint in volts
  // - udcdc (0x3A1) - DCDC voltage setpoint in volts
  // - iaclim (0x23D) - AC current limit in amps
  PCSController::set_charge_power_async(Param::GetInt(Param::pacspnt) * 1000);
  PCSController::set_hv_voltage_async(Param::GetInt(Param::udcspnt));
  PCSController::set_dcdc_voltage_async(Param::GetFloat(Param::udcdc));
  PCSController::set_ac_current_limit_async(Param::GetInt(Param::iaclim));
  
  // Note: EVSE/cable limits (0x21D) are read from external sources, not set by params
  // Note: idclim/udclim are for reference only, actual limits handled by PCS internally
  // Note: pcstype affects message format but doesn't require immediate sync

  uint32_t lastSecond = 0;

  while(1) {
    uint32_t now = millis();

    // Update error message timestamp
    ErrorMessage::SetTime(now / 1000);  // Convert to seconds

    // Update uptime counter every second
    if (now - lastSecond >= 1000) {
      uptime++;
      Param::SetInt(Param::uptime, uptime);
      lastSecond = now;

      // Toggle LED to show we're alive
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }

    // Read hardware inputs
    readInputs();

    // Update calculated parameters from PCS
    updateParameters();

    // Process incoming CAN messages on CAN2 (CP_CAN)
    uint32_t canId;
    uint8_t data[8];
    uint8_t len;

    while (cpCan.receiveMessage(canId, data, len)) {
      // Debug: CP_CAN message logging (enable DEBUG_CP_CAN flag)
      #if DEBUG_CP_CAN
      Serial.print("CAN2 (CP_CAN) RX: ID=0x");
      Serial.print(canId, HEX);
      Serial.print(" Len=");
      Serial.print(len);
      Serial.print(" Data=");
      for (uint8_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      #endif

      // CP_CAN messages are currently just monitored/logged
      // Add processing logic here as needed
    }

    // Process incoming CAN messages on CAN3 (OpenInverter protocol)
    while (openinvCan.receiveMessage(canId, data, len)) {
      // Debug: CAN traffic logging (enable DEBUG_CAN_TRAFFIC flag)
      #if DEBUG_CAN_TRAFFIC
      Serial.print("CAN3 (OpenInv) RX: ID=0x");
      Serial.print(canId, HEX);
      Serial.print(" Len=");
      Serial.print(len);
      Serial.print(" Data=");
      for (uint8_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      #endif

      // Convert to format expected by CanHardware
      uint32_t data_words[2];
      memcpy(data_words, data, 8);
      canHardware->HandleRx(canId, data_words, len);
    }

    // Trigger SDO timeout handling
    canSdo->TriggerTimeout(10);

    // Small delay to yield to other tasks (100Hz update rate)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Parameter change callback
void Param::Change(Param::PARAM_NUM paramNum) {
  // Handle parameter changes
  switch (paramNum) {
    case Param::pacspnt:
      // Update charge power setpoint
      PCSController::set_charge_power_async(Param::GetInt(Param::pacspnt) * 1000);
      break;

    case Param::udcspnt:
      // Update HV voltage setpoint
      PCSController::set_hv_voltage_async(Param::GetInt(Param::udcspnt));
      break;

    case Param::udcdc:
      // Update DCDC voltage setpoint
      PCSController::set_dcdc_voltage_async(Param::GetFloat(Param::udcdc));
      break;

    case Param::iaclim:
      // Update AC current limit
      PCSController::set_ac_current_limit_async(Param::GetInt(Param::iaclim));
      break;

    case Param::idclim:
      // DC current limit - stored for reference, actual limit controlled by PCS
      // No direct setter needed as PCS handles this internally
      break;

    case Param::udclim:
      // DC voltage limit - stored for reference, actual limit controlled by PCS
      // No direct setter needed as PCS handles this internally
      break;

    case Param::pcstype:
      // PCS Region (0=US, 1=EU) - affects 0x2B2 message format
      // No immediate action needed - next Msg2B2() call will use new format
      break;

    case Param::modectl:
      {
        // State machine mode control: 0=Off, 1=Charge, 2=Drive
        int mode = Param::GetInt(Param::modectl);
        switch (mode) {
          case 0:  // Off
            PCSController::stop_async();
            break;
          case 1:  // Charge/Run
            PCSController::start_charging_async();
            break;
          case 2:  // Drive
            PCSController::start_drive_mode_async();
            break;
          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}