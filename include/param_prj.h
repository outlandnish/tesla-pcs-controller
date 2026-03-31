/*
 * Tesla Model 3 PCS Controller - Parameter Definitions
 *
 * This file defines all parameters used in the project.
 * Parameters are accessed via the Param class from libopeninv-arduino.
 */

#ifndef PARAM_PRJ_H_INCLUDED
#define PARAM_PRJ_H_INCLUDED

#include "hwdefs.h"

// Define firmware version components
#ifndef VER_MAJOR
#define VER_MAJOR 1
#endif
#ifndef VER_MINOR
#define VER_MINOR 0
#endif
#ifndef VER_PATCH
#define VER_PATCH 0
#endif
#ifndef VER_DIRTY
#define VER_DIRTY 1  // 1 for dev builds, 0 for clean release builds
#endif

// Construct version string
#if VER_DIRTY
#define VER_STR "pcs-" STRINGIFY(VER_MAJOR) "." STRINGIFY(VER_MINOR) "." STRINGIFY(VER_PATCH) "-dirty"
#else
#define VER_STR "pcs-" STRINGIFY(VER_MAJOR) "." STRINGIFY(VER_MINOR) "." STRINGIFY(VER_PATCH)
#endif

// Numeric version (MAJOR*100 + MINOR*10 + PATCH)
#define VER_NUM (VER_MAJOR * 100 + VER_MINOR * 10 + VER_PATCH)

// Version token for VERSTR (unquoted for STRINGIFY macro)
#if VER_DIRTY
#define VERSION pcs-VER_MAJOR.VER_MINOR.VER_PATCH-dirty
#else
#define VERSION pcs-VER_MAJOR.VER_MINOR.VER_PATCH
#endif

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */

// Parameter default values (used by both parameter system and initial hardware state)
#define UDCSPNT_DEFAULT 403
#define UDCDC_DEFAULT 14
#define IACLIM_DEFAULT 16
#define CHGTERMN_DEFAULT 80

// Parameter and value definitions
/*              category     displayName           name         unit       min     max     default id */
#define PARAM_LIST \
  PARAM_ENTRY(CAT_CHARGER, "Power Setpoint", pacspnt, "kW", 0, 11, 11, 2) \
  PARAM_ENTRY(CAT_CHARGER, "DC Current Limit", idclim, "A", 0, 45, 45, 3) \
  PARAM_ENTRY(CAT_CHARGER, "AC Current Limit", iaclim, "A", 0, 72, IACLIM_DEFAULT, 4) \
  PARAM_ENTRY(CAT_CHARGER, "DC Voltage Setpoint", udcspnt, "V", 50, 420, UDCSPNT_DEFAULT, 6) \
  PARAM_ENTRY(CAT_CHARGER, "DC Voltage Limit", udclim, "V", 50, 420, 398, 7) \
  PARAM_ENTRY(CAT_CHARGER, "Time Limit", timelim, "minutes", -1, 10000, -1, 8) \
  PARAM_ENTRY(CAT_CHARGER, "Charge Termination", chgtermn, "%", 0, 100, CHGTERMN_DEFAULT, 9) \
  PARAM_ENTRY(CAT_CHARGER, "Input Type", inputype, INPUTS, 0, 3, 0, 10) \
  PARAM_ENTRY(CAT_CHARGER, "Activate Devices", activate, DEVS, 0, 3, 3, 14) \
  PARAM_ENTRY(CAT_DCDC, "DC/DC Voltage", udcdc, "V", 12, 15, UDCDC_DEFAULT, 15) \
  PARAM_ENTRY(CAT_DCDC, "Precharge Time", PreChT, "sec", 1, 10, 3, 23) \
  PARAM_ENTRY(CAT_DIAG, "Mode Control", modectl, OPMODES, 0, 2, 0, 30) \
  PARAM_ENTRY(CAT_DIAG, "Clear Faults", clearfaults, OFFON, 0, 1, 0, 31) \
  VALUE_ENTRY("Operating Mode", opmode, OPMODES, 2000) \
  VALUE_ENTRY("State", state, STATES, 2001) \
  VALUE_ENTRY("Last Error", lasterr, "", 2002) \
  VALUE_ENTRY("Version", version, VERSTR, 2003) \
  VALUE_ENTRY("Auxiliary Voltage", uaux, "V", 2005) \
  VALUE_ENTRY("Hardware AC Limit", hwaclim, "A", 2007) \
  VALUE_ENTRY("Cable Limit", cablelim, "A", 2008) \
  VALUE_ENTRY("EVSE Limit", evselim, "A", 2009) \
  VALUE_ENTRY("AC Power", powerac, "kW", 2010) \
  VALUE_ENTRY("DC/DC Power", powerdcdc, "W", 2011) \
  VALUE_ENTRY("DC Voltage", udc, "V", 2012) \
  VALUE_ENTRY("Low Voltage", ulv, "V", 2013) \
  VALUE_ENTRY("AC Voltage", uac, "V", 2014) \
  VALUE_ENTRY("AC Current", iac, "A", 2015) \
  VALUE_ENTRY("DC Current", idc, "A", 2016) \
  VALUE_ENTRY("DC/DC Current", idcdc, "A", 2017) \
  VALUE_ENTRY("Charger AC Limit", ChgACLim, "A", 2018) \
  VALUE_ENTRY("PCS Type", PCS_Type, TYPES, 2019) \
  VALUE_ENTRY("Proximity", proximity, OFFON, 2020) \
  VALUE_ENTRY("Enable", enable, OFFON, 2021) \
  VALUE_ENTRY("CPU Load", cpuload, "%", 2024) \
  VALUE_ENTRY("Charge Status", CHG_STAT, C_STAT, 2025) \
  VALUE_ENTRY("Charge Power Available", CHGPAvail, "kW", 2026) \
  VALUE_ENTRY("Grid Configuration", GridCFG, GCFG, 2027) \
  VALUE_ENTRY("Charger A Temp", ChgATemp, "C", 2031) \
  VALUE_ENTRY("Charger B Temp", ChgBTemp, "C", 2032) \
  VALUE_ENTRY("Charger C Temp", ChgCTemp, "C", 2033) \
  VALUE_ENTRY("DC/DC Temp", DCDCTemp, "C", 2034) \
  VALUE_ENTRY("Drive Enable", Drive_En, OFFON, 2038) \
  VALUE_ENTRY("Uptime", uptime, "s", 2048)

/***** Enum String definitions *****/
#define OPMODES "0=Off, 1=Run, 2=Drive"
#define OFFON "0=Off, 1=On"
#define TYPES "0=48A_1P, 1=32A_1P, 2=16A_3P"
#define GCFG "0=None, 1=1P, 2=3P, 3=3PD"
#define STATES "0=Off, 1=WaitStart, 2=Enable, 3=Activate, 4=Run, 5=Stop, 6=DRIVE"
#define INPUTS "0=Type2, 1=Type2_3P, 2=Type1, 3=Manual"
#define DEVS "0=None, 1=Charger, 2=DC-DC, 3=Both"
#define C_STAT "0=Init, 1=Idle, 2=Startup, 3=WaitAC, 4=Qualify, 5=Config, 6=Enable, 7=Shutdown, 8=Faulted, 9=CLRFaults"

// Parameter categories
#define CAT_CHARGER "Charger"
#define CAT_DCDC "DC/DC Converter"
#define CAT_GEN "General"
#define CAT_DIAG "Diagnostic"

/***** enums ******/
enum inputs {
  INP_TYPE2,
  INP_TYPE2_3P,
  INP_TYPE1,
  INP_MANUAL
};

#define VERSTR STRINGIFY(4=VERSION)

#endif // PARAM_PRJ_H_INCLUDED
