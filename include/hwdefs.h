#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#include <Arduino.h>

/* Pin Definitions for STM32F413CG-based Tesla Model 3 PCS Controller
 * Based on hardware schematic (Screenshot 2025-12-13)
 *
 * NOTE: Pin numbers use Arduino STM32 pin naming
 * Format: Pxn where x=Port letter, n=Pin number
 */

// Control Outputs
#define PIN_CHARGE_ENABLE  PB10  // Pin 21 - Charger Enable (active low)
#define PIN_DCDC_ENABLE    PB12  // Pin 25 - DCDC Enable (active low)
#define PIN_PCS_ENABLE     PB14  // Pin 27 - PCS Enable
#define PIN_EVSE_SWITCH    PB15  // Pin 28 - EVSE Switch

// CAN Bus Interfaces
// Note: PIN_CANRXn/TXn numbers don't match peripheral numbers (historical naming)
// CAN0 pins -> CAN1 peripheral (IPC CAN to PCS)
#define PIN_CANRX0         PA11  // Pin 22 - CAN1 RX (AF9)
#define PIN_CANTX0         PA12  // Pin 23 - CAN1 TX (AF9)

// CAN1 pins -> CAN2 peripheral (CP_CAN)
#define PIN_CANRX1         PB5   // Pin 41 - CAN2 RX (AF9)
#define PIN_CANTX1         PB13  // Pin 26 - CAN2 TX (AF9)

// CAN2 pins -> CAN3 peripheral (OpenInverter)
#define PIN_CANRX2         PB3   // Pin 39 - CAN3 RX (AF11)
#define PIN_CANTX2         PA15  // Pin 38 - CAN3 TX (AF11)

// USART1 (Serial Debug/Terminal)
#define PIN_USART1_TX      PB6_ALT1   // Pin 42 - USART1 TX
#define PIN_USART1_RX      PB7_ALT0   // Pin 43 - USART1 RX

// Analog Inputs
#define PIN_PILOT_IN       PA6   // Pin 16 - EVSE Pilot PWM input (via resistor divider)
#define PIN_PILOT_SENSE    PA5   // Pin 15 - Pilot voltage sense

// General Purpose I/O
#define PIN_PB0            PB0   // Pin 18
#define PIN_PB1            PB1   // Pin 19
#define PIN_PB2            PB2   // Pin 20

// Debug Interface
#define PIN_SWDIO          PA13  // Pin 34 - SWD Data
#define PIN_SWCLK          PA14  // Pin 37 - SWD Clock

// Boot Pins
#define PIN_BOOT0          BOOT0 // External boot pin
#define PIN_BOOT1          PB4   // Pin 40 - Boot 1

// Status LED (typically PC13 on STM32F4 series)
#define PIN_LED            PC13  // Status LED

// Primary CAN Interface (for PCS communication)
#define CAN_INTERFACE      CAN1  // Using CAN1 peripheral on PA11/PA12

#endif // HWDEFS_H_INCLUDED
