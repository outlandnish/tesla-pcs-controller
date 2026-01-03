#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "serial_config.h"  // Uses existing Serial->Serial1 mapping

// Define which serial port to use for debug output
// This allows all code to use DEBUG_SERIAL for consistency
#define DEBUG_SERIAL Serial  // Serial is already redefined as Serial1 in serial_config.h

#endif // DEBUG_SERIAL_H
