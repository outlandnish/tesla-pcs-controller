#ifndef SERIAL_CONFIG_H
#define SERIAL_CONFIG_H

#include <Arduino.h>

// Define Serial1 using USART1 peripheral (PB6=TX, PB7=RX)
// This must be declared extern to avoid multiple definition errors
extern HardwareSerial Serial1;

// Redefine Serial to use Serial1
#define Serial Serial1

#endif // SERIAL_CONFIG_H
