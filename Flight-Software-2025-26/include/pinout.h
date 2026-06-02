#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

#define FC_RX           0   // RX1 <- Flight Controller TX (T6)
#define FC_TX           1   // TX1 -> Flight Controller RX (R6)

#define XBEE_TX           24  // TX6 -> XBee DIN
#define XBEE_RX           25  // RX6 <- XBee DOUT

#define CONTAINER 9   // SG90 Container Release
#define EGG 10  // SG90 Egg Release

#define TEMP    14

#endif // TEENSY_PINOUT_H

