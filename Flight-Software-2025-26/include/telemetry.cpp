#include <Arduino.h>
#include "sensing.cpp" // needed for gps based stuuf

bool telemetry_setup(){
    // Initialize Serial communication for telemetry
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    return true; // telemetry setup successful
}

