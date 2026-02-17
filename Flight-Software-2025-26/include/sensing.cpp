#include <Arduino.h>
#include <Adafruit_INA260.h>

// ---SENSING DATA ---
// placeholder for sensor readings. the indices of this array correspond to the following sensors:
// [0]bmp-alt[1]bmp-temp[2]bmp-press[3]ina-voltage[4][5][6]GYRO-RPY[7][8][9]GYRO-ACC-RPY[10][11][12]MAG-RPY[13]AUTOGYRO-RR[14][15][16]GPSTimeHMS[17][18][19]GPS-alt-lat-long[20]GPS-sats[21]bus-current[22]bus-power
float sensor_readings[23] = {}; // initialize all sensor readings to 0.0

// Sensor objects
Adafruit_INA260 ina260 = Adafruit_INA260(); // initialize it with the default I2C address (0x40)

bool sensor_setup() {

   
   // Initialize I2C communication
    Wire.begin(); 

  // INA260 setup (current, voltage, power sensor)
  if (!ina260.begin()){
    Serial.println("Couldn't find INA260 chip");
    return false;
  } else {Serial.println("Found INA260 chip");} 
  
  
  Serial.println("All sensors initialized successfully.");
  
  
  
  return true; // all sensors setup successfully


}

bool read_ina260(float (&sensor_readings)[23])
{
  
  sensor_readings[3] = ina260.readBusVoltage(); // Bus voltage in mV
  sensor_readings[21] = ina260.readCurrent(); // Current in mA
  sensor_readings[22] = ina260.readPower(); // Power in mW
  
  return true;
}
