#include <Arduino.h>
#include "sensing.cpp"

// Constants
const int TELEMETRY_TIMER_MS = 1000;

// ---SENSING DATA ---
// placeholder for sensor readings. the indices of this array correspond to the following sensors:
// [0]bmp-alt[1]bmp-temp[2]bmp-press[3]ina-voltage[4][5][6]GYRO-RPY[7][8][9]GYRO-ACC-RPY[10][11][12]MAG-RPY[13]AUTOGYRO-RR[14][15][16]GPSTimeHMS[17][18][19]GPS-alt-lat-long[20]GPS-sats[21]bus-current[22]bus-power

float sensor_readings[23] = {}; // initialize all sensor readings to 0.0
float altitude_offset = 0;
float cansat_tilt[3] = {0,0,0};

// --  STATE MACHINE -- 

// will use (7,4) hamming code + overall parity checking = 8 bit codeword. this is to ensure single error correction, double error detection (SECDED)

enum class MissionState : uint8_t { // each of these states uses (7,4) hamming codes which were encoded by hand: THESE NUMBERS ARE NOT RANDOM PLEASE UNDER NO CIRCUMSTANCES CHANGE THEM
  LAUNCH_PAD_DISARMED = 0b00000000, //
  LAUNCH_PAD_ARMED = 0b11010010, // 
  ASCENT =  0b01010101, // 
  APOGEE =  0b10000111, // 
  DESCENT_PRE_PAYLOAD_RELEASE = 0b10011001, // 
  DESCENT_PAYLOAD_RELEASE = 0b01001011, // 
  DESCENT_PRE_PROBE_RELEASE = 0b11001100,
  DESCENT_PROBE_RELEASE = 0b00011110, //
  DESCENT_POST_PROBE_RELEASE = 0b11100001, // 
  GROUNDED = 0b00110011, // 
  FAULT = 0b10110100 //  (this state is for when the system detects an error that it cannot correct, or if it detects an invalid state (should never happen))
};

uint8_t fsw_state = static_cast<uint8_t>(MissionState::LAUNCH_PAD_DISARMED); // inital state



void setup() {   //  setup code here, to run once:
  
  // Initialize Serial communication for debugging
  Serial.begin(115200); 
  while (!Serial) { delay(10); }

  int desc_count = 0;
  bool descending = 0;
  float apogee = 0;

  long telemetry_time= millis(); // current time in milliseconds for telemetry timing
  long last_telemetry_time = 0;
  long last_sd_time = 0;
  float last_altitude; 

    // Sensor setup
  while(!sensor_setup()); // make sure sensors are definitely set up


}


void loop() {

  switch(fsw_state){ 

    case static_cast<uint8_t>(MissionState::LAUNCH_PAD_DISARMED):
      // code for launch pad disarmed state
      
      break;
    
    case static_cast<uint8_t>(MissionState::LAUNCH_PAD_ARMED):
     // code for launch pad armed state
      break; 
   
    case static_cast<uint8_t>(MissionState::ASCENT):
      // code for ascent state      

      break;
    
    case static_cast<uint8_t>(MissionState::APOGEE):
      // code for apogee state
      break;    
   
    case static_cast<uint8_t>(MissionState::DESCENT_PRE_PAYLOAD_RELEASE):
      // code for descent pre payload release state 
      break;
    
    case static_cast<uint8_t>(MissionState::DESCENT_PAYLOAD_RELEASE):
      // code for descent payload release state
      break;
   
    case static_cast<uint8_t>(MissionState::DESCENT_PRE_PROBE_RELEASE):
      // code for descent pre probe release state
      break;  
    
    case static_cast<uint8_t>(MissionState::DESCENT_PROBE_RELEASE):
      // code for descent probe release state
      break;
    
    case static_cast<uint8_t>(MissionState::DESCENT_POST_PROBE_RELEASE):
      // code for descent post probe release state
      break;

    case static_cast<uint8_t>(MissionState::GROUNDED):
      // code for grounded state
      break;
    
    case static_cast<uint8_t>(MissionState::FAULT):
      // code for fault state
      break;

    default:
        // code for invalid state (should never happen)
        break;      
 

  }

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}