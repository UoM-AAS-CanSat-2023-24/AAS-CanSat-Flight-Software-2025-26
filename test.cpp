
#include <cstdint>
 

// --  STATE MACHINE -- 

// will use (7,4) hamming code + overall parity checking = 8 bit codeword. this is to ensure single error correction, double error detection (SECDED)

enum class MissionState : uint8_t { // each of these states uses (7,4) hamming codes which were encoded by hand: THESE NUMBERS ARE NOT RANDOM PLEASE UNDER NO CIRCUMSTANCES CHANGE THEM
  LAUNCH_PAD_DISARMED = 0b00000000, // 0
  LAUNCH_PAD_ARMED = 0b11010010 , // 210
  ASCENT =  0b01010101, // 85
  APOGEE =  0b10000111, // 135
  DESCENT_PRE_PAYLOAD_RELEASE = 0b10011001, // 97
  DESCENT_PAYLOAD_RELEASE = 0b01001011, // 75
  DESCENT_PRE_PROBE_RELEASE = 0b0
  DESCENT_PROBE_RELEASE = 0b11001100, // 108
  DESCENT_POST_PROBE_RELEASE = 0b00011110, // 30
  GROUNDED = 0b11100001, // 225
  FAULT = 0b00110011 // 144 (this state is for when the system detects an error that it cannot correct, or if it detects an invalid state (should never happen))
};

uint8_t fsw_state = static_cast<uint8_t>(MissionState::LAUNCH_PAD_DISARMED); // inital state

int main(){

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


