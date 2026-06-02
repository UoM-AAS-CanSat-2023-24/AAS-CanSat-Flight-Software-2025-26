/*
 * CanSat Ground Station Emulator
 * Target : Teensy 4.0 / 4.1
 * Build  : PlatformIO
 *
 * Wiring:
 *   XBee DOUT -> pin 25 (RX6)
 *   XBee DIN  -> pin 24 (TX6)
 *   USB       -> PC (Serial monitor @ 115200)
 *   Pin  9    -> CONTAINER servo signal (SG90)
 *   Pin 10    -> EGG servo signal       (SG90)
 *
 * Serial6 = XBee  @ 9600 baud
 * Serial  = USB   @ 115200 baud  (debug)
 *
 * Flight states (matching dashboard):
 *   LAUNCH_PAD -> ASCENT -> APOGEE -> DESCENT ->
 *   PROBE_RELEASE -> PAYLOAD_RELEASE -> LANDED
 *
 * Sim mode:
 *   - Listens for SIMP pressure commands from ground station
 *   - Converts pressure -> altitude via barometric formula
 *   - Transitions states based on altitude thresholds
 *   - PROBE_RELEASE   : fires CONTAINER servo (pin 9)  full PWM
 *   - PAYLOAD_RELEASE : fires EGG servo       (pin 10) full PWM
 *   - Continues transmitting telemetry back to ground station
 *
 * Normal mode:
 *   - Fake sensor data drives state progression automatically
 *   - ARM + CX,ON required to start
 *   - Servos still fire at correct state transitions
 */

#include <Arduino.h>
#include "pinout.h"

// ---------------------------------------------------------------------------
// XBee is on Serial6 (TX6=pin24, RX6=pin25)
// ---------------------------------------------------------------------------
#define XBEE Serial6

// ---------------------------------------------------------------------------
// Servo PWM - SG90: 500us=0deg, 2400us=180deg, use 2400 as "max/release"
// Teensy analogWrite is 0-255 but for proper servo use we write directly
// via the PWM period. However for a simple "fire and forget" release
// we just write the pin HIGH for the release pulse duration using
// a helper, or use Servo library if available.
// We use the Servo library (included in Teensyduino).
// ---------------------------------------------------------------------------
#include <Servo.h>

Servo containerServo;   // pin CONTAINER (9)
Servo eggServo;         // pin EGG       (10)

constexpr int SERVO_IDLE    =  10;   // degrees - stowed
constexpr int SERVO_RELEASE = 180;   // degrees - full release

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
constexpr uint16_t TEAM_ID         = 1059;
constexpr uint32_t TX_INTERVAL_MS  = 1000;

// Altitude thresholds for sim mode state transitions (metres)
constexpr float ALT_ASCENT_MIN      =  50.0f;   // LAUNCH_PAD  -> ASCENT
constexpr float ALT_APOGEE          = 500.0f;   // ASCENT      -> APOGEE
constexpr float ALT_DESCENT         = 490.0f;   // APOGEE      -> DESCENT (falling)
constexpr float ALT_PROBE_RELEASE   = 300.0f;   // DESCENT     -> PROBE_RELEASE
constexpr float ALT_PAYLOAD_RELEASE = 100.0f;   // PROBE_RELEASE-> PAYLOAD_RELEASE
constexpr float ALT_LANDED          =   5.0f;   // PAYLOAD_RELEASE->LANDED

// ---------------------------------------------------------------------------
// State enums
// ---------------------------------------------------------------------------
enum class FlightState : uint8_t {
    LAUNCH_PAD = 0,
    ASCENT,
    APOGEE,
    DESCENT,
    PROBE_RELEASE,
    PAYLOAD_RELEASE,
    LANDED
};

const char* const STATE_NAMES[] = {
    "LAUNCH_PAD",
    "ASCENT",
    "APOGEE",
    "DESCENT",
    "PROBE_RELEASE",
    "PAYLOAD_RELEASE",
    "LANDED"
};

// ---------------------------------------------------------------------------
// Global cansat state
// ---------------------------------------------------------------------------
struct CansatState {
    bool        cxOn           = false;
    bool        simMode        = false;
    bool        armed          = false;
    bool        simpReceived   = false;

    FlightState flightState    = FlightState::LAUNCH_PAD;
    bool        subArmed       = false;

    // Sensors
    float altitude     = 0.0f;
    float temperature  = 21.0f;
    float pressure     = 101300.0f;
    float voltage      = 4.2f;
    float current      = 0.5f;
    float gyroR        = 0.0f;
    float gyroP        = 0.0f;
    float gyroY        = 0.0f;
    float accelR       = 0.0f;
    float accelP       = 0.0f;
    float accelY       = 9.81f;

    // GPS
    float   gpsLat     = 51.18325f;
    float   gpsLon     = -1.82139f;
    float   gpsAlt     = 0.0f;
    uint8_t gpsSats    = 8;
    char    gpsTime[12]= "00:00:00";

    // Power
    uint8_t mainSOC    = 95;
    float   busPower   = 0.377f;

    // Mechanisms bitmask: bit0=mec1, bit1=mec2, bit2=mec3(CONTAINER), bit3=mec4(EGG)
    uint8_t activeMechs  = 0;
    uint8_t activeCamera = 0;

    // Timing
    uint32_t packetCount    = 0;
    uint32_t lastTxMs       = 0;
    uint32_t missionStartMs = 0;
    uint32_t stateEnteredMs = 0;
    bool     missionStarted = false;

    float altVelocity = 0.0f;

    char cmdEcho[32]   = "NONE";
    float simpPressure = 0.0f;

    // Track whether servos have already fired this state
    bool containerFired = false;
    bool eggFired       = false;
} g;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void processCommand(const String& line);
void sendTelemetry();
void buildPacket(char* buf, size_t len);
void updateSensors();
void advanceStateMachine();
void advanceSimStateMachine();
void transitionTo(FlightState next);
void fireServo(Servo& s, int pin, const char* name);
void missionTimeString(char* buf, size_t len);
float frand(float lo, float hi);
void dbg(const char* fmt, ...);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    XBEE.begin(9600);

    // Attach servos in idle position
    containerServo.attach(CONTAINER);
    eggServo.attach(EGG);
    containerServo.write(SERVO_IDLE);
    eggServo.write(SERVO_IDLE);

    delay(500);

    Serial.println("============================================");
    Serial.println(" CanSat Emulator - Teensy PIO");
    Serial.println(" XBee on Serial6 (TX6=24, RX6=25) @ 9600");
    Serial.println(" CONTAINER servo -> pin 9");
    Serial.println(" EGG servo       -> pin 10");
    Serial.println(" Send CMD,1059,CX,ON to start telemetry");
    Serial.println("============================================");

    randomSeed(analogRead(A0));
    g.stateEnteredMs = millis();
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    // --- Read from XBee ---
    if (XBEE.available()) {
        String line = XBEE.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            Serial.print("[RX XBee] ");
            Serial.println(line);
            processCommand(line);
        }
    }

    // --- Read from USB serial (bench testing without XBee) ---
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            Serial.print("[RX USB ] ");
            Serial.println(line);
            processCommand(line);
        }
    }

    // --- State machine ---
    if (g.cxOn) {
        if (g.simMode) {
            advanceSimStateMachine();
        } else if (g.armed) {
            advanceStateMachine();
        }
    }

    // --- Transmit telemetry at 1 Hz ---
    if (g.cxOn && (millis() - g.lastTxMs >= TX_INTERVAL_MS)) {
        g.lastTxMs = millis();
        updateSensors();
        sendTelemetry();
    }
}

// ---------------------------------------------------------------------------
// Command parser
// ---------------------------------------------------------------------------
void processCommand(const String& raw) {
    // Accept CMD,<TEAM_ID>,... or CMD,$,...
    String p1 = "CMD," + String(TEAM_ID) + ",";
    String p2 = "CMD,$,";
    String body;

    if (raw.startsWith(p1)) {
        body = raw.substring(p1.length());
    } else if (raw.startsWith(p2)) {
        body = raw.substring(p2.length());
    } else {
        Serial.print("[WARN] Bad prefix: ");
        Serial.println(raw);
        return;
    }

    // Tokenise body on comma
    String tok[8];
    uint8_t n = 0;
    int start = 0;
    for (int i = 0; i <= (int)body.length() && n < 8; i++) {
        if (i == (int)body.length() || body[i] == ',') {
            tok[n++] = body.substring(start, i);
            tok[n-1].trim();
            start = i + 1;
        }
    }

    String cmd = tok[0];
    cmd.toUpperCase();

    // CX ------------------------------------------------------------------
    if (cmd == "CX") {
        String val = tok[1]; val.toUpperCase();
        if (val == "ON") {
            g.cxOn = true;
            if (!g.missionStarted) {
                g.missionStartMs = millis();
                g.missionStarted = true;
            }
            strncpy(g.cmdEcho, "CXON", sizeof(g.cmdEcho));
            Serial.println("[CMD] CX ON - telemetry enabled");
        } else {
            g.cxOn = false;
            strncpy(g.cmdEcho, "CXOFF", sizeof(g.cmdEcho));
            Serial.println("[CMD] CX OFF - telemetry disabled");
        }
    }

    // ST ------------------------------------------------------------------
    else if (cmd == "ST") {
        String val = tok[1]; val.toUpperCase();
        Serial.print("[CMD] ST -> ");
        Serial.println(val);
        strncpy(g.cmdEcho, "ST", sizeof(g.cmdEcho));
    }

    // SIM -----------------------------------------------------------------
    else if (cmd == "SIM") {
        String val = tok[1]; val.toUpperCase();
        if (val == "ENABLE") {
            g.simMode = true;
            g.simpReceived = false;
            strncpy(g.cmdEcho, "SIMEN", sizeof(g.cmdEcho));
            Serial.println("[CMD] SIM ENABLE - waiting for SIMP commands");
        } else {
            g.simMode = false;
            g.simpReceived = false;
            strncpy(g.cmdEcho, "SIMDIS", sizeof(g.cmdEcho));
            Serial.println("[CMD] SIM DISABLE - back to free-run sensors");
        }
    }

    // SIMP ----------------------------------------------------------------
    else if (cmd == "SIMP") {
        if (!g.simMode) {
            Serial.println("[WARN] SIMP received but SIM not enabled - ignoring");
            return;
        }
        float p = tok[1].toFloat();
        g.simpPressure = p;
        g.simpReceived = true;

        // Barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
        const float P0 = 101325.0f;
        g.altitude = 44330.0f * (1.0f - powf(p / P0, 1.0f / 5.255f));
        g.gpsAlt   = g.altitude + frand(-2.0f, 2.0f);

        Serial.print("[SIMP] P=");
        Serial.print(p, 1);
        Serial.print(" Pa  ->  Alt=");
        Serial.print(g.altitude, 1);
        Serial.println(" m");

        strncpy(g.cmdEcho, "SIMP", sizeof(g.cmdEcho));
    }

    // CAL -----------------------------------------------------------------
    else if (cmd == "CAL") {
        g.altitude = 0.0f;
        g.gpsAlt   = 0.0f;
        strncpy(g.cmdEcho, "CAL", sizeof(g.cmdEcho));
        Serial.println("[CMD] CAL - altitude zeroed");
    }

    // ARM -----------------------------------------------------------------
    else if (cmd == "ARM") {
        String val = tok[1]; val.toUpperCase();
        if (val == "ON") {
            g.armed           = true;
            g.subArmed        = true;
            g.stateEnteredMs  = millis();
            strncpy(g.cmdEcho, "ARMON", sizeof(g.cmdEcho));
            Serial.println("[CMD] *** ARMED ***");
        } else {
            g.armed    = false;
            g.subArmed = false;
            strncpy(g.cmdEcho, "ARMOFF", sizeof(g.cmdEcho));
            Serial.println("[CMD] DISARMED");
        }
    }

    // MEC -----------------------------------------------------------------
    else if (cmd == "MEC") {
        int  code   = tok[1].toInt();
        String onof = tok[2]; onof.toUpperCase();
        bool  on    = (onof == "ON");

        // Map device code to bit index and pin
        int  bitIdx = -1;
        const char* mechName = "UNKNOWN";

        if      (code == 1000) { bitIdx = 0; mechName = "Mec1 (Payload release servo)"; }
        else if (code == 200)  { bitIdx = 1; mechName = "Mec2 (Port steering servo)"; }
        else if (code == 30)   { bitIdx = 2; mechName = "Mec3 (CONTAINER - pin 9)"; }
        else if (code == 4)    { bitIdx = 3; mechName = "Mec4 (EGG - pin 10)"; }

        if (bitIdx < 0) {
            Serial.print("[WARN] Unknown MEC code: ");
            Serial.println(code);
            return;
        }

        if (on) {
            g.activeMechs |= (1 << bitIdx);
            Serial.print("[MEC ON ] "); Serial.println(mechName);

            // Drive real servos on pins 9 and 10
            if (bitIdx == 2) {
                fireServo(containerServo, CONTAINER, mechName);
            } else if (bitIdx == 3) {
                fireServo(eggServo, EGG, mechName);
            }
        } else {
            g.activeMechs &= ~(1 << bitIdx);
            Serial.print("[MEC OFF] "); Serial.println(mechName);

            // Return servo to idle
            if (bitIdx == 2) { containerServo.write(SERVO_IDLE); }
            if (bitIdx == 3) { eggServo.write(SERVO_IDLE); }
        }

        strncpy(g.cmdEcho, "MEC", sizeof(g.cmdEcho));
    }

    else {
        Serial.print("[WARN] Unknown command: "); Serial.println(cmd);
    }
}

// ---------------------------------------------------------------------------
// Fire a servo to full release angle
// ---------------------------------------------------------------------------
void fireServo(Servo& s, int pin, const char* name) {
    s.write(SERVO_RELEASE);
    Serial.print("[SERVO] ");
    Serial.print(name);
    Serial.print(" -> pin ");
    Serial.print(pin);
    Serial.println(" -> RELEASE (180 deg)");
}

// ---------------------------------------------------------------------------
// Normal (free-run) state machine
// ---------------------------------------------------------------------------
void advanceStateMachine() {
    uint32_t elapsed = millis() - g.stateEnteredMs;

    switch (g.flightState) {
        case FlightState::LAUNCH_PAD:
            // Auto-launch 5 s after arming
            if (elapsed > 5000) transitionTo(FlightState::ASCENT);
            break;

        case FlightState::ASCENT:
            if (g.altitude > ALT_APOGEE || elapsed > 60000)
                transitionTo(FlightState::APOGEE);
            break;

        case FlightState::APOGEE:
            if (elapsed > 3000) transitionTo(FlightState::DESCENT);
            break;

        case FlightState::DESCENT:
            if (g.altitude < ALT_PROBE_RELEASE)
                transitionTo(FlightState::PROBE_RELEASE);
            break;

        case FlightState::PROBE_RELEASE:
            // Fire CONTAINER servo once on entry
            if (!g.containerFired) {
                fireServo(containerServo, CONTAINER, "CONTAINER (Mec3)");
                g.activeMechs |= (1 << 2);
                g.containerFired = true;
            }
            if (g.altitude < ALT_PAYLOAD_RELEASE)
                transitionTo(FlightState::PAYLOAD_RELEASE);
            break;

        case FlightState::PAYLOAD_RELEASE:
            // Fire EGG servo once on entry
            if (!g.eggFired) {
                fireServo(eggServo, EGG, "EGG (Mec4)");
                g.activeMechs |= (1 << 3);
                g.eggFired = true;
            }
            if (g.altitude <= ALT_LANDED)
                transitionTo(FlightState::LANDED);
            break;

        case FlightState::LANDED:
            break;
    }
}

// ---------------------------------------------------------------------------
// Sim mode state machine - driven by SIMP altitude values
// ---------------------------------------------------------------------------
void advanceSimStateMachine() {
    if (!g.simpReceived) return;   // wait for first SIMP before doing anything

    float alt = g.altitude;

    switch (g.flightState) {
        case FlightState::LAUNCH_PAD:
            if (alt > ALT_ASCENT_MIN) transitionTo(FlightState::ASCENT);
            break;

        case FlightState::ASCENT:
            if (alt >= ALT_APOGEE) transitionTo(FlightState::APOGEE);
            break;

        case FlightState::APOGEE:
            // Detect we have started descending
            if (alt < ALT_DESCENT) transitionTo(FlightState::DESCENT);
            break;

        case FlightState::DESCENT:
            if (alt < ALT_PROBE_RELEASE) transitionTo(FlightState::PROBE_RELEASE);
            break;

        case FlightState::PROBE_RELEASE:
            if (!g.containerFired) {
                fireServo(containerServo, CONTAINER, "CONTAINER (Mec3) - SIM auto");
                g.activeMechs   |= (1 << 2);
                g.containerFired = true;
                strncpy(g.cmdEcho, "MECAUTO3", sizeof(g.cmdEcho));
            }
            if (alt < ALT_PAYLOAD_RELEASE) transitionTo(FlightState::PAYLOAD_RELEASE);
            break;

        case FlightState::PAYLOAD_RELEASE:
            if (!g.eggFired) {
                fireServo(eggServo, EGG, "EGG (Mec4) - SIM auto");
                g.activeMechs |= (1 << 3);
                g.eggFired     = true;
                strncpy(g.cmdEcho, "MECAUTO4", sizeof(g.cmdEcho));
            }
            if (alt <= ALT_LANDED) transitionTo(FlightState::LANDED);
            break;

        case FlightState::LANDED:
            break;
    }
}

// ---------------------------------------------------------------------------
// State transition helper
// ---------------------------------------------------------------------------
void transitionTo(FlightState next) {
    Serial.print("[STATE] ");
    Serial.print(STATE_NAMES[(uint8_t)g.flightState]);
    Serial.print(" -> ");
    Serial.println(STATE_NAMES[(uint8_t)next]);

    g.flightState    = next;
    g.stateEnteredMs = millis();

    // Reset servo-fired flags on fresh transitions
    if (next == FlightState::PROBE_RELEASE)   g.containerFired = false;
    if (next == FlightState::PAYLOAD_RELEASE) g.eggFired       = false;
}

// ---------------------------------------------------------------------------
// Telemetry transmit
// ---------------------------------------------------------------------------
void sendTelemetry() {
    char buf[512];
    buildPacket(buf, sizeof(buf));
    XBEE.println(buf);
    Serial.print("[TX GS ] ");
    Serial.println(buf);
}

void buildPacket(char* buf, size_t len) {
    char mt[12];
    missionTimeString(mt, sizeof(mt));

    snprintf(buf, len,
        "%u, %s, %lu, %c, %s, %.1f, %.1f, %.1f, %.2f, %.2f, "
        "%.1f, %.1f, %.1f, %.2f, %.2f, %.2f, "
        "%s, %.1f, %.4f, %.4f, %u, "
        "%s, %s, %u, %.3f, %u, %u, EMULATOR",
        TEAM_ID,
        mt,
        ++g.packetCount,
        g.simMode ? 'S' : 'F',
        STATE_NAMES[(uint8_t)g.flightState],
        g.altitude,
        g.temperature,
        g.pressure,
        g.voltage,
        g.current,
        g.gyroR, g.gyroP, g.gyroY,
        g.accelR, g.accelP, g.accelY,
        mt,                           // GPS time = mission time for emulator
        g.gpsAlt,
        g.gpsLat,
        g.gpsLon,
        (unsigned)g.gpsSats,
        g.cmdEcho,
        g.subArmed ? "ARMED" : "DISARMED",
        (unsigned)g.mainSOC,
        g.busPower,
        (unsigned)g.activeMechs,
        (unsigned)g.activeCamera
    );
}

// ---------------------------------------------------------------------------
// Sensor simulation
// ---------------------------------------------------------------------------
void updateSensors() {
    // In sim mode altitude is already set by SIMP; only noise up other values
    if (!g.simMode) {
        switch (g.flightState) {
            case FlightState::LAUNCH_PAD:
                g.altitude    = frand(-0.5f, 0.5f);
                g.altVelocity = 0.0f;
                break;

            case FlightState::ASCENT:
                g.altVelocity  = frand(8.0f, 12.0f);
                g.altitude    += g.altVelocity * (TX_INTERVAL_MS / 1000.0f);
                g.gpsAlt       = g.altitude + frand(-3.0f, 3.0f);
                g.gpsLat      += frand(-0.00005f, 0.00005f);
                g.gpsLon      += frand(-0.00005f, 0.00005f);
                g.gyroR        = frand(-20.0f, 20.0f);
                g.gyroP        = frand(-20.0f, 20.0f);
                g.accelY       = frand(9.0f, 11.0f);
                break;

            case FlightState::APOGEE:
                g.altitude    += frand(-0.5f, 0.5f);
                g.accelY       = frand(0.0f, 2.0f);
                break;

            case FlightState::DESCENT:
            case FlightState::PROBE_RELEASE:
            case FlightState::PAYLOAD_RELEASE:
                g.altVelocity  = frand(-4.0f, -2.0f);
                g.altitude    += g.altVelocity * (TX_INTERVAL_MS / 1000.0f);
                if (g.altitude < 0.0f) g.altitude = 0.0f;
                g.gpsAlt       = g.altitude + frand(-2.0f, 2.0f);
                g.gpsLat      += frand(-0.0001f, 0.0001f);
                g.gpsLon      += frand(-0.0001f, 0.0001f);
                g.gyroR        = frand(-50.0f, 50.0f);
                g.gyroP        = frand(-50.0f, 50.0f);
                g.accelY       = frand(7.0f, 10.0f);
                break;

            case FlightState::LANDED:
                g.altVelocity = 0.0f;
                g.gyroR = g.gyroP = g.gyroY = 0.0f;
                g.accelR = g.accelP = 0.0f;
                g.accelY = 9.81f;
                break;
        }
    }

    // Universal sensor drift regardless of mode
    g.temperature += frand(-0.1f, 0.1f);
    const float P0 = 101325.0f;
    g.pressure     = P0 * powf(1.0f - (g.altitude / 44330.0f), 5.255f) + frand(-5.0f, 5.0f);
    g.voltage     += frand(-0.01f, 0.01f);
    g.voltage      = constrain(g.voltage, 3.5f, 4.2f);
    g.current     += frand(-0.05f, 0.05f);
    g.current      = constrain(g.current, 0.1f, 3.0f);
    g.busPower     = g.voltage * g.current;

    missionTimeString(g.gpsTime, sizeof(g.gpsTime));
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------
void missionTimeString(char* buf, size_t len) {
    if (!g.missionStarted) { strncpy(buf, "00:00:00", len); return; }
    uint32_t s  = (millis() - g.missionStartMs) / 1000;
    uint32_t hh = s / 3600;
    uint32_t mm = (s % 3600) / 60;
    uint32_t ss = s % 60;
    snprintf(buf, len, "%02lu:%02lu:%02lu", hh, mm, ss);
}

float frand(float lo, float hi) {
    return lo + (float)random(0, 10000) / 10000.0f * (hi - lo);
}