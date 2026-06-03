/*
 * CanSat Ground Station Emulator
 * Target: Teensy (any model with 2x hardware serial)
 * 
 * Wiring:
 *   XBee TX  -> Teensy Serial1 RX (pin 0)
 *   XBee RX  -> Teensy Serial1 TX (pin 1)
 *   USB      -> PC (Serial monitor + ground station dashboard)
 *
 * Serial1 = XBee (ground station comms) at 9600 baud
 * Serial  = USB debug terminal at 115200 baud
 *
 * Behaviour:
 *   - Responds to CMD strings from ground station
 *   - Transmits fake telemetry at ~1 Hz when CX is ON
 *   - Prints all activity to USB serial for visibility
 *   - Simulates state machine: LAUNCH_WAIT -> ASCENT -> APOGEE -> DESCENT -> LANDED
 *   - Responds to SIM,ENABLE/DISABLE and SIMP pressure commands
 *   - Responds to ARM, CAL, MEC, ST commands
 *   - Intentionally drifts sensor values to look realistic
 */

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define TEAM_ID         1059
#define XBEE_BAUD       9600
#define USB_BAUD        115200
#define TX_INTERVAL_MS  1000    // telemetry transmit interval

// XBee on Serial1
#define XBEE Serial1

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

enum FlightState {
  STATE_LAUNCH_WAIT,
  STATE_ASCENT,
  STATE_APOGEE,
  STATE_DESCENT,
  STATE_LANDED
};

const char* stateNames[] = {
  "LAUNCH_WAIT",
  "ASCENT",
  "APOGEE",
  "DESCENT",
  "LANDED"
};

enum SubState {
  SUB_ARMED,
  SUB_DISARMED
};

const char* subStateNames[] = {
  "ARMED",
  "DISARMED"
};

// ---------------------------------------------------------------------------
// Global CanSat state
// ---------------------------------------------------------------------------

bool        cxOn          = false;    // telemetry TX enabled
bool        simMode       = false;    // simulation mode active
bool        armed         = false;    // armed state
FlightState flightState   = STATE_LAUNCH_WAIT;
SubState    subState      = SUB_DISARMED;

// Simulated sensor values
float altitude      = 0.0;
float temperature   = 21.0;
float pressure      = 101300.0;
float voltage       = 4.2;
float current       = 0.5;
float gyroR         = 0.0;
float gyroP         = 0.0;
float gyroY         = 0.0;
float accelR        = 0.0;
float accelP        = 0.0;
float accelY        = 9.81;

// GPS
float gpsLat        = 51.18325;
float gpsLon        = -1.82139;
float gpsAlt        = 0.0;
int   gpsSats       = 8;
char  gpsTime[12]   = "00:00:00";

// Power
int   mainSOC       = 95;
float busPower      = 0.377;

// Mechanisms (bitmask: bit0=mec1, bit1=mec2, bit2=mec3, bit3=mec4)
int   activeMechs   = 0;
int   activeCamera  = 0;

// Mission time
unsigned long missionStartMs = 0;
bool missionStarted = false;

// Packet counter
int packetCount = 0;

// TX timing
unsigned long lastTxMs = 0;

// Last received command echo
char cmdEcho[32] = "NONE";

// Simulated pressure from SIMP commands
float simpPressure = 0.0;
bool  simpReceived = false;

// State timing for auto-progression
unsigned long stateEnteredMs = 0;

// Altitude drift direction for ascent/descent sim
float altVelocity = 0.0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

void processCommand(const String& cmd);
void sendTelemetry();
void updateSimulatedSensors();
void advanceStateMachine();
String buildPacket();
void printStatus(const String& msg);
void missionTimeString(char* buf);
float randomFloat(float lo, float hi);
void applyMech(int device, bool on);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(USB_BAUD);
  XBEE.begin(XBEE_BAUD);

  delay(500);

  Serial.println("============================================");
  Serial.println(" CanSat Emulator - Teensy");
  Serial.println(" XBee on Serial1 @ 9600");
  Serial.println(" Waiting for CX,ON from ground station...");
  Serial.println("============================================");

  // Seed random with floating analog read
  randomSeed(analogRead(A0));

  stateEnteredMs = millis();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

void loop() {
  // --- Read commands from XBee ---
  if (XBEE.available()) {
    String line = XBEE.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("[RX from GS] ");
      Serial.println(line);
      processCommand(line);
    }
  }

  // --- Also accept commands from USB serial (for testing without XBee) ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("[RX from USB] ");
      Serial.println(line);
      processCommand(line);
    }
  }

  // --- Auto-advance state machine if armed and CX on ---
  if (armed && cxOn) {
    advanceStateMachine();
  }

  // --- Transmit telemetry at 1 Hz if CX is on ---
  if (cxOn && (millis() - lastTxMs >= TX_INTERVAL_MS)) {
    lastTxMs = millis();
    updateSimulatedSensors();
    sendTelemetry();
  }
}

// ---------------------------------------------------------------------------
// Command parser
// ---------------------------------------------------------------------------

void processCommand(const String& raw) {
  // Validate prefix: CMD,<TEAM_ID>,...
  // Accept both correct team ID and broadcast ($)
  String prefix1 = "CMD," + String(TEAM_ID) + ",";
  String prefix2 = "CMD,$,";

  String body = "";
  if (raw.startsWith(prefix1)) {
    body = raw.substring(prefix1.length());
  } else if (raw.startsWith(prefix2)) {
    body = raw.substring(prefix2.length());
  } else {
    Serial.print("[WARN] Unknown command format: ");
    Serial.println(raw);
    return;
  }

  // Split body by comma into tokens
  String tokens[8];
  int    tokenCount = 0;
  int    start = 0;
  for (int i = 0; i <= body.length() && tokenCount < 8; i++) {
    if (i == body.length() || body[i] == ',') {
      tokens[tokenCount++] = body.substring(start, i);
      start = i + 1;
    }
  }

  String cmd = tokens[0];
  cmd.toUpperCase();

  // --- CX: telemetry on/off ---
  if (cmd == "CX") {
    String state = tokens[1];
    state.toUpperCase();
    if (state == "ON") {
      cxOn = true;
      if (!missionStarted) {
        missionStartMs = millis();
        missionStarted = true;
      }
      strncpy(cmdEcho, "CXON", sizeof(cmdEcho));
      printStatus("CX ON - telemetry transmit enabled");
    } else if (state == "OFF") {
      cxOn = false;
      strncpy(cmdEcho, "CXOFF", sizeof(cmdEcho));
      printStatus("CX OFF - telemetry transmit disabled");
    }
  }

  // --- ST: set mission time ---
  else if (cmd == "ST") {
    String timeVal = tokens[1];
    timeVal.toUpperCase();
    if (timeVal == "GPS") {
      // Sync from GPS - just echo current GPS time
      strncpy(gpsTime, "13:14:00", sizeof(gpsTime));
      printStatus("ST GPS - mission time synced to GPS: " + String(gpsTime));
    } else {
      // Set explicit time - store as mission start reference
      // In real CanSat this sets the RTC
      printStatus("ST set to: " + timeVal);
    }
    strncpy(cmdEcho, "ST", sizeof(cmdEcho));
  }

  // --- SIM: simulation mode enable/disable ---
  else if (cmd == "SIM") {
    String mode = tokens[1];
    mode.toUpperCase();
    if (mode == "ENABLE") {
      simMode = true;
      strncpy(cmdEcho, "SIMEN", sizeof(cmdEcho));
      printStatus("SIM MODE ENABLED - waiting for SIMP pressure commands");
    } else if (mode == "DISABLE") {
      simMode = false;
      simpReceived = false;
      strncpy(cmdEcho, "SIMDIS", sizeof(cmdEcho));
      printStatus("SIM MODE DISABLED - using real sensors");
    }
  }

  // --- SIMP: simulated pressure data ---
  else if (cmd == "SIMP") {
    if (!simMode) {
      Serial.println("[WARN] SIMP received but SIM mode not enabled - ignoring");
      return;
    }
    float p = tokens[1].toFloat();
    simpPressure = p;
    simpReceived = true;

    // Convert simulated pressure to altitude using barometric formula
    // h = 44330 * (1 - (P/P0)^(1/5.255))
    float p0 = 101325.0;
    altitude = 44330.0 * (1.0 - pow(p / p0, 1.0 / 5.255));
    gpsAlt   = altitude + randomFloat(-2.0, 2.0);

    Serial.print("[SIMP] Pressure: ");
    Serial.print(p, 1);
    Serial.print(" Pa -> Altitude: ");
    Serial.print(altitude, 1);
    Serial.println(" m");
    strncpy(cmdEcho, "SIMP", sizeof(cmdEcho));
  }

  // --- CAL: calibrate altitude to zero ---
  else if (cmd == "CAL") {
    altitude = 0.0;
    gpsAlt   = 0.0;
    printStatus("CAL - altitude calibrated to zero");
    strncpy(cmdEcho, "CAL", sizeof(cmdEcho));
  }

  // --- ARM: arm/disarm ---
  else if (cmd == "ARM") {
    String state = tokens[1];
    state.toUpperCase();
    if (state == "ON") {
      armed    = true;
      subState = SUB_ARMED;
      stateEnteredMs = millis();
      printStatus("*** ARMED ***");
      strncpy(cmdEcho, "ARMON", sizeof(cmdEcho));
    } else if (state == "OFF") {
      armed    = false;
      subState = SUB_DISARMED;
      printStatus("DISARMED");
      strncpy(cmdEcho, "ARMOFF", sizeof(cmdEcho));
    }
  }

  // --- MEC: mechanism control ---
  else if (cmd == "MEC") {
    String deviceCode = tokens[1];
    String onOff      = tokens[2];
    onOff.toUpperCase();
    bool activate = (onOff == "ON");
    applyMech(deviceCode.toInt(), activate);
    strncpy(cmdEcho, "MEC", sizeof(cmdEcho));
  }

  else {
    Serial.print("[WARN] Unrecognised command: ");
    Serial.println(cmd);
  }
}

// ---------------------------------------------------------------------------
// Mechanism handler
// ---------------------------------------------------------------------------

void applyMech(int deviceCode, bool on) {
  // Device codes from spec:
  // 1000 = Servo 1 - Payload release
  // 0200 = Servo 2 - Port steering
  //  030 = Servo 3 - Starboard steering
  //    4 = Servo 4 - Egg release

  int mechIndex = -1;
  const char* mechName = "UNKNOWN";

  if      (deviceCode == 1000) { mechIndex = 0; mechName = "Servo1 (Payload release)"; }
  else if (deviceCode == 200)  { mechIndex = 1; mechName = "Servo2 (Port steering)"; }
  else if (deviceCode == 30)   { mechIndex = 2; mechName = "Servo3 (Starboard steering)"; }
  else if (deviceCode == 4)    { mechIndex = 3; mechName = "Servo4 (Egg release)"; }

  if (mechIndex < 0) {
    Serial.print("[WARN] Unknown mechanism code: ");
    Serial.println(deviceCode);
    return;
  }

  if (on) {
    activeMechs |= (1 << mechIndex);
    Serial.print("[MECH ON ] ");
  } else {
    activeMechs &= ~(1 << mechIndex);
    Serial.print("[MECH OFF] ");
  }
  Serial.print(mechName);
  Serial.print("  activeMechs bitmask: ");
  Serial.println(activeMechs);
}

// ---------------------------------------------------------------------------
// Telemetry builder and sender
// ---------------------------------------------------------------------------

void sendTelemetry() {
  String packet = buildPacket();

  // Send over XBee to ground station
  XBEE.println(packet);

  // Echo to USB so you can watch in serial monitor
  Serial.print("[TX to GS] ");
  Serial.println(packet);
}

String buildPacket() {
  char missionTime[12];
  missionTimeString(missionTime);

  char buf[512];
  snprintf(buf, sizeof(buf),
    "%d, %s, %d, %c, %s, %.1f, %.1f, %.1f, %.2f, %.2f, "
    "%.1f, %.1f, %.1f, %.2f, %.2f, %.2f, "
    "%s, %.1f, %.4f, %.4f, %d, "
    "%s, %s, %d, %.3f, %d, %d, EMULATOR",
    TEAM_ID,
    missionTime,
    ++packetCount,
    simMode ? 'S' : 'F',          // S = simulation, F = flight
    stateNames[flightState],
    altitude,
    temperature,
    pressure,
    voltage,
    current,
    gyroR, gyroP, gyroY,
    accelR, accelP, accelY,
    gpsTime,
    gpsAlt,
    gpsLat,
    gpsLon,
    gpsSats,
    cmdEcho,
    subStateNames[subState],
    mainSOC,
    busPower,
    activeMechs,
    activeCamera
  );

  return String(buf);
}

// ---------------------------------------------------------------------------
// Simulated sensor updates
// ---------------------------------------------------------------------------

void updateSimulatedSensors() {
  if (simMode && simpReceived) {
    // In sim mode altitude is driven by SIMP commands - already set in processCommand
    // Just add small noise to other sensors
  } else {
    // Free-running simulation based on state machine
    switch (flightState) {
      case STATE_LAUNCH_WAIT:
        altitude    = randomFloat(-0.5, 0.5);
        altVelocity = 0.0;
        break;

      case STATE_ASCENT:
        altVelocity = randomFloat(8.0, 12.0);   // ~10 m/s ascent
        altitude   += altVelocity * (TX_INTERVAL_MS / 1000.0);
        gpsAlt      = altitude + randomFloat(-3.0, 3.0);
        gpsLat     += randomFloat(-0.00005, 0.00005);
        gpsLon     += randomFloat(-0.00005, 0.00005);
        gyroR       = randomFloat(-20.0, 20.0);
        gyroP       = randomFloat(-20.0, 20.0);
        gyroY       = randomFloat(-10.0, 10.0);
        accelR      = randomFloat(-1.0, 1.0);
        accelP      = randomFloat(-1.0, 1.0);
        accelY      = randomFloat(9.0, 11.0);
        break;

      case STATE_APOGEE:
        altitude   += randomFloat(-0.5, 0.5);
        gyroR       = randomFloat(-5.0, 5.0);
        accelY      = randomFloat(0.0, 2.0);
        break;

      case STATE_DESCENT:
        altVelocity = randomFloat(-4.0, -2.0);  // ~3 m/s descent
        altitude   += altVelocity * (TX_INTERVAL_MS / 1000.0);
        if (altitude < 0.0) altitude = 0.0;
        gpsAlt      = altitude + randomFloat(-2.0, 2.0);
        gpsLat     += randomFloat(-0.0001, 0.0001);
        gpsLon     += randomFloat(-0.0001, 0.0001);
        gyroR       = randomFloat(-50.0, 50.0);
        gyroP       = randomFloat(-50.0, 50.0);
        accelR      = randomFloat(-2.0, 2.0);
        accelP      = randomFloat(-2.0, 2.0);
        accelY      = randomFloat(7.0, 10.0);
        break;

      case STATE_LANDED:
        altVelocity = 0.0;
        gyroR = gyroP = gyroY = 0.0;
        accelR = accelP = 0.0;
        accelY = 9.81;
        break;
    }
  }

  // Sensors always drift slightly
  temperature += randomFloat(-0.1, 0.1);
  pressure     = 101325.0 * pow(1.0 - (altitude / 44330.0), 5.255) + randomFloat(-5.0, 5.0);
  voltage     += randomFloat(-0.01, 0.01);
  voltage      = constrain(voltage, 3.5, 4.2);
  current     += randomFloat(-0.05, 0.05);
  current      = constrain(current, 0.1, 3.0);
  mainSOC      = constrain(mainSOC, 0, 100);
  busPower     = voltage * current;

  // Update GPS time to match mission time
  missionTimeString(gpsTime);
}

// ---------------------------------------------------------------------------
// State machine auto-progression
// ---------------------------------------------------------------------------

void advanceStateMachine() {
  unsigned long elapsed = millis() - stateEnteredMs;

  switch (flightState) {
    case STATE_LAUNCH_WAIT:
      // Auto-launch 5 seconds after arming (just for testing)
      if (elapsed > 5000) {
        flightState    = STATE_ASCENT;
        stateEnteredMs = millis();
        printStatus("STATE -> ASCENT");
      }
      break;

    case STATE_ASCENT:
      // Transition to apogee when altitude > 500 m (or after 60s in sim)
      if (altitude > 500.0 || elapsed > 60000) {
        flightState    = STATE_APOGEE;
        stateEnteredMs = millis();
        printStatus("STATE -> APOGEE (peak altitude: " + String(altitude, 1) + " m)");
      }
      break;

    case STATE_APOGEE:
      // Short apogee window - 3 seconds
      if (elapsed > 3000) {
        flightState    = STATE_DESCENT;
        stateEnteredMs = millis();
        printStatus("STATE -> DESCENT");
      }
      break;

    case STATE_DESCENT:
      // Land when altitude drops to near zero
      if (altitude <= 5.0) {
        flightState    = STATE_LANDED;
        stateEnteredMs = millis();
        altitude       = 0.0;
        printStatus("STATE -> LANDED");
      }
      break;

    case STATE_LANDED:
      // Stay landed - nothing to advance to
      break;
  }
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

void missionTimeString(char* buf) {
  if (!missionStarted) {
    strncpy(buf, "00:00:00", 12);
    return;
  }
  unsigned long elapsed = (millis() - missionStartMs) / 1000;
  int hh = elapsed / 3600;
  int mm = (elapsed % 3600) / 60;
  int ss = elapsed % 60;
  snprintf(buf, 12, "%02d:%02d:%02d", hh, mm, ss);
}

void printStatus(const String& msg) {
  Serial.print("[STATUS] ");
  Serial.println(msg);
}

float randomFloat(float lo, float hi) {
  return lo + (float)random(0, 10000) / 10000.0 * (hi - lo);
}
