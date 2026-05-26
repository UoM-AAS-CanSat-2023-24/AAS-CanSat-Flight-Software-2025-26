#include <Arduino.h>

#define XBEE_BAUD 115200
#define LED_PIN 13

uint32_t last_rx_time = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Serial6.begin(XBEE_BAUD);

  delay(2000);

  Serial.println("=== CLEAN XBee RX TEST ===");
  Serial.println("Waiting for packets...");
}

void loop() {

  // --- CHECK FOR INCOMING DATA ---
  if (Serial6.available()) {

    String msg = Serial6.readStringUntil('\n');

    Serial.print("[RX] ");
    Serial.println(msg);

    // ✔ HARD INDICATOR: packet received
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);

    last_rx_time = millis();
  }

  // --- LINK DEAD INDICATOR (optional but useful) ---
  // If no packets for 3 seconds → slow blink
  if (millis() - last_rx_time > 3000) {

    // slow blink = "no RF activity"
    digitalWrite(LED_PIN, (millis() / 400) % 2);
  }
}