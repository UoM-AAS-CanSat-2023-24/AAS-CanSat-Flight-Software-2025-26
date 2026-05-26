#include <Arduino.h>

#define XBEE_BAUD 115200
#define LED_PIN 13

uint32_t packet_count = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Serial6.begin(XBEE_BAUD);

  delay(2000);

  Serial.println("=== CLEAN XBee TX TEST ===");
}

void loop() {

  char packet[128];

  snprintf(packet, sizeof(packet),
    "TEST_TX,%lu,%lu\n",
    millis(),
    packet_count
  );

  // Send packet
  size_t written = Serial6.print(packet);

  // Debug
  Serial.print("[TX] ");
  Serial.print(packet);

  Serial.print("[UART written] ");
  Serial.println(written);

  // ✔ LED PULSE = physical send indicator
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);

  packet_count++;

  delay(1000);
}