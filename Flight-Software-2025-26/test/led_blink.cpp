#include <Arduino.h>

// LED pin definition for Teensy 4.1
#define LED_PIN 13

void setup() {
  // Initialize the LED pin as output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Turn LED on
  digitalWrite(LED_PIN, HIGH);
  delay(1000);  // Wait 1 second
  
  // Turn LED off
  digitalWrite(LED_PIN, LOW);
  delay(1000);  // Wait 1 second
  
  // Loop repeats indefinitely
}
