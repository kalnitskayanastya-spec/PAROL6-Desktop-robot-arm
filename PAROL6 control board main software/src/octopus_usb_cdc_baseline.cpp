#include <Arduino.h>

#define Serial SerialUSB

static const unsigned long BAUD_RATE = 115200;
static const unsigned long PRINT_INTERVAL_MS = 1000;
static const uint32_t LED_PIN = PB10;

static unsigned long lastPrintMs = 0;
static unsigned long lastBlinkMs = 0;
static bool ledState = false;

static void printBaselineStatus() {
  Serial.println(F("--- OCTOPUS USB CDC BASELINE ---"));
  Serial.print(F("Build: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  Serial.print(F("millis="));
  Serial.println(millis());
  Serial.flush();
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  printBaselineStatus();
}

void loop() {
  unsigned long now = millis();

  if ((now - lastBlinkMs) >= 500) {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  if ((now - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printBaselineStatus();
  }
}
