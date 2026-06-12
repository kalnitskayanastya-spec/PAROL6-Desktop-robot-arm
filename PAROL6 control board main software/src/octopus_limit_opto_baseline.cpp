#include <Arduino.h>

#define Serial SerialUSB

static const unsigned long BAUD_RATE = 115200;
static const unsigned long ALIVE_INTERVAL_MS = 1000;
static const unsigned long STREAM_INTERVAL_MS = 250;
static const int DIAG_COUNT = 6;

struct DiagInput {
  const char *name;
  const char *pinName;
  uint32_t pin;
};

// Octopus Stop0..Stop5 / DIAG0..DIAG5 pins, matching the existing Octopus map.
// Hardware validated through optocouplers on DIAG0..DIAG5.
// Idle raw=111111; a triggered NPN NO sensor drives its DIAG input LOW.
// Expected single-trigger masks: DIAG0=011111, DIAG1=101111, DIAG2=110111,
// DIAG3=111011, DIAG4=111101, DIAG5=111110.
static const DiagInput diagInputs[DIAG_COUNT] = {
    {"DIAG0", "PG6", PG6},
    {"DIAG1", "PG9", PG9},
    {"DIAG2", "PG10", PG10},
    {"DIAG3", "PG11", PG11},
    {"DIAG4", "PG12", PG12},
    {"DIAG5", "PG13", PG13},
};

static bool usePullup = false;
static bool streamEnabled = false;
static unsigned long lastAliveMs = 0;
static unsigned long lastStreamMs = 0;

static const char *levelName(bool high) {
  return high ? "HIGH" : "LOW";
}

static char levelBit(bool high) {
  return high ? '1' : '0';
}

static bool readDiag(int index) {
  return digitalRead(diagInputs[index].pin) == HIGH;
}

static void applyPinModes() {
  for (int i = 0; i < DIAG_COUNT; ++i) {
    pinMode(diagInputs[i].pin, usePullup ? INPUT_PULLUP : INPUT);
  }
}

static void printRawBits(const char *prefix) {
  Serial.print(prefix);
  for (int i = 0; i < DIAG_COUNT; ++i) {
    Serial.print(levelBit(readDiag(i)));
  }
  Serial.println();
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("help"));
  Serial.println(F("safe"));
  Serial.println(F("raw"));
  Serial.println(F("status"));
  Serial.println(F("stream on"));
  Serial.println(F("stream off"));
  Serial.println(F("pullup on"));
  Serial.println(F("pullup off"));
}

static void printSafety() {
  Serial.println(F("SAFETY:"));
  Serial.println(F("- USB CDC baseline plus DIAG0..DIAG5 read-only inputs."));
  Serial.println(F("- No motors."));
  Serial.println(F("- No CAN."));
  Serial.println(F("- No TMC."));
  Serial.println(F("- No homing."));
  Serial.println(F("- No step pulses."));
  Serial.println(F("- No writes to motor pins."));
}

static void printStatus() {
  for (int i = 0; i < DIAG_COUNT; ++i) {
    Serial.print(diagInputs[i].name);
    Serial.print(F(" "));
    Serial.print(diagInputs[i].pinName);
    Serial.print(F(" "));
    Serial.println(levelName(readDiag(i)));
  }
}

static void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == F("help")) {
    printHelp();
  } else if (command == F("safe")) {
    printSafety();
  } else if (command == F("raw")) {
    printRawBits("raw=");
  } else if (command == F("status")) {
    printStatus();
  } else if (command == F("stream on")) {
    streamEnabled = true;
    lastStreamMs = 0;
    Serial.println(F("stream: on"));
  } else if (command == F("stream off")) {
    streamEnabled = false;
    Serial.println(F("stream: off"));
  } else if (command == F("pullup on")) {
    usePullup = true;
    applyPinModes();
    Serial.println(F("pullup: on"));
  } else if (command == F("pullup off")) {
    usePullup = false;
    applyPinModes();
    Serial.println(F("pullup: off"));
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type help."));
  }
  Serial.flush();
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);

  applyPinModes();

  Serial.println(F("--- OCTOPUS LIMIT OPTO BASELINE ---"));
  Serial.print(F("Build: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  printSafety();
  Serial.println(F("Type help."));
  Serial.flush();
}

void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.replace("\r", "");
    handleCommand(command);
  }

  if (streamEnabled && (now - lastStreamMs) >= STREAM_INTERVAL_MS) {
    lastStreamMs = now;
    printRawBits("LIMITS raw=");
  }

  if (!streamEnabled && (now - lastAliveMs) >= ALIVE_INTERVAL_MS) {
    lastAliveMs = now;
    Serial.print(F("alive millis="));
    Serial.println(now);
    Serial.flush();
  }
}
