#include <Arduino.h>

#define Serial SerialUSB

static const unsigned long BAUD_RATE = 115200;
static const unsigned long DEBOUNCE_MS = 20;
static const unsigned long STREAM_INTERVAL_MS = 250;
static const int INPUT_COUNT = 6;

struct LimitInput {
  const char *stopName;
  const char *limitName;
  const char *jointName;
  const char *pinName;
  uint32_t pin;
  bool rawState;
  bool stableState;
  bool lastRawState;
  unsigned long lastRawChangeMs;
  unsigned long changedCount;
};

static LimitInput inputs[INPUT_COUNT] = {
    {"Stop0", "LIMIT1", "J1", "PG6", PG6},
    {"Stop1", "LIMIT2", "J2", "PG9", PG9},
    {"Stop2", "LIMIT3", "J3", "PG10", PG10},
    {"Stop3", "LIMIT4", "J4", "PG11", PG11},
    {"Stop4", "LIMIT5", "J5", "PG12", PG12},
    {"Stop5", "LIMIT6", "J6", "PG13", PG13},
};

static const uint32_t motorEnablePins[] = {
    PF14, PF15, PG5, PA0, PG2, PF1,
};

static bool invertInputs = false;
static bool usePullup = false;
static bool streamEnabled = false;
static unsigned long lastStreamMs = 0;

static const char *levelName(bool state) {
  return state ? "HIGH" : "LOW";
}

static char levelBit(bool state) {
  return state ? '1' : '0';
}

static bool activeState(int index) {
  return invertInputs ? !inputs[index].stableState : inputs[index].stableState;
}

void printSafety() {
  Serial.println(F("SAFETY:"));
  Serial.println(F("- This firmware reads Stop0..Stop5 only."));
  Serial.println(F("- No motors are enabled."));
  Serial.println(F("- No step pulses are generated."));
  Serial.println(F("- Never feed 24V directly into Octopus MCU inputs."));
  Serial.println(F("- Verify optocoupler output voltage before connecting to Stop inputs."));
}

void printMapping() {
  Serial.println(F("Stop0 / LIMIT1 -> Joint1"));
  Serial.println(F("Stop1 / LIMIT2 -> Joint2"));
  Serial.println(F("Stop2 / LIMIT3 -> Joint3"));
  Serial.println(F("Stop3 / LIMIT4 -> Joint4"));
  Serial.println(F("Stop4 / LIMIT5 -> Joint5"));
  Serial.println(F("Stop5 / LIMIT6 -> Joint6"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("help"));
  Serial.println(F("status"));
  Serial.println(F("raw"));
  Serial.println(F("stream on"));
  Serial.println(F("stream off"));
  Serial.println(F("invert on"));
  Serial.println(F("invert off"));
  Serial.println(F("pullup on"));
  Serial.println(F("pullup off"));
  Serial.println(F("counts"));
  Serial.println(F("reset_counts"));
  Serial.println(F("safe"));
}

void applyPinModes() {
  for (int i = 0; i < INPUT_COUNT; ++i) {
    pinMode(inputs[i].pin, usePullup ? INPUT_PULLUP : INPUT);
    bool state = digitalRead(inputs[i].pin) == HIGH;
    inputs[i].rawState = state;
    inputs[i].stableState = state;
    inputs[i].lastRawState = state;
    inputs[i].lastRawChangeMs = millis();
  }
}

void updateInputs() {
  unsigned long now = millis();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    bool currentRaw = digitalRead(inputs[i].pin) == HIGH;
    inputs[i].rawState = currentRaw;

    if (currentRaw != inputs[i].lastRawState) {
      inputs[i].lastRawState = currentRaw;
      inputs[i].lastRawChangeMs = now;
    }

    if ((now - inputs[i].lastRawChangeMs) >= DEBOUNCE_MS && inputs[i].stableState != currentRaw) {
      inputs[i].stableState = currentRaw;
      inputs[i].changedCount++;
    }
  }
}

void printStatus() {
  updateInputs();
  Serial.println(F("Input  Joint  PinName  Raw   Active  ChangedCount"));
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].stopName);
    Serial.print(F("  "));
    Serial.print(inputs[i].jointName);
    Serial.print(F("     "));
    Serial.print(inputs[i].pinName);
    Serial.print(F("     "));
    Serial.print(levelName(inputs[i].stableState));
    if (!inputs[i].stableState) {
      Serial.print(F(" "));
    }
    Serial.print(F("  "));
    Serial.print(levelName(activeState(i)));
    if (!activeState(i)) {
      Serial.print(F(" "));
    }
    Serial.print(F("   "));
    Serial.println(inputs[i].changedCount);
  }
}

void printRaw() {
  updateInputs();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].stopName);
    Serial.print(F("="));
    Serial.print(levelName(inputs[i].rawState));
    if (i < INPUT_COUNT - 1) {
      Serial.print(F(" "));
    }
  }
  Serial.println();
}

void printCounts() {
  updateInputs();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].stopName);
    Serial.print(F("="));
    Serial.print(inputs[i].changedCount);
    if (i < INPUT_COUNT - 1) {
      Serial.print(F(" "));
    }
  }
  Serial.println();
}

void resetCounts() {
  for (int i = 0; i < INPUT_COUNT; ++i) {
    inputs[i].changedCount = 0;
  }
  Serial.println(F("Changed counts reset."));
}

void printStreamLine() {
  updateInputs();
  Serial.print(F("LIMITS raw="));
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(levelBit(inputs[i].stableState));
  }

  Serial.print(F(" active="));
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(levelBit(activeState(i)));
  }

  Serial.print(F(" counts="));
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].changedCount);
    if (i < INPUT_COUNT - 1) {
      Serial.print(F(","));
    }
  }
  Serial.println();
}

static void disableMotorEnables() {
  for (uint32_t pin : motorEnablePins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

static void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == F("help")) {
    printHelp();
  } else if (command == F("status")) {
    printStatus();
  } else if (command == F("raw")) {
    printRaw();
  } else if (command == F("stream on")) {
    streamEnabled = true;
    lastStreamMs = 0;
    Serial.println(F("stream: on"));
  } else if (command == F("stream off")) {
    streamEnabled = false;
    Serial.println(F("stream: off"));
  } else if (command == F("invert on")) {
    invertInputs = true;
    Serial.println(F("invert: on"));
  } else if (command == F("invert off")) {
    invertInputs = false;
    Serial.println(F("invert: off"));
  } else if (command == F("pullup on")) {
    usePullup = true;
    applyPinModes();
    Serial.println(F("pullup: on"));
  } else if (command == F("pullup off")) {
    usePullup = false;
    applyPinModes();
    Serial.println(F("pullup: off"));
  } else if (command == F("counts")) {
    printCounts();
  } else if (command == F("reset_counts")) {
    resetCounts();
  } else if (command == F("safe")) {
    printSafety();
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type help."));
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(500);

  disableMotorEnables();
  applyPinModes();

  Serial.println(F("--- OCTOPUS LIMIT / OPTO DIAG ---"));
  Serial.println(F("Safe input-only firmware."));
  Serial.println(F("No motors."));
  Serial.println(F("No homing."));
  Serial.println(F("No CAN."));
  printMapping();
  Serial.println(F("Type help."));
}

void loop() {
  updateInputs();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.replace("\r", "");
    handleCommand(command);
  }

  if (streamEnabled && (millis() - lastStreamMs) >= STREAM_INTERVAL_MS) {
    lastStreamMs = millis();
    printStreamLine();
  }
}
