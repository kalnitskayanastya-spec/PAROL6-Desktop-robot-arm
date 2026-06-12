#include <Arduino.h>

#define Serial SerialUSB

static const char *FIRMWARE_NAME = "octopus_parol6_limit_opto_diag_0002";
static const char *ENV_NAME = "octopus_parol6_limit_opto_diag_0002";
static const unsigned long USB_BAUD = 115200;
static const unsigned long ALIVE_INTERVAL_MS = 1000;
static const unsigned long STREAM_INTERVAL_MS = 250;
static const unsigned long DEFAULT_DEBOUNCE_MS = 20;
static const unsigned long MAX_DEBOUNCE_MS = 1000;
static const int INPUT_COUNT = 6;

struct StopInput {
  const char *stopName;
  const char *diagName;
  const char *pinName;
  uint32_t pin;
  bool rawHigh;
  bool lastRawHigh;
  bool debouncedHigh;
  bool lastTriggered;
  unsigned long lastRawChangeMs;
  unsigned long triggerCount;
};

// Octopus Stop0..Stop5 / DIAG0..DIAG5, hardware validated through optocouplers.
// Raw bit order is DIAG0..DIAG5. Idle raw=111111; triggered inputs are active LOW.
static StopInput inputs[INPUT_COUNT] = {
    {"Stop0", "DIAG0", "PG6", PG6, false, false, false, false, 0, 0},
    {"Stop1", "DIAG1", "PG9", PG9, false, false, false, false, 0, 0},
    {"Stop2", "DIAG2", "PG10", PG10, false, false, false, false, 0, 0},
    {"Stop3", "DIAG3", "PG11", PG11, false, false, false, false, 0, 0},
    {"Stop4", "DIAG4", "PG12", PG12, false, false, false, false, 0, 0},
    {"Stop5", "DIAG5", "PG13", PG13, false, false, false, false, 0, 0},
};

static bool usePullup = true;
static bool activeLow = true;
static bool streamEnabled = false;
static unsigned long debounceMs = DEFAULT_DEBOUNCE_MS;
static unsigned long lastAliveMs = 0;
static unsigned long lastStreamMs = 0;

static const char *levelName(bool high) {
  return high ? "HIGH" : "LOW";
}

static char levelBit(bool high) {
  return high ? '1' : '0';
}

static bool interpretedTriggered(bool debouncedHigh) {
  return activeLow ? !debouncedHigh : debouncedHigh;
}

static const char *stateName(bool triggered) {
  return triggered ? "TRIGGERED" : "OPEN";
}

static bool readRawHigh(int index) {
  return digitalRead(inputs[index].pin) == HIGH;
}

static void sampleInput(int index, unsigned long now) {
  StopInput &input = inputs[index];
  bool rawHigh = readRawHigh(index);
  input.rawHigh = rawHigh;

  if (rawHigh != input.lastRawHigh) {
    input.lastRawHigh = rawHigh;
    input.lastRawChangeMs = now;
  }

  if ((now - input.lastRawChangeMs) >= debounceMs && input.debouncedHigh != rawHigh) {
    input.debouncedHigh = rawHigh;
    bool triggered = interpretedTriggered(input.debouncedHigh);
    if (triggered && !input.lastTriggered) {
      input.triggerCount++;
    }
    input.lastTriggered = triggered;
  }
}

static void updateInputs() {
  unsigned long now = millis();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    sampleInput(i, now);
  }
}

static void initializeInputState() {
  unsigned long now = millis();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    bool rawHigh = readRawHigh(i);
    inputs[i].rawHigh = rawHigh;
    inputs[i].lastRawHigh = rawHigh;
    inputs[i].debouncedHigh = rawHigh;
    inputs[i].lastTriggered = interpretedTriggered(rawHigh);
    inputs[i].lastRawChangeMs = now;
    inputs[i].triggerCount = 0;
  }
}

static void applyPinModes() {
  for (int i = 0; i < INPUT_COUNT; ++i) {
    pinMode(inputs[i].pin, usePullup ? INPUT_PULLUP : INPUT);
  }
  initializeInputState();
}

static void printRawBits(const char *prefix) {
  updateInputs();
  Serial.print(prefix);
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(levelBit(inputs[i].rawHigh));
  }
  Serial.println();
}

static void printStatesCsv() {
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(stateName(interpretedTriggered(inputs[i].debouncedHigh)));
    if (i < INPUT_COUNT - 1) {
      Serial.print(F(","));
    }
  }
}

static void printCountsCsv() {
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].triggerCount);
    if (i < INPUT_COUNT - 1) {
      Serial.print(F(","));
    }
  }
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("help"));
  Serial.println(F("version"));
  Serial.println(F("safe"));
  Serial.println(F("raw"));
  Serial.println(F("status"));
  Serial.println(F("stream on"));
  Serial.println(F("stream off"));
  Serial.println(F("pullup on"));
  Serial.println(F("pullup off"));
  Serial.println(F("active_low on"));
  Serial.println(F("active_low off"));
  Serial.println(F("debounce"));
  Serial.println(F("debounce N"));
  Serial.println(F("counts"));
  Serial.println(F("reset_counts"));
}

static void printVersion() {
  Serial.print(F("firmware="));
  Serial.println(FIRMWARE_NAME);
  Serial.print(F("env="));
  Serial.println(ENV_NAME);
  Serial.print(F("build="));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  Serial.print(F("usb_baud="));
  Serial.println(USB_BAUD);
  Serial.println(F("mapping=Stop0/DIAG0/PG6,Stop1/DIAG1/PG9,Stop2/DIAG2/PG10,Stop3/DIAG3/PG11,Stop4/DIAG4/PG12,Stop5/DIAG5/PG13"));
  Serial.println(F("raw_bit_order=DIAG0,DIAG1,DIAG2,DIAG3,DIAG4,DIAG5"));
}

static void printSafe() {
  Serial.println(F("SAFETY:"));
  Serial.println(F("- Input-only diagnostic firmware."));
  Serial.println(F("- No motors."));
  Serial.println(F("- No CAN."));
  Serial.println(F("- No TMC."));
  Serial.println(F("- No homing."));
  Serial.println(F("- No step pulses."));
  Serial.println(F("- DIAG active state is LOW by default."));
}

static void printStatus() {
  updateInputs();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(inputs[i].stopName);
    Serial.print(F(" "));
    Serial.print(inputs[i].diagName);
    Serial.print(F(" "));
    Serial.print(inputs[i].pinName);
    Serial.print(F(" raw="));
    Serial.print(levelName(inputs[i].rawHigh));
    Serial.print(F(" debounced="));
    Serial.print(levelName(inputs[i].debouncedHigh));
    Serial.print(F(" state="));
    Serial.print(stateName(interpretedTriggered(inputs[i].debouncedHigh)));
    Serial.print(F(" count="));
    Serial.println(inputs[i].triggerCount);
  }
}

static void printStreamLine() {
  updateInputs();
  Serial.print(F("LIMITS raw="));
  for (int i = 0; i < INPUT_COUNT; ++i) {
    Serial.print(levelBit(inputs[i].rawHigh));
  }
  Serial.print(F(" states="));
  printStatesCsv();
  Serial.print(F(" counts="));
  printCountsCsv();
  Serial.println();
}

static void printCounts() {
  updateInputs();
  Serial.print(F("counts="));
  printCountsCsv();
  Serial.println();
}

static void resetCounts() {
  updateInputs();
  for (int i = 0; i < INPUT_COUNT; ++i) {
    inputs[i].triggerCount = 0;
    inputs[i].lastTriggered = interpretedTriggered(inputs[i].debouncedHigh);
  }
  Serial.println(F("counts reset."));
}

static bool parseUnsignedLong(const String &text, unsigned long *value) {
  if (text.length() == 0) {
    return false;
  }
  char *end = nullptr;
  unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

static void handleDebounceCommand(const String &command) {
  if (command == F("debounce")) {
    Serial.print(F("debounce_ms="));
    Serial.println(debounceMs);
    return;
  }

  String valueText = command.substring(String("debounce ").length());
  valueText.trim();
  unsigned long value = 0;
  if (!parseUnsignedLong(valueText, &value) || value > MAX_DEBOUNCE_MS) {
    Serial.println(F("ERROR: debounce N requires N in range 0..1000 ms."));
    return;
  }

  debounceMs = value;
  initializeInputState();
  Serial.print(F("debounce_ms="));
  Serial.println(debounceMs);
}

static void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == F("help")) {
    printHelp();
  } else if (command == F("version")) {
    printVersion();
  } else if (command == F("safe")) {
    printSafe();
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
  } else if (command == F("active_low on")) {
    activeLow = true;
    initializeInputState();
    Serial.println(F("active_low: on"));
  } else if (command == F("active_low off")) {
    activeLow = false;
    initializeInputState();
    Serial.println(F("active_low: off"));
  } else if (command == F("debounce") || command.startsWith(F("debounce "))) {
    handleDebounceCommand(command);
  } else if (command == F("counts")) {
    printCounts();
  } else if (command == F("reset_counts")) {
    resetCounts();
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type help."));
  }
  Serial.flush();
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(1000);

  applyPinModes();

  Serial.println(F("--- OCTOPUS LIMIT OPTO DIAG V2 ---"));
  printVersion();
  printSafe();
  Serial.println(F("Defaults: pullup=on active_low=on debounce_ms=20 stream_interval_ms=250"));
  Serial.println(F("Type help."));
  Serial.flush();
}

void loop() {
  unsigned long now = millis();
  updateInputs();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.replace("\r", "");
    handleCommand(command);
  }

  if (streamEnabled && (now - lastStreamMs) >= STREAM_INTERVAL_MS) {
    lastStreamMs = now;
    printStreamLine();
  }

  if (!streamEnabled && (now - lastAliveMs) >= ALIVE_INTERVAL_MS) {
    lastAliveMs = now;
    Serial.print(F("alive millis="));
    Serial.println(now);
    Serial.flush();
  }
}
