#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#define Serial SerialUSB

static const char *FIRMWARE_VERSION = "octopus_parol6_motor_slot_test_0000";
static const int SLOT_COUNT = 6;
static const int NO_SLOT = -1;
static const int MAX_STEP_PULSES = 200;
static const unsigned int STEP_PULSE_US = 500;
static const float RUN_CURRENT_SCALE = 0.85f;
static const float R_SENSE = 0.075f;
static const uint16_t MICROSTEP = 32;
static const uint16_t MOTOR1_MAX_CURRENT = 2000;
static const uint16_t MOTOR2_MAX_CURRENT = 2000;
static const uint16_t MOTOR3_MAX_CURRENT = 1900;
static const uint16_t MOTOR4_MAX_CURRENT = 1700;
static const uint16_t MOTOR5_MAX_CURRENT = 1700;
static const uint16_t MOTOR6_MAX_CURRENT = 965;

struct MotorSlot {
  const char *physical;
  const char *joint;
  uint32_t stepPin;
  uint32_t dirPin;
  uint32_t enablePin;
  uint32_t csPin;
  uint16_t maxCurrentMa;
};

static const MotorSlot slots[SLOT_COUNT] = {
    {"MOTOR0", "Joint1", PF13, PF12, PF14, PC4, MOTOR1_MAX_CURRENT},
    {"MOTOR1", "Joint2", PG0, PG1, PF15, PD11, MOTOR2_MAX_CURRENT},
    {"MOTOR2", "Joint3", PF11, PG3, PG5, PC6, MOTOR3_MAX_CURRENT},
    {"MOTOR3", "Joint4", PG4, PC1, PA0, PC7, MOTOR4_MAX_CURRENT},
    {"MOTOR4", "Joint5", PF9, PF10, PG2, PF2, MOTOR5_MAX_CURRENT},
    {"MOTOR5", "Joint6", PC13, PF0, PF1, PE4, MOTOR6_MAX_CURRENT},
};

static TMC5160Stepper drivers[SLOT_COUNT] = {
    TMC5160Stepper(PC4, R_SENSE),
    TMC5160Stepper(PD11, R_SENSE),
    TMC5160Stepper(PC6, R_SENSE),
    TMC5160Stepper(PC7, R_SENSE),
    TMC5160Stepper(PF2, R_SENSE),
    TMC5160Stepper(PE4, R_SENSE),
};

static int selectedSlot = NO_SLOT;
static bool enabled[SLOT_COUNT] = {false, false, false, false, false, false};
static bool dirState[SLOT_COUNT] = {false, false, false, false, false, false};
static long stepCounter[SLOT_COUNT] = {0, 0, 0, 0, 0, 0};

const char *physicalConnectorName(int slot) {
  if (slot < 0 || slot >= SLOT_COUNT) {
    return "NONE";
  }
  return slots[slot].physical;
}

const char *jointName(int slot) {
  if (slot < 0 || slot >= SLOT_COUNT) {
    return "NONE";
  }
  return slots[slot].joint;
}

bool validateSelectedSlot() {
  if (selectedSlot < 0 || selectedSlot >= SLOT_COUNT) {
    Serial.println(F("ERROR: no selected slot. Use select 0..5 first."));
    return false;
  }
  return true;
}

static uint16_t currentSettingMa(int slot) {
  return (uint16_t)(slots[slot].maxCurrentMa * RUN_CURRENT_SCALE);
}

static void setEnablePin(int slot, bool enable) {
  digitalWrite(slots[slot].enablePin, enable ? LOW : HIGH);
  enabled[slot] = enable;
}

static void disableAllMotors() {
  for (int i = 0; i < SLOT_COUNT; ++i) {
    setEnablePin(i, false);
  }
}

static void setDirState(int slot, bool state) {
  digitalWrite(slots[slot].dirPin, state ? HIGH : LOW);
  dirState[slot] = state;
}

void printSlots() {
  Serial.println(F("select 0 -> MOTOR0 / Joint1"));
  Serial.println(F("select 1 -> MOTOR1 / Joint2"));
  Serial.println(F("select 2 -> MOTOR2 / Joint3"));
  Serial.println(F("MOTOR2_2 -> extra physical connector / duplicate MOTOR2 output, NOT select 3"));
  Serial.println(F("select 3 -> MOTOR3 / Joint4, physically AFTER MOTOR2_2"));
  Serial.println(F("select 4 -> MOTOR4 / Joint5"));
  Serial.println(F("select 5 -> MOTOR5 / Joint6"));
}

void printWhere() {
  if (!validateSelectedSlot()) {
    return;
  }

  Serial.print(F("Selected slot "));
  Serial.print(selectedSlot);
  Serial.print(F(" = "));
  Serial.print(physicalConnectorName(selectedSlot));
  Serial.print(F(" / "));
  Serial.println(jointName(selectedSlot));

  Serial.print(F("Plug motor cable into physical "));
  Serial.print(physicalConnectorName(selectedSlot));
  Serial.println(F(" connector."));

  if (selectedSlot == 3) {
    Serial.println(F("Important: MOTOR3 is after MOTOR2_2 on the Octopus board."));
    Serial.println(F("Do not plug into MOTOR2_2."));
  }
}

void printSafety() {
  Serial.println(F("SAFETY:"));
  Serial.println(F("- All motors are disabled on startup."));
  Serial.println(F("- Use disable before changing selected slot."));
  Serial.println(F("- Turn PSU OUTPUT OFF before moving motor connector."));
  Serial.println(F("- Never plug/unplug motors under power."));
  Serial.println(F("- No homing in this firmware."));
  Serial.println(F("- No automatic motion."));
  Serial.println(F("- Max step command is 200 pulses."));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("help"));
  Serial.println(F("slots"));
  Serial.println(F("where"));
  Serial.println(F("safe"));
  Serial.println(F("select 0..5"));
  Serial.println(F("status"));
  Serial.println(F("all_status"));
  Serial.println(F("enable"));
  Serial.println(F("disable"));
  Serial.println(F("dir 0"));
  Serial.println(F("dir 1"));
  Serial.println(F("step N"));
  Serial.println(F("jog N"));
}

static void printSlotStatus(int slot) {
  Serial.print(F("slot: "));
  Serial.println(slot);
  Serial.print(F("physical motor connector: "));
  Serial.println(physicalConnectorName(slot));
  Serial.print(F("joint: "));
  Serial.println(jointName(slot));
  Serial.print(F("enabled: "));
  Serial.println(enabled[slot] ? F("YES") : F("NO"));
  Serial.print(F("DIR state: "));
  Serial.println(dirState[slot] ? 1 : 0);
  Serial.print(F("step counter: "));
  Serial.println(stepCounter[slot]);
  Serial.print(F("TMC test_connection: "));
  Serial.println(drivers[slot].test_connection());
  Serial.print(F("version: "));
  Serial.println(FIRMWARE_VERSION);
  Serial.print(F("current setting mA: "));
  Serial.println(currentSettingMa(slot));
  Serial.print(F("microsteps: "));
  Serial.println(MICROSTEP);
}

static void printStatus() {
  if (!validateSelectedSlot()) {
    return;
  }
  printSlotStatus(selectedSlot);
}

static void printAllStatus() {
  for (int i = 0; i < SLOT_COUNT; ++i) {
    printSlotStatus(i);
    if (i < SLOT_COUNT - 1) {
      Serial.println(F("---"));
    }
  }
}

static bool parseIntArgument(const String &command, const char *prefix, long *value) {
  String prefixText(prefix);
  if (!command.startsWith(prefixText)) {
    return false;
  }

  String arg = command.substring(prefixText.length());
  arg.trim();
  if (arg.length() == 0) {
    return false;
  }

  char *end = nullptr;
  long parsed = strtol(arg.c_str(), &end, 10);
  if (end == arg.c_str() || *end != '\0') {
    return false;
  }

  *value = parsed;
  return true;
}

static void pulseSelectedMotor(unsigned int pulses) {
  for (unsigned int i = 0; i < pulses; ++i) {
    digitalWrite(slots[selectedSlot].stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(slots[selectedSlot].stepPin, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
}

bool parseStepLikeCommand(const String &command, const char *verb) {
  long pulses = 0;
  String prefix = String(verb) + " ";
  if (!parseIntArgument(command, prefix.c_str(), &pulses)) {
    return false;
  }

  if (!validateSelectedSlot()) {
    return true;
  }

  if (!enabled[selectedSlot]) {
    Serial.println(F("ERROR: selected slot is disabled. Use enable first."));
    return true;
  }

  long magnitude = labs(pulses);
  if (magnitude == 0) {
    Serial.println(F("ERROR: pulse count must be non-zero."));
    return true;
  }
  if (magnitude > MAX_STEP_PULSES) {
    Serial.println(F("ERROR: max step command is 200 pulses."));
    return true;
  }

  bool originalDir = dirState[selectedSlot];
  bool usedDir = originalDir;
  if (pulses < 0) {
    usedDir = !originalDir;
    setDirState(selectedSlot, usedDir);
  }

  pulseSelectedMotor((unsigned int)magnitude);
  stepCounter[selectedSlot] += (pulses < 0) ? -magnitude : magnitude;

  if (pulses < 0) {
    setDirState(selectedSlot, originalDir);
  }

  Serial.print(F("Moved "));
  Serial.print((int)magnitude);
  Serial.print(F(" pulses on slot "));
  Serial.print(selectedSlot);
  Serial.print(F(" using DIR "));
  Serial.print(usedDir ? 1 : 0);
  if (pulses < 0) {
    Serial.print(F(" (negative command; DIR restored to "));
    Serial.print(originalDir ? 1 : 0);
    Serial.print(F(")"));
  }
  Serial.println(F("."));
  return true;
}

static void selectSlot(int slot) {
  if (slot < 0 || slot >= SLOT_COUNT) {
    Serial.println(F("ERROR: select slot must be 0..5."));
    return;
  }

  disableAllMotors();
  selectedSlot = slot;

  Serial.print(F("Selected slot "));
  Serial.print(selectedSlot);
  Serial.print(F(" = "));
  Serial.print(physicalConnectorName(selectedSlot));
  Serial.print(F(" / "));
  Serial.println(jointName(selectedSlot));
  Serial.println(F("All motors disabled. Use where, safe, then enable."));
}

static void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == F("help")) {
    printHelp();
  } else if (command == F("slots")) {
    printSlots();
  } else if (command == F("where")) {
    printWhere();
  } else if (command == F("safe")) {
    printSafety();
  } else if (command == F("status")) {
    printStatus();
  } else if (command == F("all_status")) {
    printAllStatus();
  } else if (command == F("enable")) {
    if (validateSelectedSlot()) {
      disableAllMotors();
      setEnablePin(selectedSlot, true);
      Serial.print(F("Enabled slot "));
      Serial.print(selectedSlot);
      Serial.print(F(" only: "));
      Serial.print(physicalConnectorName(selectedSlot));
      Serial.print(F(" / "));
      Serial.println(jointName(selectedSlot));
    }
  } else if (command == F("disable")) {
    disableAllMotors();
    Serial.println(F("All motors disabled."));
  } else if (command.startsWith(F("select "))) {
    long slot = 0;
    if (parseIntArgument(command, "select ", &slot)) {
      selectSlot((int)slot);
    } else {
      Serial.println(F("ERROR: use select 0..5."));
    }
  } else if (command.startsWith(F("dir "))) {
    long dir = 0;
    if (!parseIntArgument(command, "dir ", &dir) || (dir != 0 && dir != 1)) {
      Serial.println(F("ERROR: use dir 0 or dir 1."));
    } else if (validateSelectedSlot()) {
      setDirState(selectedSlot, dir == 1);
      Serial.print(F("DIR state for slot "));
      Serial.print(selectedSlot);
      Serial.print(F(" set to "));
      Serial.println(dir);
    }
  } else if (command.startsWith(F("step "))) {
    parseStepLikeCommand(command, "step");
  } else if (command.startsWith(F("jog "))) {
    parseStepLikeCommand(command, "jog");
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type help."));
  }
}

static void initPins() {
  for (int i = 0; i < SLOT_COUNT; ++i) {
    pinMode(slots[i].stepPin, OUTPUT);
    pinMode(slots[i].dirPin, OUTPUT);
    pinMode(slots[i].enablePin, OUTPUT);
    pinMode(slots[i].csPin, OUTPUT);
    digitalWrite(slots[i].stepPin, LOW);
    digitalWrite(slots[i].dirPin, LOW);
    digitalWrite(slots[i].enablePin, HIGH);
    digitalWrite(slots[i].csPin, HIGH);
  }
}

static void initDrivers() {
  SPI.setMOSI(PA7);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5);
  SPI.begin();

  for (int i = 0; i < SLOT_COUNT; ++i) {
    drivers[i].begin();
    drivers[i].rms_current(currentSettingMa(i));
    drivers[i].en_pwm_mode(1);
    drivers[i].toff(4);
    drivers[i].blank_time(24);
    drivers[i].pwm_autoscale(1);
    drivers[i].microsteps(MICROSTEP);
  }
}

void setup() {
  Serial.begin(3000000);
  delay(1500);

  initPins();
  disableAllMotors();
  initDrivers();

  Serial.println(FIRMWARE_VERSION);
  Serial.println(F("All motors disabled on startup."));
  printSafety();
  Serial.println(F("Type help."));
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.replace("\r", "");
    handleCommand(command);
  }
}
