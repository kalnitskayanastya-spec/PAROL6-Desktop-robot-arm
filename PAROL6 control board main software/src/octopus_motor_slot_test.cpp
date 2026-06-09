#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#include "iodefs.h"

static constexpr uint8_t SLOT_COUNT = 6;
static constexpr uint16_t MOTOR_CURRENT_MA_RMS = 200;
static constexpr uint16_t MOTOR_MICROSTEPS = 16;
static constexpr uint16_t MAX_STEP_COMMAND = 200;
static constexpr uint16_t STEP_PULSE_DELAY_US = 1500;

struct MotorSlot {
  const char *motor;
  const char *joint;
  const char *stepLabel;
  const char *dirLabel;
  const char *csLabel;
  const char *enLabel;
  uint32_t stepPin;
  uint32_t dirPin;
  uint32_t csPin;
  uint32_t enPin;
  TMC5160Stepper *driver;
  bool enabled;
  bool dirState;
  uint32_t stepCounter;
};

static TMC5160Stepper motor0_driver(SELECT1, R_SENSE);
static TMC5160Stepper motor1_driver(SELECT6, R_SENSE);
static TMC5160Stepper motor2_driver(SELECT5, R_SENSE);
static TMC5160Stepper motor3_driver(SELECT4, R_SENSE);
static TMC5160Stepper motor4_driver(SELECT2, R_SENSE);
static TMC5160Stepper motor5_driver(SELECT3, R_SENSE);

static MotorSlot slots[SLOT_COUNT] = {
  {"MOTOR0", "Joint1", "PF13", "PF12", "PC4", "PF14", PUL1, DIR1, SELECT1, GLOBAL_ENABLE, &motor0_driver, false, false, 0},
  {"MOTOR1", "Joint2", "PG0", "PG1", "PD11", "PF15", PUL6, DIR6, SELECT6, ENABLE_M1, &motor1_driver, false, false, 0},
  {"MOTOR2", "Joint3", "PF11", "PG3", "PC6", "PG5", PUL5, DIR5, SELECT5, ENABLE_M2, &motor2_driver, false, false, 0},
  {"MOTOR3", "Joint4", "PG4", "PC1", "PC7", "PA0", PUL4, DIR4, SELECT4, ENABLE_M3, &motor3_driver, false, false, 0},
  {"MOTOR4", "Joint5", "PF9", "PF10", "PF2", "PG2", PUL2, DIR2, SELECT2, ENABLE_M4, &motor4_driver, false, false, 0},
  {"MOTOR5", "Joint6", "PC13", "PF0", "PE4", "PF1", PUL3, DIR3, SELECT3, ENABLE_M5, &motor5_driver, false, false, 0},
};

static uint8_t selectedSlot = 0;
static bool slotSelected = false;
static String commandLine;

static void printHex32(const char *name, uint32_t value)
{
  SerialUSB.print("  ");
  SerialUSB.print(name);
  SerialUSB.print(" = 0x");
  SerialUSB.println(value, HEX);
}

static void printSlotPins(const MotorSlot &slot)
{
  SerialUSB.print(slot.motor);
  SerialUSB.print(" / ");
  SerialUSB.print(slot.joint);
  SerialUSB.print(" STEP=");
  SerialUSB.print(slot.stepLabel);
  SerialUSB.print(" DIR=");
  SerialUSB.print(slot.dirLabel);
  SerialUSB.print(" CS=");
  SerialUSB.print(slot.csLabel);
  SerialUSB.print(" EN=");
  SerialUSB.println(slot.enLabel);
}

static void disableAllMotors()
{
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    digitalWrite(slots[i].enPin, HIGH);
    slots[i].enabled = false;
  }
}

static void keepNonSelectedMotorsDisabled()
{
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    if (!slotSelected || i != selectedSlot) {
      digitalWrite(slots[i].enPin, HIGH);
      slots[i].enabled = false;
    }
  }
}

static void setupSafePins()
{
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    pinMode(slots[i].enPin, OUTPUT);
    digitalWrite(slots[i].enPin, HIGH);

    pinMode(slots[i].stepPin, OUTPUT);
    pinMode(slots[i].dirPin, OUTPUT);
    digitalWrite(slots[i].stepPin, LOW);
    digitalWrite(slots[i].dirPin, LOW);

    pinMode(slots[i].csPin, OUTPUT);
    digitalWrite(slots[i].csPin, HIGH);
  }
}

static void configureDrivers()
{
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    TMC5160Stepper *driver = slots[i].driver;
    driver->begin();
    driver->setSPISpeed(2000000);
    driver->rms_current(MOTOR_CURRENT_MA_RMS);
    driver->en_pwm_mode(1);
    driver->toff(4);
    driver->blank_time(24);
    driver->pwm_autoscale(1);
    driver->microsteps(MOTOR_MICROSTEPS);
  }
}

static void printHelp()
{
  SerialUSB.println("Commands:");
  SerialUSB.println("  help        - print this help");
  SerialUSB.println("  select 0    - select MOTOR0 / Joint1 and disable all motors");
  SerialUSB.println("  select 1    - select MOTOR1 / Joint2 and disable all motors");
  SerialUSB.println("  select 2    - select MOTOR2 / Joint3 and disable all motors");
  SerialUSB.println("  select 3    - select MOTOR3 / Joint4 and disable all motors");
  SerialUSB.println("  select 4    - select MOTOR4 / Joint5 and disable all motors");
  SerialUSB.println("  select 5    - select MOTOR5 / Joint6 and disable all motors");
  SerialUSB.println("  status      - print selected slot status");
  SerialUSB.println("  all_status  - print brief SPI status for all slots");
  SerialUSB.println("  enable      - enable only selected slot if SPI test_connection is OK");
  SerialUSB.println("  disable     - disable all motors");
  SerialUSB.println("  dir 0       - set selected DIR LOW");
  SerialUSB.println("  dir 1       - set selected DIR HIGH");
  SerialUSB.println("  step 10     - generate 10 slow STEP pulses if selected slot is enabled");
  SerialUSB.println("  step 100    - generate 100 slow STEP pulses if selected slot is enabled");
  SerialUSB.println("Max step command is 200 pulses.");
}

static void printStatus()
{
  keepNonSelectedMotorsDisabled();
  if (!slotSelected) {
    SerialUSB.println("--- MOTOR SLOT TEST STATUS ---");
    SerialUSB.println("selected_slot = none");
    SerialUSB.println("enabled = false");
    SerialUSB.println("All motor EN pins are HIGH / disabled.");
    return;
  }

  MotorSlot &slot = slots[selectedSlot];

  SerialUSB.println("--- MOTOR SLOT TEST STATUS ---");
  SerialUSB.print("selected_slot = ");
  SerialUSB.println(selectedSlot);
  printSlotPins(slot);
  SerialUSB.print("enabled = ");
  SerialUSB.println(slot.enabled ? "true" : "false");
  SerialUSB.print("step_counter = ");
  SerialUSB.println(slot.stepCounter);
  SerialUSB.print("DIR = ");
  SerialUSB.println(slot.dirState ? "HIGH" : "LOW");
  SerialUSB.print("configured_current_mA_RMS = ");
  SerialUSB.println(MOTOR_CURRENT_MA_RMS);
  SerialUSB.print("configured_microsteps = ");
  SerialUSB.println(MOTOR_MICROSTEPS);

  const uint8_t connection = slot.driver->test_connection();
  SerialUSB.print("  test_connection = ");
  SerialUSB.println(connection);
  SerialUSB.print("  version = 0x");
  SerialUSB.println(slot.driver->version(), HEX);
  printHex32("GSTAT", slot.driver->GSTAT());
  printHex32("DRV_STATUS", slot.driver->DRV_STATUS());
  printHex32("GCONF", slot.driver->GCONF());
  printHex32("CHOPCONF", slot.driver->CHOPCONF());
}

static void printAllStatus()
{
  keepNonSelectedMotorsDisabled();
  SerialUSB.println("--- ALL MOTOR SLOT SPI STATUS ---");
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    MotorSlot &slot = slots[i];
    SerialUSB.print("slot ");
    SerialUSB.print(i);
    SerialUSB.print(" ");
    SerialUSB.print(slot.motor);
    SerialUSB.print(" test_connection=");
    SerialUSB.print(slot.driver->test_connection());
    SerialUSB.print(" version=0x");
    SerialUSB.print(slot.driver->version(), HEX);
    SerialUSB.print(" enabled=");
    SerialUSB.println(slot.enabled ? "true" : "false");
  }
}

static void selectSlot(uint8_t slotIndex)
{
  if (slotIndex >= SLOT_COUNT) {
    SerialUSB.println("ERROR: select slot must be 0..5.");
    return;
  }

  disableAllMotors();
  selectedSlot = slotIndex;
  slotSelected = true;
  SerialUSB.print("SELECTED SLOT ");
  SerialUSB.println(selectedSlot);
  printSlotPins(slots[selectedSlot]);
  SerialUSB.println("All motor EN pins are HIGH / disabled after select.");
}

static void enableSelectedSlot()
{
  if (!slotSelected) {
    SerialUSB.println("ERROR: select a motor slot first with 'select 0'..'select 5'.");
    return;
  }

  MotorSlot &slot = slots[selectedSlot];
  const uint8_t connection = slot.driver->test_connection();
  if (connection != 0) {
    SerialUSB.print("ERROR: TMC test_connection failed, not enabling. value=");
    SerialUSB.println(connection);
    disableAllMotors();
    return;
  }

  disableAllMotors();
  digitalWrite(slot.enPin, LOW);
  slot.enabled = true;
  SerialUSB.print(slot.motor);
  SerialUSB.println(" ENABLED");
}

static void setDirection(bool high)
{
  if (!slotSelected) {
    SerialUSB.println("ERROR: select a motor slot first with 'select 0'..'select 5'.");
    return;
  }

  MotorSlot &slot = slots[selectedSlot];
  slot.dirState = high;
  digitalWrite(slot.dirPin, slot.dirState ? HIGH : LOW);
  SerialUSB.print(slot.motor);
  SerialUSB.print(" DIR ");
  SerialUSB.println(slot.dirState ? "HIGH" : "LOW");
}

static void doSteps(uint16_t count)
{
  if (!slotSelected) {
    SerialUSB.println("ERROR: select a motor slot first with 'select 0'..'select 5'.");
    return;
  }

  MotorSlot &slot = slots[selectedSlot];
  if (!slot.enabled) {
    SerialUSB.println("ERROR: selected motor disabled. Run 'enable' before 'step'.");
    return;
  }

  if (count == 0 || count > MAX_STEP_COMMAND) {
    SerialUSB.println("ERROR: step count must be 1..200.");
    return;
  }

  keepNonSelectedMotorsDisabled();
  for (uint16_t i = 0; i < count; i++) {
    digitalWrite(slot.stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY_US);
    digitalWrite(slot.stepPin, LOW);
    delayMicroseconds(STEP_PULSE_DELAY_US);
    slot.stepCounter++;
  }

  SerialUSB.print("STEPS DONE = ");
  SerialUSB.println(count);
}

static bool parseUnsignedArg(const String &line, const char *prefix, uint16_t *value)
{
  const size_t prefixLen = strlen(prefix);
  if (!line.startsWith(prefix)) {
    return false;
  }

  String arg = line.substring(prefixLen);
  arg.trim();
  if (arg.length() == 0) {
    return false;
  }

  for (uint16_t i = 0; i < arg.length(); i++) {
    if (!isDigit(arg[i])) {
      return false;
    }
  }

  const long parsed = arg.toInt();
  if (parsed < 0 || parsed > 65535L) {
    return false;
  }

  *value = static_cast<uint16_t>(parsed);
  return true;
}

static void handleCommand(String line)
{
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) {
    return;
  }

  if (line == "help") {
    printHelp();
  } else if (line == "status") {
    printStatus();
  } else if (line == "all_status") {
    printAllStatus();
  } else if (line == "enable") {
    enableSelectedSlot();
  } else if (line == "disable") {
    disableAllMotors();
    SerialUSB.println("ALL MOTORS DISABLED");
  } else if (line == "dir 0") {
    setDirection(false);
  } else if (line == "dir 1") {
    setDirection(true);
  } else {
    uint16_t value = 0;
    if (parseUnsignedArg(line, "select ", &value)) {
      if (value <= 5) {
        selectSlot(static_cast<uint8_t>(value));
      } else {
        SerialUSB.println("ERROR: select slot must be 0..5.");
      }
    } else if (parseUnsignedArg(line, "step ", &value)) {
      doSteps(value);
    } else {
      SerialUSB.println("Unknown command.");
      printHelp();
    }
  }
}

static void pollSerialCommands()
{
  while (SerialUSB.available() > 0) {
    const char c = static_cast<char>(SerialUSB.read());
    if (c == '\n' || c == '\r') {
      if (commandLine.length() > 0) {
        handleCommand(commandLine);
        commandLine = "";
      }
    } else if (commandLine.length() < 80) {
      commandLine += c;
    } else {
      commandLine = "";
      SerialUSB.println("ERROR: command too long.");
      printHelp();
    }
  }
}

void setup()
{
  setupSafePins();
  disableAllMotors();

  SerialUSB.begin(115200);
  delay(1500);

  SerialUSB.println();
  SerialUSB.println("MOTOR SLOT TEST START");
  SerialUSB.println("Startup state: all EN pins HIGH / disabled, no automatic movement.");

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(SCK);
  SPI.begin();
  configureDrivers();
  disableAllMotors();

  SerialUSB.println("SPI initialized: MOSI PA7 / MISO PA6 / SCK PA5.");
  SerialUSB.print("R_SENSE = ");
  SerialUSB.println(R_SENSE, 3);
  SerialUSB.print("configured_current_mA_RMS = ");
  SerialUSB.println(MOTOR_CURRENT_MA_RMS);
  SerialUSB.print("configured_microsteps = ");
  SerialUSB.println(MOTOR_MICROSTEPS);
  SerialUSB.println("selected_slot = none. Use 'select 0'..'select 5' before enabling.");
  printHelp();
}

void loop()
{
  keepNonSelectedMotorsDisabled();
  pollSerialCommands();
}
