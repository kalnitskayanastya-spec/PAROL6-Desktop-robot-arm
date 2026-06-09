#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#include "iodefs.h"

static constexpr uint16_t MOTOR_CURRENT_MA_RMS = 200;
static constexpr uint16_t MOTOR_MICROSTEPS = 16;
static constexpr uint16_t MAX_STEP_COMMAND = 200;
static constexpr uint16_t STEP_PULSE_DELAY_US = 1500;

static TMC5160Stepper motor0_driver(SELECT1, R_SENSE);

static bool motorEnabled = false;
static bool dirState = false;
static uint32_t stepCounter = 0;
static String commandLine;

static void disableAllMotorOutputs()
{
  pinMode(GLOBAL_ENABLE, OUTPUT);
  pinMode(ENABLE_M1, OUTPUT);
  pinMode(ENABLE_M2, OUTPUT);
  pinMode(ENABLE_M3, OUTPUT);
  pinMode(ENABLE_M4, OUTPUT);
  pinMode(ENABLE_M5, OUTPUT);

  digitalWrite(GLOBAL_ENABLE, HIGH);
  digitalWrite(ENABLE_M1, HIGH);
  digitalWrite(ENABLE_M2, HIGH);
  digitalWrite(ENABLE_M3, HIGH);
  digitalWrite(ENABLE_M4, HIGH);
  digitalWrite(ENABLE_M5, HIGH);
  motorEnabled = false;
}

static void keepUnusedMotorOutputsDisabled()
{
  digitalWrite(ENABLE_M1, HIGH);
  digitalWrite(ENABLE_M2, HIGH);
  digitalWrite(ENABLE_M3, HIGH);
  digitalWrite(ENABLE_M4, HIGH);
  digitalWrite(ENABLE_M5, HIGH);
}

static void printHex32(const char *name, uint32_t value)
{
  SerialUSB.print("  ");
  SerialUSB.print(name);
  SerialUSB.print(" = 0x");
  SerialUSB.println(value, HEX);
}

static void printHelp()
{
  SerialUSB.println("Commands:");
  SerialUSB.println("  help       - print this help");
  SerialUSB.println("  status     - print MOTOR0 and TMC5160 status");
  SerialUSB.println("  enable     - enable only MOTOR0 / Joint1");
  SerialUSB.println("  disable    - disable MOTOR0 / Joint1");
  SerialUSB.println("  dir 0      - set DIR LOW");
  SerialUSB.println("  dir 1      - set DIR HIGH");
  SerialUSB.println("  step 10    - generate 10 slow STEP pulses if enabled");
  SerialUSB.println("  step 100   - generate 100 slow STEP pulses if enabled");
  SerialUSB.println("Max step command is 200 pulses.");
}

static void printStatus()
{
  keepUnusedMotorOutputsDisabled();

  SerialUSB.println("--- ONE MOTOR TEST STATUS ---");
  SerialUSB.print("enabled = ");
  SerialUSB.println(motorEnabled ? "true" : "false");
  SerialUSB.print("step_counter = ");
  SerialUSB.println(stepCounter);
  SerialUSB.print("DIR = ");
  SerialUSB.println(dirState ? "HIGH" : "LOW");
  SerialUSB.println("MOTOR0 Joint1 STEP=PF13 DIR=PF12 CS=PC4 EN=PF14");
  SerialUSB.println("MOTOR1..MOTOR5 EN pins are HIGH / disabled.");
  SerialUSB.print("configured_current_mA_RMS = ");
  SerialUSB.println(MOTOR_CURRENT_MA_RMS);
  SerialUSB.print("configured_microsteps = ");
  SerialUSB.println(MOTOR_MICROSTEPS);

  const uint8_t connection = motor0_driver.test_connection();
  SerialUSB.print("  test_connection = ");
  SerialUSB.println(connection);

  SerialUSB.print("  version = 0x");
  SerialUSB.println(motor0_driver.version(), HEX);

  printHex32("GSTAT", motor0_driver.GSTAT());
  printHex32("DRV_STATUS", motor0_driver.DRV_STATUS());
  printHex32("GCONF", motor0_driver.GCONF());
  printHex32("CHOPCONF", motor0_driver.CHOPCONF());
}

static void enableMotor0()
{
  keepUnusedMotorOutputsDisabled();
  digitalWrite(GLOBAL_ENABLE, LOW);
  motorEnabled = true;
  SerialUSB.println("MOTOR0 ENABLED");
}

static void disableMotor0()
{
  digitalWrite(GLOBAL_ENABLE, HIGH);
  motorEnabled = false;
  keepUnusedMotorOutputsDisabled();
  SerialUSB.println("MOTOR0 DISABLED");
}

static void setDirection(bool high)
{
  dirState = high;
  digitalWrite(DIR1, dirState ? HIGH : LOW);
  SerialUSB.print("DIR ");
  SerialUSB.println(dirState ? "HIGH" : "LOW");
}

static void doSteps(uint16_t count)
{
  if (!motorEnabled) {
    SerialUSB.println("ERROR: motor disabled. Run 'enable' before 'step'.");
    return;
  }

  if (count == 0 || count > MAX_STEP_COMMAND) {
    SerialUSB.println("ERROR: step count must be 1..200.");
    return;
  }

  keepUnusedMotorOutputsDisabled();
  for (uint16_t i = 0; i < count; i++) {
    digitalWrite(PUL1, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY_US);
    digitalWrite(PUL1, LOW);
    delayMicroseconds(STEP_PULSE_DELAY_US);
    stepCounter++;
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
  } else if (line == "enable") {
    enableMotor0();
  } else if (line == "disable") {
    disableMotor0();
  } else if (line == "dir 0") {
    setDirection(false);
  } else if (line == "dir 1") {
    setDirection(true);
  } else {
    uint16_t count = 0;
    if (parseUnsignedArg(line, "step ", &count)) {
      doSteps(count);
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

static void configureMotor0Driver()
{
  pinMode(SELECT1, OUTPUT);
  digitalWrite(SELECT1, HIGH);

  motor0_driver.begin();
  motor0_driver.setSPISpeed(2000000);
  motor0_driver.rms_current(MOTOR_CURRENT_MA_RMS);
  motor0_driver.en_pwm_mode(1);
  motor0_driver.toff(4);
  motor0_driver.blank_time(24);
  motor0_driver.pwm_autoscale(1);
  motor0_driver.microsteps(MOTOR_MICROSTEPS);
}

void setup()
{
  disableAllMotorOutputs();

  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  digitalWrite(PUL1, LOW);
  digitalWrite(DIR1, LOW);

  SerialUSB.begin(115200);
  delay(1500);

  SerialUSB.println();
  SerialUSB.println("ONE MOTOR TEST START");
  SerialUSB.println("Startup state: MOTOR0 EN=HIGH disabled, no automatic movement.");

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(SCK);
  SPI.begin();
  configureMotor0Driver();

  disableMotor0();
  SerialUSB.println("SPI initialized: MOSI PA7 / MISO PA6 / SCK PA5.");
  printHelp();
}

void loop()
{
  keepUnusedMotorOutputsDisabled();
  pollSerialCommands();
}
