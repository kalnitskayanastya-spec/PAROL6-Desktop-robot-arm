#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#include "iodefs.h"

struct TmcSlot {
  const char *motor;
  const char *joint;
  const char *csLabel;
  const char *enLabel;
  uint32_t csPin;
  uint32_t enPin;
  TMC5160Stepper *driver;
};

static TMC5160Stepper motor0_driver(SELECT1, R_SENSE);
static TMC5160Stepper motor1_driver(SELECT6, R_SENSE);
static TMC5160Stepper motor2_driver(SELECT5, R_SENSE);
static TMC5160Stepper motor3_driver(SELECT4, R_SENSE);
static TMC5160Stepper motor4_driver(SELECT2, R_SENSE);
static TMC5160Stepper motor5_driver(SELECT3, R_SENSE);

static TmcSlot slots[] = {
  {"MOTOR0", "Joint1", "PC4", "PF14", SELECT1, GLOBAL_ENABLE, &motor0_driver},
  {"MOTOR1", "Joint2", "PD11", "PF15", SELECT6, ENABLE_M1, &motor1_driver},
  {"MOTOR2", "Joint3", "PC6", "PG5", SELECT5, ENABLE_M2, &motor2_driver},
  {"MOTOR3", "Joint4", "PC7", "PA0", SELECT4, ENABLE_M3, &motor3_driver},
  {"MOTOR4", "Joint5", "PF2", "PG2", SELECT2, ENABLE_M4, &motor4_driver},
  {"MOTOR5", "Joint6", "PE4", "PF1", SELECT3, ENABLE_M5, &motor5_driver},
};

static void keepMotorOutputsDisabled()
{
  for (TmcSlot &slot : slots) {
    pinMode(slot.enPin, OUTPUT);
    digitalWrite(slot.enPin, HIGH);
  }
}

static void initChipSelectPins()
{
  for (TmcSlot &slot : slots) {
    pinMode(slot.csPin, OUTPUT);
    digitalWrite(slot.csPin, HIGH);
  }
}

static void initDriversForReadOnlySpi()
{
  for (TmcSlot &slot : slots) {
    slot.driver->setSPISpeed(2000000);
  }
}

static void printHex32(const char *name, uint32_t value)
{
  SerialUSB.print("  ");
  SerialUSB.print(name);
  SerialUSB.print(" = 0x");
  SerialUSB.println(value, HEX);
}

static void printSlot(const TmcSlot &slot)
{
  SerialUSB.print(slot.motor);
  SerialUSB.print(" ");
  SerialUSB.print(slot.joint);
  SerialUSB.print(" CS=");
  SerialUSB.print(slot.csLabel);
  SerialUSB.print(" EN=");
  SerialUSB.print(slot.enLabel);
  SerialUSB.println(" disabled");

  const uint8_t connection = slot.driver->test_connection();
  SerialUSB.print("  test_connection = ");
  SerialUSB.println(connection);

  SerialUSB.print("  version = 0x");
  SerialUSB.println(slot.driver->version(), HEX);

  printHex32("GSTAT", slot.driver->GSTAT());
  printHex32("DRV_STATUS", slot.driver->DRV_STATUS());
  printHex32("IOIN", slot.driver->IOIN());
  printHex32("GCONF", slot.driver->GCONF());
  printHex32("CHOPCONF", slot.driver->CHOPCONF());
  SerialUSB.println();
}

void setup()
{
  keepMotorOutputsDisabled();
  initChipSelectPins();

  SerialUSB.begin(115200);
  delay(1500);

  SerialUSB.println();
  SerialUSB.println("TMC ALL SLOTS DIAG START");
  SerialUSB.println("Diagnostic firmware: no movement, no step pulses, all motor enables stay HIGH.");
  SerialUSB.println("CAN init is disabled for this build.");
  SerialUSB.print("R_SENSE = ");
  SerialUSB.println(R_SENSE, 3);

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(SCK);
  SPI.begin();
  initDriversForReadOnlySpi();

  SerialUSB.println("SPI initialized: MOSI PA7 / MISO PA6 / SCK PA5.");
}

void loop()
{
  keepMotorOutputsDisabled();

  SerialUSB.println("--- TMC ALL SLOTS DIAG ---");
  for (const TmcSlot &slot : slots) {
    printSlot(slot);
  }

  delay(1000);
}
