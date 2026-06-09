#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>

#include "iodefs.h"

static TMC5160Stepper motor0_driver(SELECT1, R_SENSE);

static void keepMotorOutputsDisabled()
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
}

static void printHex32(const char *name, uint32_t value)
{
  SerialUSB.print(name);
  SerialUSB.print(" = 0x");
  SerialUSB.println(value, HEX);
}

void setup()
{
  keepMotorOutputsDisabled();

  SerialUSB.begin(115200);
  delay(1500);

  SerialUSB.println();
  SerialUSB.println("TMC DIAG START");
  SerialUSB.println("Diagnostic firmware: no movement, no step pulses, motor enable stays HIGH.");

  pinMode(SELECT1, OUTPUT);
  digitalWrite(SELECT1, HIGH);

  SPI.setMOSI(MOSI);
  SPI.setMISO(MISO);
  SPI.setSCLK(SCK);
  SPI.begin();

  motor0_driver.setSPISpeed(2000000);

  SerialUSB.println("SPI initialized for MOTOR0 register reads.");
}

void loop()
{
  keepMotorOutputsDisabled();

  SerialUSB.println("--- TMC5160 MOTOR0 DIAG ---");
  SerialUSB.println("CS = SELECT1 / PC4");
  SerialUSB.println("SPI = MOSI PA7 / MISO PA6 / SCK PA5");
  SerialUSB.println("EN = GLOBAL_ENABLE / PF14 = HIGH disabled");
  SerialUSB.println("Other EN pins ENABLE_M1..ENABLE_M5 = HIGH disabled");
  SerialUSB.print("R_SENSE = ");
  SerialUSB.println(R_SENSE, 3);

  const uint8_t connection = motor0_driver.test_connection();
  SerialUSB.print("test_connection = ");
  SerialUSB.println(connection);

  printHex32("GSTAT", motor0_driver.GSTAT());
  printHex32("DRV_STATUS", motor0_driver.DRV_STATUS());
  printHex32("IOIN", motor0_driver.IOIN());
  printHex32("GCONF", motor0_driver.GCONF());
  printHex32("CHOPCONF", motor0_driver.CHOPCONF());

  SerialUSB.print("version = 0x");
  SerialUSB.println(motor0_driver.version(), HEX);

  delay(1000);
}
