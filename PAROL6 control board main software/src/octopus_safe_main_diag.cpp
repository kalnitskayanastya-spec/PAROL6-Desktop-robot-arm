#ifdef PAROL6_OCTOPUS_SAFE_MAIN

#include "octopus_safe_main_diag.h"

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

#include "constants.h"
#include "iodefs.h"
#include "octopus_commander_safety.h"
#include "octopus_safe_motion_gate.h"

static char commandBuffer[80];
static size_t commandLength = 0;

static const char *levelName(int value)
{
  return value == HIGH ? "HIGH" : "LOW";
}

static const char *yesNo(bool value)
{
  return value ? "YES" : "NO";
}

static char levelBit(int pin)
{
  return digitalRead(pin) == HIGH ? '1' : '0';
}

static bool isShellByte(int value)
{
  return value == '\r' || value == '\n' || value == '\b' || value == 127 || (value >= 32 && value <= 126);
}

static bool isBlankCommand(const char *command)
{
  while (*command != '\0') {
    if (!isspace((unsigned char)*command)) {
      return false;
    }
    ++command;
  }
  return true;
}

static void trimCommand(char *command)
{
  char *start = command;
  while (*start != '\0' && isspace((unsigned char)*start)) {
    ++start;
  }

  if (start != command) {
    memmove(command, start, strlen(start) + 1);
  }

  size_t len = strlen(command);
  while (len > 0 && isspace((unsigned char)command[len - 1])) {
    command[len - 1] = '\0';
    --len;
  }
}

static void printHelp()
{
  SerialUSB.println("This is PAROL6 OCTOPUS SAFE MAIN.");
  SerialUSB.println("Read-only diagnostic shell.");
  SerialUSB.println("Motion/homing/motor enable are blocked.");
  SerialUSB.println();
  SerialUSB.println("Commands:");
  SerialUSB.println("help");
  SerialUSB.println("version");
  SerialUSB.println("safe");
  SerialUSB.println("status");
  SerialUSB.println("board");
  SerialUSB.println("pins");
  SerialUSB.println("limits");
  SerialUSB.println("motion_gate");
  SerialUSB.println("can_status");
  SerialUSB.println("tmc_status");
  SerialUSB.println("commander_status");
  SerialUSB.println("commander_last");
  SerialUSB.println("commander_counts");
  SerialUSB.println("commander_reset");
  SerialUSB.println("commander_policy");
}

static void printSafe()
{
  SerialUSB.println("SAFETY:");
  SerialUSB.println("- This safe main build must not move motors.");
  SerialUSB.println("- Motion gate is active.");
  SerialUSB.println("- Homing is blocked.");
  SerialUSB.println("- Full robot motion is blocked.");
  SerialUSB.println("- Do not connect 24V directly to MCU inputs.");
  SerialUSB.println("- Verify optocoupler outputs before connecting Stop inputs.");
  SerialUSB.println("- Use diagnostic envs for motor-slot and optocoupler tests.");
}

static void printMotionGate()
{
  SerialUSB.println("Motion gate: ACTIVE");
  SerialUSB.println("Homing commands: BLOCKED");
  SerialUSB.println("Motion commands: BLOCKED");
  SerialUSB.println("Motor enable from full firmware: BLOCKED");
  SerialUSB.println("Diagnostic motor_slot_test env is separate and unaffected.");
}

static void printCanStatus()
{
#ifdef PAROL6_CAN_TIMEOUT_MS
  SerialUSB.print("CAN timeout/fallback enabled: PAROL6_CAN_TIMEOUT_MS=");
  SerialUSB.println(PAROL6_CAN_TIMEOUT_MS);
#else
  SerialUSB.println("CAN timeout/fallback enabled: PAROL6_CAN_TIMEOUT_MS=default");
#endif
  SerialUSB.println("CAN runtime init status is not exported yet in safe main.");
}

static void printVersion()
{
  SerialUSB.println("--- PAROL6 OCTOPUS SAFE MAIN ---");
  SerialUSB.println("Firmware: octopus_parol6_main_safe_0000");
  SerialUSB.println("Board: BIGTREETECH Octopus Pro F446");
  SerialUSB.print("Build: ");
  SerialUSB.print(__DATE__);
  SerialUSB.print(" ");
  SerialUSB.println(__TIME__);
  SerialUSB.println("Baud: 115200");
  SerialUSB.println("Motion gate: ACTIVE");
  SerialUSB.println("Homing: BLOCKED");
  SerialUSB.println("Motion: BLOCKED");
  SerialUSB.println("CAN: timeout/fallback enabled");
}

static void printBoard()
{
  SerialUSB.println("Board profile: BIGTREETECH Octopus Pro F446");
  SerialUSB.println("MCU: STM32F446");
  SerialUSB.println("SPI: MOSI=PA7 MISO=PA6 SCK=PA5");
  SerialUSB.println("R_SENSE=0.075");
  SerialUSB.println("Physical motor connector order:");
  SerialUSB.println("MOTOR0 MOTOR1 MOTOR2 MOTOR2_2 MOTOR3 MOTOR4 MOTOR5 MOTOR6 MOTOR7");
  SerialUSB.println("Important: MOTOR2_2 is not logical Joint4. Logical Joint4 is MOTOR3.");
}

static void printPins()
{
  SerialUSB.println("Joint1 / MOTOR0 STEP=PF13 DIR=PF12 EN=PF14 CS=PC4");
  SerialUSB.println("Joint2 / MOTOR1 STEP=PG0 DIR=PG1 EN=PF15 CS=PD11");
  SerialUSB.println("Joint3 / MOTOR2 STEP=PF11 DIR=PG3 EN=PG5 CS=PC6");
  SerialUSB.println("Joint4 / MOTOR3 STEP=PG4 DIR=PC1 EN=PA0 CS=PC7");
  SerialUSB.println("Joint5 / MOTOR4 STEP=PF9 DIR=PF10 EN=PG2 CS=PF2");
  SerialUSB.println("Joint6 / MOTOR5 STEP=PC13 DIR=PF0 EN=PF1 CS=PE4");
}

static void printLimits()
{
  SerialUSB.print("Stop0 / LIMIT1 / Joint1 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT1)));
  SerialUSB.print("Stop1 / LIMIT2 / Joint2 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT2)));
  SerialUSB.print("Stop2 / LIMIT3 / Joint3 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT3)));
  SerialUSB.print("Stop3 / LIMIT4 / Joint4 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT4)));
  SerialUSB.print("Stop4 / LIMIT5 / Joint5 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT5)));
  SerialUSB.print("Stop5 / LIMIT6 / Joint6 = ");
  SerialUSB.println(levelName(digitalRead(LIMIT6)));
}

static void printLimitsBrief()
{
  SerialUSB.print("LIMITS=");
  SerialUSB.print(levelBit(LIMIT1));
  SerialUSB.print(levelBit(LIMIT2));
  SerialUSB.print(levelBit(LIMIT3));
  SerialUSB.print(levelBit(LIMIT4));
  SerialUSB.print(levelBit(LIMIT5));
  SerialUSB.println(levelBit(LIMIT6));
}

static void printMotorEnables()
{
  SerialUSB.println("Motor enable pins:");
  SerialUSB.println("HIGH = disabled, LOW = enabled");
  SerialUSB.print("MOTOR0/J1 EN PF14: ");
  SerialUSB.println(levelName(digitalRead(GLOBAL_ENABLE)));
  SerialUSB.print("MOTOR1/J2 EN PF15: ");
  SerialUSB.println(levelName(digitalRead(ENABLE_M1)));
  SerialUSB.print("MOTOR2/J3 EN PG5: ");
  SerialUSB.println(levelName(digitalRead(ENABLE_M2)));
  SerialUSB.print("MOTOR3/J4 EN PA0: ");
  SerialUSB.println(levelName(digitalRead(ENABLE_M3)));
  SerialUSB.print("MOTOR4/J5 EN PG2: ");
  SerialUSB.println(levelName(digitalRead(ENABLE_M4)));
  SerialUSB.print("MOTOR5/J6 EN PF1: ");
  SerialUSB.println(levelName(digitalRead(ENABLE_M5)));
}

static void printJointStatus(const MotorStruct joints[], int jointCount)
{
  SerialUSB.println("Joints:");
  SerialUSB.println("Joint  PosSteps  Speed  CmdPos  CmdVel  Homed  LimitRaw");
  for (int i = 0; i < jointCount; ++i) {
    SerialUSB.print("J");
    SerialUSB.print(i + 1);
    SerialUSB.print("     ");
    SerialUSB.print(joints[i].position);
    SerialUSB.print("         ");
    SerialUSB.print(joints[i].speed);
    SerialUSB.print("      ");
    SerialUSB.print(joints[i].commanded_position);
    SerialUSB.print("       ");
    SerialUSB.print(joints[i].commanded_velocity);
    SerialUSB.print("       ");
    SerialUSB.print(joints[i].homed);
    SerialUSB.print("      ");
    SerialUSB.println(levelName(digitalRead(joints[i].LIMIT)));
  }
}

static void printStatus(const Robot &robot, const MotorStruct joints[], int jointCount)
{
  SerialUSB.println("SAFE_MAIN=ON");
  SerialUSB.println("BOARD=OCTOPUS_PRO_F446");
  SerialUSB.println("MOTION_GATE=ACTIVE");
  SerialUSB.println("HOMING=BLOCKED");
  SerialUSB.println("MOTION=BLOCKED");
#ifdef PAROL6_CAN_TIMEOUT_MS
  SerialUSB.print("CAN=timeout/fallback enabled, timeout_ms=");
  SerialUSB.println(PAROL6_CAN_TIMEOUT_MS);
#else
  SerialUSB.println("CAN=timeout/fallback enabled");
#endif
  printLimitsBrief();
  SerialUSB.print("ROBOT_DISABLED=");
  SerialUSB.println(robot.disabled);
  SerialUSB.print("LAST_COMMAND=");
  SerialUSB.println(robot.command);
  SerialUSB.print("JOINTS_REPORTED=");
  SerialUSB.println(jointCount);
  (void)joints;
}

static void printTmcStatus()
{
  SerialUSB.println("TMC read-only status is not wired into safe main yet.");
  SerialUSB.println("Use octopus_parol6_motor_slot_test_0000 for TMC SPI diagnostic.");
}

static void processCommand(const char *command, Robot &robot, MotorStruct joints[], int jointCount)
{
  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "safe") == 0) {
    printSafe();
  } else if (strcmp(command, "status") == 0) {
    printStatus(robot, joints, jointCount);
  } else if (strcmp(command, "board") == 0) {
    printBoard();
  } else if (strcmp(command, "limits") == 0) {
    printLimits();
  } else if (strcmp(command, "pins") == 0) {
    printPins();
  } else if (strcmp(command, "motion_gate") == 0) {
    printMotionGate();
  } else if (strcmp(command, "can_status") == 0) {
    printCanStatus();
  } else if (strcmp(command, "tmc_status") == 0) {
    printTmcStatus();
  } else if (strcmp(command, "commander_status") == 0) {
    octopusPrintCommanderStatus();
  } else if (strcmp(command, "commander_last") == 0) {
    octopusPrintLastCommanderCommand();
  } else if (strcmp(command, "commander_counts") == 0) {
    octopusPrintCommanderStats();
  } else if (strcmp(command, "commander_reset") == 0) {
    octopusResetCommanderStats();
  } else if (strcmp(command, "commander_policy") == 0) {
    octopusPrintCommanderPolicy();
  } else if (strcmp(command, "version") == 0) {
    printVersion();
  } else {
    SerialUSB.print("Unknown command: ");
    SerialUSB.println(command);
    SerialUSB.println("Type help.");
  }

  SerialUSB.flush();
}

void octopusSafeMainDiagPrintStartupHint()
{
  SerialUSB.println("Type help for read-only diagnostic commands.");
  SerialUSB.println("Motion gate: ACTIVE");
  SerialUSB.flush();
}

bool octopusSafeMainDiagPoll(Robot &robot, MotorStruct joints[], int jointCount)
{
  bool consumed = false;

  while (SerialUSB.available() > 0) {
    const int nextByte = SerialUSB.peek();
    if (!isShellByte(nextByte)) {
      return consumed;
    }

    consumed = true;
    const char c = (char)SerialUSB.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      trimCommand(commandBuffer);
      if (!isBlankCommand(commandBuffer)) {
        processCommand(commandBuffer, robot, joints, jointCount);
      }
      commandLength = 0;
      commandBuffer[0] = '\0';
      continue;
    }

    if (c == '\b' || c == 127) {
      if (commandLength > 0) {
        --commandLength;
        commandBuffer[commandLength] = '\0';
      }
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
      commandBuffer[commandLength] = '\0';
    } else {
      commandLength = 0;
      commandBuffer[0] = '\0';
      SerialUSB.println("ERROR: command too long.");
      SerialUSB.println("Type help.");
      SerialUSB.flush();
    }
  }

  return consumed;
}

#endif
