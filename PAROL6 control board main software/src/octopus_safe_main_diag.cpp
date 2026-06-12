#ifdef PAROL6_OCTOPUS_SAFE_MAIN

#include "octopus_safe_main_diag.h"

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

#include "constants.h"
#include "iodefs.h"
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
  SerialUSB.println("Commands:");
  SerialUSB.println("help");
  SerialUSB.println("safe");
  SerialUSB.println("status");
  SerialUSB.println("motors");
  SerialUSB.println("limits");
  SerialUSB.println("pins");
  SerialUSB.println("slots");
  SerialUSB.println("gate");
  SerialUSB.println("can");
  SerialUSB.println("version");
  SerialUSB.println();
  SerialUSB.println("Read-only shell. Motion, homing, motor enable, and gripper commands are blocked.");
}

static void printSafe()
{
  SerialUSB.println("SAFETY:");
  SerialUSB.println("- Octopus safe main diagnostic shell is read-only.");
  SerialUSB.println("- No shell command enables motors.");
  SerialUSB.println("- No shell command generates step pulses.");
  SerialUSB.println("- Homing commands are blocked by motion gate.");
  SerialUSB.println("- Motion commands are blocked by motion gate.");
  SerialUSB.println("- Motor enable from full firmware is blocked by motion gate.");
  SerialUSB.println("- Nothing should be flashed from this shell.");
}

static void printGate()
{
  SerialUSB.println("Motion gate:");
  SerialUSB.print("active: ");
  SerialUSB.println(yesNo(!octopusSafeMotionEnabled()));
  SerialUSB.println("homing commands: BLOCKED");
  SerialUSB.println("motion commands: BLOCKED");
  SerialUSB.println("motor enable from full firmware: BLOCKED");
  SerialUSB.println("gripper CAN motion commands: BLOCKED");
}

static void printCan()
{
  SerialUSB.println("CAN:");
  SerialUSB.println("timeout/fallback: enabled");
  SerialUSB.print("timeout_ms: ");
#ifdef PAROL6_CAN_TIMEOUT_MS
  SerialUSB.println(PAROL6_CAN_TIMEOUT_MS);
#else
  SerialUSB.println("default");
#endif
}

static void printVersion()
{
  SerialUSB.println("Version:");
  SerialUSB.print("firmware VERSION: ");
  SerialUSB.println(VERSION);
  SerialUSB.println("env: octopus_parol6_main_safe_0000");
  SerialUSB.println("board: BIGTREETECH Octopus Pro F446");
}

static void printSlots()
{
  SerialUSB.println("Logical slot -> physical connector:");
  SerialUSB.println("slot 0 -> MOTOR0 / Joint1");
  SerialUSB.println("slot 1 -> MOTOR1 / Joint2");
  SerialUSB.println("slot 2 -> MOTOR2 / Joint3");
  SerialUSB.println("MOTOR2_2 -> extra physical connector / duplicate MOTOR2 output, NOT slot 3");
  SerialUSB.println("slot 3 -> MOTOR3 / Joint4, physically AFTER MOTOR2_2");
  SerialUSB.println("slot 4 -> MOTOR4 / Joint5");
  SerialUSB.println("slot 5 -> MOTOR5 / Joint6");
}

static void printPins()
{
  SerialUSB.println("Octopus Pro F446 pins:");
  SerialUSB.println("SPI: MOSI PA7, MISO PA6, SCK PA5");
  SerialUSB.print("R_SENSE: ");
  SerialUSB.println(R_SENSE, 3);
  SerialUSB.println("MOTOR0/J1: STEP PF13 DIR PF12 CS PC4 EN PF14");
  SerialUSB.println("MOTOR1/J2: STEP PG0 DIR PG1 CS PD11 EN PF15");
  SerialUSB.println("MOTOR2/J3: STEP PF11 DIR PG3 CS PC6 EN PG5");
  SerialUSB.println("MOTOR3/J4: STEP PG4 DIR PC1 CS PC7 EN PA0");
  SerialUSB.println("MOTOR4/J5: STEP PF9 DIR PF10 CS PF2 EN PG2");
  SerialUSB.println("MOTOR5/J6: STEP PC13 DIR PF0 CS PE4 EN PF1");
  SerialUSB.println("Stop0..Stop5: PG6 PG9 PG10 PG11 PG12 PG13");
}

static void printLimits()
{
  SerialUSB.println("Stop/LIMIT inputs:");
  SerialUSB.println("Input  Joint  PinName  Raw");
  SerialUSB.print("Stop0  J1     LIMIT1   ");
  SerialUSB.println(levelName(digitalRead(LIMIT1)));
  SerialUSB.print("Stop1  J2     LIMIT2   ");
  SerialUSB.println(levelName(digitalRead(LIMIT2)));
  SerialUSB.print("Stop2  J3     LIMIT3   ");
  SerialUSB.println(levelName(digitalRead(LIMIT3)));
  SerialUSB.print("Stop3  J4     LIMIT4   ");
  SerialUSB.println(levelName(digitalRead(LIMIT4)));
  SerialUSB.print("Stop4  J5     LIMIT5   ");
  SerialUSB.println(levelName(digitalRead(LIMIT5)));
  SerialUSB.print("Stop5  J6     LIMIT6   ");
  SerialUSB.println(levelName(digitalRead(LIMIT6)));
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
  SerialUSB.println("--- OCTOPUS SAFE MAIN STATUS ---");
  SerialUSB.println("mode: PAROL6_OCTOPUS_SAFE_MAIN");
  SerialUSB.print("motion_gate_active: ");
  SerialUSB.println(yesNo(!octopusSafeMotionEnabled()));
  SerialUSB.print("robot_disabled: ");
  SerialUSB.println(robot.disabled);
  SerialUSB.print("last_command: ");
  SerialUSB.println(robot.command);
  SerialUSB.print("timeout_ms_commanded: ");
  SerialUSB.println(robot.Timeout);
  SerialUSB.print("timeout_error: ");
  SerialUSB.println(robot.timeout_error);
  SerialUSB.print("estop_raw: ");
  SerialUSB.println(levelName(digitalRead(ESTOP)));
  SerialUSB.print("input1_raw: ");
  SerialUSB.println(levelName(digitalRead(INPUT1)));
  SerialUSB.print("input2_raw: ");
  SerialUSB.println(levelName(digitalRead(INPUT2)));
  printMotorEnables();
  printJointStatus(joints, jointCount);
}

static void processCommand(const char *command, Robot &robot, MotorStruct joints[], int jointCount)
{
  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "safe") == 0) {
    printSafe();
  } else if (strcmp(command, "status") == 0) {
    printStatus(robot, joints, jointCount);
  } else if (strcmp(command, "motors") == 0) {
    printMotorEnables();
    printJointStatus(joints, jointCount);
  } else if (strcmp(command, "limits") == 0) {
    printLimits();
  } else if (strcmp(command, "pins") == 0) {
    printPins();
  } else if (strcmp(command, "slots") == 0) {
    printSlots();
  } else if (strcmp(command, "gate") == 0) {
    printGate();
  } else if (strcmp(command, "can") == 0) {
    printCan();
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
  SerialUSB.println("Safe main diagnostic shell: type help.");
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
