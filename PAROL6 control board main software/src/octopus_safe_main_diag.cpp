#ifdef PAROL6_OCTOPUS_SAFE_MAIN

#include "octopus_safe_main_diag.h"

#include <Arduino.h>
#include <TMCStepper.h>
#include <ctype.h>
#include <string.h>

#include "constants.h"
#include "iodefs.h"
#include "octopus_commander_safety.h"
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
#include "octopus_joint_config.h"
#include "octopus_safe_homing_dryrun.h"
#include "octopus_safe_homing_executor.h"
#include "octopus_safe_homing_preflight.h"
#include "octopus_safe_limits.h"
#endif
#ifdef PAROL6_OCTOPUS_SAFE_JOINT_TEST
#include "octopus_safe_joint_test.h"
#endif
#include "octopus_safe_motion_gate.h"

static char commandBuffer[80];
static size_t commandLength = 0;

extern TMC5160Stepper driver[];

static const char *yesNo(bool value)
{
  return value ? "YES" : "NO";
}

static void printHex8(uint8_t value)
{
  SerialUSB.print("0x");
  if (value < 0x10) {
    SerialUSB.print("0");
  }
  SerialUSB.print(value, HEX);
}

static void printHex32(uint32_t value)
{
  SerialUSB.print("0x");
  for (int shift = 28; shift >= 0; shift -= 4) {
    SerialUSB.print((value >> shift) & 0x0F, HEX);
  }
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
  SerialUSB.println("motor_backend");
  SerialUSB.println("limits");
  SerialUSB.println("limits_raw");
  SerialUSB.println("limits_config");
  SerialUSB.println("limit_invert N on");
  SerialUSB.println("limit_invert N off");
  SerialUSB.println("limit_counts");
  SerialUSB.println("limit_reset_counts");
  SerialUSB.println("limit_safe");
  SerialUSB.println("homing_preflight_help");
  SerialUSB.println("homing_preflight_status");
  SerialUSB.println("homing_preflight_check N");
  SerialUSB.println("homing_preflight_all");
  SerialUSB.println("homing_set_max_travel N STEPS");
  SerialUSB.println("homing_set_dir N positive");
  SerialUSB.println("homing_set_dir N negative");
  SerialUSB.println("homing_mark_dir_validated N");
  SerialUSB.println("homing_clear_config N");
  SerialUSB.println("homing_preflight_safe");
  SerialUSB.println("homing_dryrun_help");
  SerialUSB.println("homing_dryrun_safe");
  SerialUSB.println("homing_dryrun_status");
  SerialUSB.println("homing_dryrun_reset");
  SerialUSB.println("homing_dryrun_config N MAX_TRAVEL LIMIT_AT BACKOFF");
  SerialUSB.println("homing_dryrun_run N");
  SerialUSB.println("homing_dryrun_all");
  SerialUSB.println("homing_exec_help");
  SerialUSB.println("homing_exec_safe");
  SerialUSB.println("homing_exec_status");
  SerialUSB.println("homing_exec_check N");
  SerialUSB.println("homing_exec_request N");
  SerialUSB.println("homing_exec_arm N");
  SerialUSB.println("homing_exec_run N");
  SerialUSB.println("homing_exec_cancel");
  SerialUSB.println("homing_exec_clear N");
  SerialUSB.println("homing_exec_mark_limit_validated N");
  SerialUSB.println("homing_exec_mark_polarity_validated N");
  SerialUSB.println("homing_exec_mark_dryrun_passed N");
  SerialUSB.println("homing_exec_policy");
  SerialUSB.println("joint_config_help");
  SerialUSB.println("joint_config_safe");
  SerialUSB.println("joint_config_status");
  SerialUSB.println("joint_config_show N");
  SerialUSB.println("joint_config_all");
  SerialUSB.println("joint_config_set_soft_limits N MIN_DEG MAX_DEG");
  SerialUSB.println("joint_config_set_home_offset N OFFSET_DEG");
  SerialUSB.println("joint_config_set_steps_per_deg N VALUE");
  SerialUSB.println("joint_config_set_dir_invert N on");
  SerialUSB.println("joint_config_set_dir_invert N off");
  SerialUSB.println("joint_config_set_homing_max_travel N STEPS");
  SerialUSB.println("joint_config_clear N");
  SerialUSB.println("motion_gate");
  SerialUSB.println("can_status");
  SerialUSB.println("tmc_status");
  SerialUSB.println("commander_status");
  SerialUSB.println("commander_last");
  SerialUSB.println("commander_counts");
  SerialUSB.println("commander_reset");
  SerialUSB.println("commander_policy");
#ifdef PAROL6_OCTOPUS_SAFE_JOINT_TEST
  SerialUSB.println("joint_test_help");
  SerialUSB.println("joint_test_status");
  SerialUSB.println("joint_select N");
  SerialUSB.println("joint_arm");
  SerialUSB.println("joint_disarm");
  SerialUSB.println("joint_enable");
  SerialUSB.println("joint_disable");
  SerialUSB.println("joint_step N");
  SerialUSB.println("joint_safe");
#endif
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
  SerialUSB.println("Homing preflight: available");
  SerialUSB.println("Homing dry-run: available");
  SerialUSB.println("Homing executor skeleton: available");
  SerialUSB.println("Joint config: available");
  SerialUSB.println("Calibration values require validation before real motion");
  SerialUSB.println("Real homing: still BLOCKED");
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

static void printMotorBackend()
{
  SerialUSB.println("Octopus motor backend: ACTIVE");
  SerialUSB.println("Joint1 -> MOTOR0 STEP=PF13 DIR=PF12 EN=PF14 CS=PC4");
  SerialUSB.println("Joint2 -> MOTOR1 STEP=PG0 DIR=PG1 EN=PF15 CS=PD11");
  SerialUSB.println("Joint3 -> MOTOR2 STEP=PF11 DIR=PG3 EN=PG5 CS=PC6");
  SerialUSB.println("Joint4 -> MOTOR3 STEP=PG4 DIR=PC1 EN=PA0 CS=PC7");
  SerialUSB.println("Joint5 -> MOTOR4 STEP=PF9 DIR=PF10 EN=PG2 CS=PF2");
  SerialUSB.println("Joint6 -> MOTOR5 STEP=PC13 DIR=PF0 EN=PF1 CS=PE4");
  SerialUSB.println("SPI: MOSI=PA7 MISO=PA6 SCK=PA5");
  SerialUSB.println("R_SENSE=0.075");
  SerialUSB.println("Important: MOTOR2_2 is not logical Joint4.");
}

static void printMotorEnables()
{
  SerialUSB.println("Motor enable pins:");
  SerialUSB.println("HIGH = disabled, LOW = enabled");
  SerialUSB.print("MOTOR0/J1 EN PF14: ");
  SerialUSB.println(digitalRead(GLOBAL_ENABLE) == HIGH ? "HIGH" : "LOW");
  SerialUSB.print("MOTOR1/J2 EN PF15: ");
  SerialUSB.println(digitalRead(ENABLE_M1) == HIGH ? "HIGH" : "LOW");
  SerialUSB.print("MOTOR2/J3 EN PG5: ");
  SerialUSB.println(digitalRead(ENABLE_M2) == HIGH ? "HIGH" : "LOW");
  SerialUSB.print("MOTOR3/J4 EN PA0: ");
  SerialUSB.println(digitalRead(ENABLE_M3) == HIGH ? "HIGH" : "LOW");
  SerialUSB.print("MOTOR4/J5 EN PG2: ");
  SerialUSB.println(digitalRead(ENABLE_M4) == HIGH ? "HIGH" : "LOW");
  SerialUSB.print("MOTOR5/J6 EN PF1: ");
  SerialUSB.println(digitalRead(ENABLE_M5) == HIGH ? "HIGH" : "LOW");
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
    SerialUSB.println(digitalRead(joints[i].LIMIT) == HIGH ? "HIGH" : "LOW");
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
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  octopusSafeLimitsHandleCommand("limits_brief");
#else
  SerialUSB.println("LIMITS=unavailable");
#endif
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
  static const char *jointNames[] = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  static const char *motorNames[] = {"MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4", "MOTOR5"};
  static const char *csNames[] = {"PC4", "PD11", "PC6", "PC7", "PF2", "PE4"};

  SerialUSB.println("--- TMC5160 READ-ONLY STATUS ---");
  SerialUSB.println("Mode: SAFE_MAIN read-only");
  SerialUSB.println("No motors enabled. No motion. No register writes.");
  SerialUSB.println("This command is read-only.");
  SerialUSB.println("It does not enable motors.");
  SerialUSB.println("It does not move the robot.");
  SerialUSB.println("Expected good TMC version for TMC5160 is usually 0x30.");
  SerialUSB.println("Expected good test_connection result is 0.");

  for (int i = 0; i < 6; ++i) {
    const uint8_t connection = driver[i].test_connection();
    const uint8_t version = driver[i].version();
    const uint8_t gstat = driver[i].GSTAT();
    const uint32_t drvStatus = driver[i].DRV_STATUS();
    const uint32_t gconf = driver[i].GCONF();
    const uint32_t chopconf = driver[i].CHOPCONF();

    SerialUSB.println();
    SerialUSB.print(jointNames[i]);
    SerialUSB.print(" / ");
    SerialUSB.print(motorNames[i]);
    SerialUSB.print(" CS=");
    SerialUSB.print(csNames[i]);
    SerialUSB.println(":");

    SerialUSB.print("  test_connection = ");
    SerialUSB.println(connection);
    SerialUSB.print("  version = ");
    printHex8(version);
    SerialUSB.println();
    SerialUSB.print("  GSTAT = ");
    printHex8(gstat);
    SerialUSB.println();
    SerialUSB.print("  DRV_STATUS = ");
    printHex32(drvStatus);
    SerialUSB.println();
    SerialUSB.print("  GCONF = ");
    printHex32(gconf);
    SerialUSB.println();
    SerialUSB.print("  CHOPCONF = ");
    printHex32(chopconf);
    SerialUSB.println();

    if (connection != 0) {
      SerialUSB.print("WARNING: TMC communication problem on ");
      SerialUSB.print(jointNames[i]);
      SerialUSB.print(" / ");
      SerialUSB.print(motorNames[i]);
      SerialUSB.println(".");
      SerialUSB.println("Check CS wiring/pin mapping/SPI/driver power.");
    }

    if (version != 0x30) {
      SerialUSB.println("WARNING: Unexpected TMC version.");
      SerialUSB.println("Expected TMC5160 version 0x30.");
    }
  }
}

static void processCommand(const char *command, Robot &robot, MotorStruct joints[], int jointCount)
{
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  if (octopusSafeLimitsHandleCommand(command)) {
    return;
  }
  if (octopusHomingPreflightHandleCommand(command)) {
    return;
  }
  if (octopusHomingDryRunHandleCommand(command)) {
    return;
  }
  if (octopusHomingExecutorHandleCommand(command)) {
    return;
  }
  if (octopusJointConfigHandleCommand(command)) {
    return;
  }
#endif

#ifdef PAROL6_OCTOPUS_SAFE_JOINT_TEST
  if (octopusSafeJointTestHandleCommand(command)) {
    return;
  }
#endif

  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "safe") == 0) {
    printSafe();
  } else if (strcmp(command, "status") == 0) {
    printStatus(robot, joints, jointCount);
  } else if (strcmp(command, "board") == 0) {
    printBoard();
  } else if (strcmp(command, "pins") == 0) {
    printPins();
  } else if (strcmp(command, "motor_backend") == 0) {
    printMotorBackend();
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
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  octopusHomingPreflightPrintStartup();
  octopusHomingDryRunPrintStartup();
  octopusHomingExecutorPrintStartup();
  octopusJointConfigPrintStartup();
#endif
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
