#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include "octopus_safe_homing_preflight.h"

#include <stdlib.h>
#include <string.h>

#include "octopus_safe_limits.h"

#ifndef PAROL6_OCTOPUS_PREFLIGHT_MAX_TRAVEL_STEPS
#define PAROL6_OCTOPUS_PREFLIGHT_MAX_TRAVEL_STEPS 20000L
#endif

static const uint8_t JOINT_COUNT = 6;
static const long MAX_TRAVEL_STEPS = PAROL6_OCTOPUS_PREFLIGHT_MAX_TRAVEL_STEPS;

enum HomingDirection {
  HOMING_DIR_UNCONFIGURED = 0,
  HOMING_DIR_POSITIVE,
  HOMING_DIR_NEGATIVE
};

struct HomingPreflightConfig {
  long maxTravelSteps;
  HomingDirection direction;
  bool directionValidated;
};

static HomingPreflightConfig configs[JOINT_COUNT] = {
    {0, HOMING_DIR_UNCONFIGURED, false},
    {0, HOMING_DIR_UNCONFIGURED, false},
    {0, HOMING_DIR_UNCONFIGURED, false},
    {0, HOMING_DIR_UNCONFIGURED, false},
    {0, HOMING_DIR_UNCONFIGURED, false},
    {0, HOMING_DIR_UNCONFIGURED, false},
};

static char lastReason[JOINT_COUNT][160] = {
    "not checked",
    "not checked",
    "not checked",
    "not checked",
    "not checked",
    "not checked",
};

static const char *motorNames[JOINT_COUNT] = {"MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4", "MOTOR5"};
static const char *stopNames[JOINT_COUNT] = {"Stop0", "Stop1", "Stop2", "Stop3", "Stop4", "Stop5"};
static const char *limitNames[JOINT_COUNT] = {"LIMIT1", "LIMIT2", "LIMIT3", "LIMIT4", "LIMIT5", "LIMIT6"};

static const char *levelName(bool state)
{
  return state ? "HIGH" : "LOW";
}

static const char *yesNo(bool value)
{
  return value ? "YES" : "NO";
}

static const char *onOff(bool value)
{
  return value ? "ON" : "OFF";
}

static const char *directionName(HomingDirection direction)
{
  switch (direction) {
  case HOMING_DIR_POSITIVE:
    return "positive";
  case HOMING_DIR_NEGATIVE:
    return "negative";
  default:
    return "not configured";
  }
}

static bool parseJointNumber(const char *text, uint8_t *jointIndex, const char **tail)
{
  char *end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (end == text || parsed < 1 || parsed > JOINT_COUNT) {
    return false;
  }

  *jointIndex = (uint8_t)(parsed - 1);
  if (tail != nullptr) {
    *tail = end;
  }
  return true;
}

static bool appendReason(char *buffer, size_t size, const char *reason)
{
  if (buffer[0] != '\0') {
    strncat(buffer, "; ", size - strlen(buffer) - 1);
  }
  strncat(buffer, reason, size - strlen(buffer) - 1);
  return false;
}

static bool safeModeBlocksHoming()
{
#if defined(PAROL6_OCTOPUS_SAFE_MAIN) && defined(PAROL6_SAFE_NO_MOTION) && defined(PAROL6_DISABLE_HOMING)
  return true;
#else
  return false;
#endif
}

static bool evaluatePreflight(uint8_t jointIndex)
{
  char reason[sizeof(lastReason[0])] = "";
  bool pass = true;

  if (jointIndex >= JOINT_COUNT) {
    return false;
  }

  if (!octopusSafeLimitsInitialized()) {
    pass = appendReason(reason, sizeof(reason), "safe limits not initialized");
  }

  if (!safeModeBlocksHoming()) {
    pass = appendReason(reason, sizeof(reason), "motion gate/homing block not active");
  }

  if (octopusSafeLimitsDebounceMs() == 0) {
    pass = appendReason(reason, sizeof(reason), "debounce not configured");
  }

  if (octopusSafeLimitsInitialized() && octopusSafeLimitsJointActive(jointIndex)) {
    pass = appendReason(reason, sizeof(reason), "limit is active");
  }

  if (configs[jointIndex].maxTravelSteps <= 0) {
    pass = appendReason(reason, sizeof(reason), "max travel not configured");
  }

  if (configs[jointIndex].direction == HOMING_DIR_UNCONFIGURED) {
    pass = appendReason(reason, sizeof(reason), "homing direction not configured");
  }

  if (!configs[jointIndex].directionValidated) {
    pass = appendReason(reason, sizeof(reason), "direction not validated");
  }

  if (pass) {
    strncpy(lastReason[jointIndex], "preflight checks satisfied", sizeof(lastReason[jointIndex]) - 1);
  } else {
    strncpy(lastReason[jointIndex], reason, sizeof(lastReason[jointIndex]) - 1);
  }
  lastReason[jointIndex][sizeof(lastReason[jointIndex]) - 1] = '\0';
  return pass;
}

static void printHelp()
{
  SerialUSB.println("Safe homing preflight commands:");
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
}

static void printSafe()
{
  SerialUSB.println("SAFE HOMING PREFLIGHT MODE");
  SerialUSB.println("This does not move the robot.");
  SerialUSB.println("This does not enable motors.");
  SerialUSB.println("This does not run homing.");
  SerialUSB.println("It only checks whether future homing could be allowed.");
  SerialUSB.println("Limits, polarity, direction, max travel, and hardware must be validated before homing.");
}

static void printJointPreflight(Stream &out, uint8_t jointIndex)
{
  const bool pass = evaluatePreflight(jointIndex);

  out.print("Joint");
  out.print(jointIndex + 1);
  out.print(" / ");
  out.print(motorNames[jointIndex]);
  out.print(" / ");
  out.print(stopNames[jointIndex]);
  out.print(": ");
  out.print(pass ? "PASS" : "FAIL");
  out.print(" - ");
  out.print(lastReason[jointIndex]);
  out.print(" | limit=");
  out.print(limitNames[jointIndex]);
  out.print(" raw=");
  out.print(octopusSafeLimitsInitialized() ? levelName(octopusSafeLimitsJointRaw(jointIndex)) : "UNKNOWN");
  out.print(" debounced=");
  out.print(octopusSafeLimitsInitialized() ? levelName(octopusSafeLimitsJointDebounced(jointIndex)) : "UNKNOWN");
  out.print(" active=");
  out.print(octopusSafeLimitsInitialized() ? yesNo(octopusSafeLimitsJointActive(jointIndex)) : "UNKNOWN");
  out.print(" invert=");
  out.print(octopusSafeLimitsInitialized() ? onOff(octopusSafeLimitsJointInvert(jointIndex)) : "UNKNOWN");
  out.print(" max_travel=");
  if (configs[jointIndex].maxTravelSteps > 0) {
    out.print(configs[jointIndex].maxTravelSteps);
  } else {
    out.print("not configured");
  }
  out.print(" dir=");
  out.print(directionName(configs[jointIndex].direction));
  out.print(" dir_validated=");
  out.println(yesNo(configs[jointIndex].directionValidated));
}

static void printStatus()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    printJointPreflight(SerialUSB, i);
  }
}

static bool parseMaxTravelCommand(const char *command)
{
  static const char prefix[] = "homing_set_max_travel ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: homing_set_max_travel requires N in range 1..6 and positive STEPS.");
    return true;
  }

  while (*tail == ' ') {
    ++tail;
  }

  char *end = nullptr;
  const long steps = strtol(tail, &end, 10);
  if (end == tail || *end != '\0' || steps <= 0) {
    SerialUSB.println("ERROR: homing_set_max_travel requires positive STEPS.");
    return true;
  }

  if (steps > MAX_TRAVEL_STEPS) {
    SerialUSB.print("Refusing homing_set_max_travel: STEPS exceeds safe limit ");
    SerialUSB.print(MAX_TRAVEL_STEPS);
    SerialUSB.println(".");
    return true;
  }

  octopusHomingPreflightSetMaxTravel(jointIndex, steps);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.print(" max_travel_steps=");
  SerialUSB.print(steps);
  SerialUSB.println(" (RAM-only)");
  return true;
}

static bool parseDirectionCommand(const char *command)
{
  static const char prefix[] = "homing_set_dir ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: homing_set_dir requires N in range 1..6 and positive/negative.");
    return true;
  }

  while (*tail == ' ') {
    ++tail;
  }

  if (strcmp(tail, "positive") == 0) {
    configs[jointIndex].direction = HOMING_DIR_POSITIVE;
  } else if (strcmp(tail, "negative") == 0) {
    configs[jointIndex].direction = HOMING_DIR_NEGATIVE;
  } else {
    SerialUSB.println("ERROR: homing_set_dir requires positive or negative.");
    return true;
  }

  configs[jointIndex].directionValidated = false;
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.print(" homing_direction=");
  SerialUSB.print(directionName(configs[jointIndex].direction));
  SerialUSB.println(" (RAM-only, not validated)");
  return true;
}

static bool parseSingleJointCommand(const char *command, const char *prefix, uint8_t *jointIndex)
{
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, jointIndex, &tail) || *tail != '\0') {
    SerialUSB.print("ERROR: ");
    SerialUSB.print(prefix);
    SerialUSB.println("requires N in range 1..6.");
    *jointIndex = JOINT_COUNT;
  }
  return true;
}

static void markDirectionValidated(uint8_t jointIndex)
{
  if (configs[jointIndex].direction == HOMING_DIR_UNCONFIGURED) {
    SerialUSB.println("ERROR: configure homing direction before marking it validated.");
    return;
  }

  configs[jointIndex].directionValidated = true;
  SerialUSB.println("Direction validation must only be marked after physical joint direction was checked safely.");
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" direction_validated=YES (RAM-only)");
}

static void clearConfig(uint8_t jointIndex)
{
  configs[jointIndex].maxTravelSteps = 0;
  configs[jointIndex].direction = HOMING_DIR_UNCONFIGURED;
  configs[jointIndex].directionValidated = false;
  strncpy(lastReason[jointIndex], "not checked", sizeof(lastReason[jointIndex]) - 1);
  lastReason[jointIndex][sizeof(lastReason[jointIndex]) - 1] = '\0';

  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" homing preflight config cleared.");
}

bool octopusHomingPreflightHandleCommand(const char *command)
{
  uint8_t jointIndex = 0;

  if (strcmp(command, "homing_preflight_help") == 0) {
    printHelp();
  } else if (strcmp(command, "homing_preflight_safe") == 0) {
    printSafe();
  } else if (strcmp(command, "homing_preflight_status") == 0 || strcmp(command, "homing_preflight_all") == 0) {
    printStatus();
  } else if (parseSingleJointCommand(command, "homing_preflight_check ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      printJointPreflight(SerialUSB, jointIndex);
    }
  } else if (parseMaxTravelCommand(command)) {
  } else if (parseDirectionCommand(command)) {
  } else if (parseSingleJointCommand(command, "homing_mark_dir_validated ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      markDirectionValidated(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_clear_config ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      clearConfig(jointIndex);
    }
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

bool octopusHomingPreflightPass(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return false;
  }
  return evaluatePreflight(jointIndex);
}

const char *octopusHomingPreflightLastReason(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return "invalid joint";
  }
  return lastReason[jointIndex];
}

void octopusPrintHomingPreflight(Stream &out, uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    out.println("FAIL - invalid joint");
    return;
  }
  printJointPreflight(out, jointIndex);
}

void octopusHomingPreflightPrintStartup()
{
  SerialUSB.println("Homing preflight: available");
  SerialUSB.println("Real homing: still BLOCKED");
  SerialUSB.flush();
}

long octopusHomingPreflightMaxTravel(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return 0;
  }
  return configs[jointIndex].maxTravelSteps;
}

bool octopusHomingPreflightSetMaxTravel(uint8_t jointIndex, long steps)
{
  if (jointIndex >= JOINT_COUNT || steps < 0 || steps > MAX_TRAVEL_STEPS) {
    return false;
  }
  configs[jointIndex].maxTravelSteps = steps;
  return true;
}

const char *octopusHomingPreflightDirectionName(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return "invalid joint";
  }
  return directionName(configs[jointIndex].direction);
}

bool octopusHomingPreflightDirectionValidated(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return false;
  }
  return configs[jointIndex].directionValidated;
}

#endif
