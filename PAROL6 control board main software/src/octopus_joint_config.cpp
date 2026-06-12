#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include "octopus_joint_config.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "constants.h"
#include "octopus_safe_homing_preflight.h"

#ifndef PAROL6_OCTOPUS_JOINT_CONFIG_MAX_HOMING_TRAVEL_STEPS
#define PAROL6_OCTOPUS_JOINT_CONFIG_MAX_HOMING_TRAVEL_STEPS 20000L
#endif

#ifndef PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS
#define PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS 200L
#endif

static const uint8_t JOINT_COUNT = 6;
static const long MAX_HOMING_TRAVEL_STEPS = PAROL6_OCTOPUS_JOINT_CONFIG_MAX_HOMING_TRAVEL_STEPS;

static const OctopusJointConfig defaultConfigs[JOINT_COUNT] = {
    {1, "MOTOR0", "Stop0", "LIMIT1", 0.0f, true, 0.0f, 0.0f, 0.0f, 50000.0f, 1000.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR1_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
    {2, "MOTOR1", "Stop1", "LIMIT2", 0.0f, false, 0.0f, 0.0f, 0.0f, 50000.0f, 1000.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR2_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
    {3, "MOTOR2", "Stop2", "LIMIT3", 0.0f, true, 0.0f, 0.0f, 0.0f, 50000.0f, 500.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR3_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
    {4, "MOTOR3", "Stop3", "LIMIT4", 0.0f, false, 0.0f, 0.0f, 0.0f, 50000.0f, 500.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR4_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
    {5, "MOTOR4", "Stop4", "LIMIT5", 0.0f, false, 0.0f, 0.0f, 0.0f, 50000.0f, 500.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR5_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
    {6, "MOTOR5", "Stop5", "LIMIT6", 0.0f, true, 0.0f, 0.0f, 0.0f, 50000.0f, 100.0f, 0,
     PAROL6_OCTOPUS_JOINT_CONFIG_SAFE_TEST_MAX_STEPS, MOTOR6_MAX_CURRENT, PLACEHOLDER_NOT_CONFIRMED,
     PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED, ORIGINAL_PAROL6_CONFIRMED, PLACEHOLDER_NOT_CONFIRMED, OCTOPUS_PORT_CONFIRMED,
     ORIGINAL_PAROL6_CONFIRMED},
};

static OctopusJointConfig configs[JOINT_COUNT];
static bool initialized = false;

static void ensureInitialized()
{
  if (initialized) {
    return;
  }

  memcpy(configs, defaultConfigs, sizeof(configs));
  initialized = true;
}

const char *octopusJointConfigSourceName(OctopusJointConfigSource source)
{
  switch (source) {
  case ORIGINAL_PAROL6_CONFIRMED:
    return "ORIGINAL_PAROL6_CONFIRMED";
  case OCTOPUS_PORT_CONFIRMED:
    return "OCTOPUS_PORT_CONFIRMED";
  case PLACEHOLDER_NOT_CONFIRMED:
  default:
    return "PLACEHOLDER_NOT_CONFIRMED";
  }
}

static const char *onOff(bool value)
{
  return value ? "ON" : "OFF";
}

static OctopusJointConfigSource aggregateSource(const OctopusJointConfig &config)
{
  if (config.stepsPerDegreeSource == PLACEHOLDER_NOT_CONFIRMED || config.softLimitsSource == PLACEHOLDER_NOT_CONFIRMED ||
      config.homeOffsetSource == PLACEHOLDER_NOT_CONFIRMED || config.homingMaxTravelSource == PLACEHOLDER_NOT_CONFIRMED) {
    return PLACEHOLDER_NOT_CONFIRMED;
  }
  if (config.directionInvertSource == OCTOPUS_PORT_CONFIRMED || config.safeJointTestMaxStepSource == OCTOPUS_PORT_CONFIRMED) {
    return OCTOPUS_PORT_CONFIRMED;
  }
  return ORIGINAL_PAROL6_CONFIRMED;
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

static bool parseSingleJointCommand(const char *command, const char *prefix, uint8_t *jointIndex)
{
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, jointIndex, &tail) || *tail != '\0') {
    SerialUSB.println("ERROR: command requires N in range 1..6.");
    *jointIndex = JOINT_COUNT;
  }
  return true;
}

static bool parseFloat(const char *text, float *value, const char **tail)
{
  char *end = nullptr;
  const float parsed = strtof(text, &end);
  if (end == text || !isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  if (tail != nullptr) {
    *tail = end;
  }
  return true;
}

static void skipSpaces(const char **text)
{
  while (**text == ' ') {
    ++(*text);
  }
}

static void printFloatValue(float value)
{
  SerialUSB.print(value, 3);
}

static void printConfigRow(const OctopusJointConfig &config)
{
  SerialUSB.print("J");
  SerialUSB.print(config.jointNumber);
  SerialUSB.print("     ");
  SerialUSB.print(config.motorConnector);
  SerialUSB.print("  ");
  SerialUSB.print(config.limitInput);
  SerialUSB.print("   ");
  if (config.stepsPerDegree > 0.0f) {
    printFloatValue(config.stepsPerDegree);
  } else {
    SerialUSB.print("not_configured");
  }
  SerialUSB.print("   ");
  SerialUSB.print(onOff(config.directionInverted));
  SerialUSB.print("      ");
  if (config.softLimitsSource == PLACEHOLDER_NOT_CONFIRMED) {
    SerialUSB.print("not_configured");
    SerialUSB.print(" ");
    SerialUSB.print("not_configured");
  } else {
    printFloatValue(config.softMinDeg);
    SerialUSB.print(" ");
    printFloatValue(config.softMaxDeg);
  }
  SerialUSB.print("      ");
  printFloatValue(config.homeOffsetDeg);
  SerialUSB.print("          ");
  if (config.homingMaxTravelSteps > 0) {
    SerialUSB.print(config.homingMaxTravelSteps);
  } else {
    SerialUSB.print("not_configured");
  }
  SerialUSB.print("               ");
  SerialUSB.println(octopusJointConfigSourceName(config.configSource));
}

static void printStatus()
{
  ensureInitialized();
  SerialUSB.println("Joint  Motor   Limit   Steps/deg   DirInv   MinDeg   MaxDeg   HomeOffset   HomingMaxTravel   Source");
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    configs[i].configSource = aggregateSource(configs[i]);
    printConfigRow(configs[i]);
  }
}

static void printFieldSource(const char *name, OctopusJointConfigSource source)
{
  SerialUSB.print(name);
  SerialUSB.print("_source=");
  SerialUSB.println(octopusJointConfigSourceName(source));
}

static void printConfigDetail(uint8_t jointIndex)
{
  ensureInitialized();
  if (jointIndex >= JOINT_COUNT) {
    SerialUSB.println("ERROR: joint index out of range.");
    return;
  }

  const OctopusJointConfig &config = configs[jointIndex];
  SerialUSB.print("Joint");
  SerialUSB.println(config.jointNumber);
  SerialUSB.print("motor_connector=");
  SerialUSB.println(config.motorConnector);
  SerialUSB.println("motor_connector_source=OCTOPUS_PORT_CONFIRMED");
  SerialUSB.print("limit_input=");
  SerialUSB.print(config.limitInput);
  SerialUSB.print(" / ");
  SerialUSB.println(config.limitAlias);
  SerialUSB.println("limit_input_source=OCTOPUS_PORT_CONFIRMED");
  SerialUSB.print("steps_per_degree=");
  if (config.stepsPerDegree > 0.0f) {
    printFloatValue(config.stepsPerDegree);
    SerialUSB.println();
  } else {
    SerialUSB.println("not configured");
  }
  printFieldSource("steps_per_degree", config.stepsPerDegreeSource);
  SerialUSB.print("direction_inverted=");
  SerialUSB.println(onOff(config.directionInverted));
  printFieldSource("direction_inverted", config.directionInvertSource);
  SerialUSB.print("soft_min_deg=");
  printFloatValue(config.softMinDeg);
  SerialUSB.println();
  SerialUSB.print("soft_max_deg=");
  printFloatValue(config.softMaxDeg);
  SerialUSB.println();
  printFieldSource("soft_limits", config.softLimitsSource);
  SerialUSB.print("home_offset_deg=");
  printFloatValue(config.homeOffsetDeg);
  SerialUSB.println();
  printFieldSource("home_offset", config.homeOffsetSource);
  SerialUSB.print("max_speed=");
  printFloatValue(config.maxSpeed);
  SerialUSB.println();
  printFieldSource("max_speed", config.maxSpeedSource);
  SerialUSB.print("max_acceleration=");
  printFloatValue(config.maxAcceleration);
  SerialUSB.println();
  printFieldSource("max_acceleration", config.maxAccelerationSource);
  SerialUSB.print("homing_max_travel_steps=");
  SerialUSB.println(config.homingMaxTravelSteps > 0 ? config.homingMaxTravelSteps : 0);
  printFieldSource("homing_max_travel_steps", config.homingMaxTravelSource);
  SerialUSB.print("safe_joint_test_max_step_count=");
  SerialUSB.println(config.safeJointTestMaxStepCount);
  printFieldSource("safe_joint_test_max_step_count", config.safeJointTestMaxStepSource);
  SerialUSB.print("tmc_current_ma=");
  SerialUSB.println(config.tmcCurrentMa);
  printFieldSource("tmc_current_ma", config.tmcCurrentSource);
  SerialUSB.print("overall_source=");
  SerialUSB.println(octopusJointConfigSourceName(aggregateSource(config)));
}

static void printHelp()
{
  SerialUSB.println("Octopus joint config commands:");
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
}

static void printSafe()
{
  SerialUSB.println("JOINT CONFIG SAFE MODE");
  SerialUSB.println("This does not move the robot.");
  SerialUSB.println("This does not enable motors.");
  SerialUSB.println("This does not run homing.");
  SerialUSB.println("Configuration is RAM-only.");
  SerialUSB.println("Values must be physically validated before real homing or full motion.");
}

static bool parseSoftLimitsCommand(const char *command)
{
  static const char prefix[] = "joint_config_set_soft_limits ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_soft_limits requires N in range 1..6 and MIN_DEG MAX_DEG.");
    return true;
  }
  skipSpaces(&tail);

  float minDeg = 0.0f;
  float maxDeg = 0.0f;
  if (!parseFloat(tail, &minDeg, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_soft_limits requires numeric MIN_DEG MAX_DEG.");
    return true;
  }
  skipSpaces(&tail);
  if (!parseFloat(tail, &maxDeg, &tail) || *tail != '\0') {
    SerialUSB.println("ERROR: joint_config_set_soft_limits requires numeric MIN_DEG MAX_DEG.");
    return true;
  }
  if (minDeg >= maxDeg) {
    SerialUSB.println("ERROR: MIN_DEG must be less than MAX_DEG.");
    return true;
  }
  if (minDeg < -360.0f || maxDeg > 360.0f) {
    SerialUSB.println("ERROR: soft limit range must remain within -360..360 degrees.");
    return true;
  }

  ensureInitialized();
  configs[jointIndex].softMinDeg = minDeg;
  configs[jointIndex].softMaxDeg = maxDeg;
  configs[jointIndex].softLimitsSource = PLACEHOLDER_NOT_CONFIRMED;
  configs[jointIndex].configSource = aggregateSource(configs[jointIndex]);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" soft limits updated (RAM-only, not physically validated).");
  return true;
}

static bool parseHomeOffsetCommand(const char *command)
{
  static const char prefix[] = "joint_config_set_home_offset ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_home_offset requires N in range 1..6 and OFFSET_DEG.");
    return true;
  }
  skipSpaces(&tail);

  float offsetDeg = 0.0f;
  if (!parseFloat(tail, &offsetDeg, &tail) || *tail != '\0') {
    SerialUSB.println("ERROR: joint_config_set_home_offset requires numeric OFFSET_DEG.");
    return true;
  }
  if (offsetDeg < -360.0f || offsetDeg > 360.0f) {
    SerialUSB.println("ERROR: home offset must remain within -360..360 degrees.");
    return true;
  }

  ensureInitialized();
  configs[jointIndex].homeOffsetDeg = offsetDeg;
  configs[jointIndex].homeOffsetSource = PLACEHOLDER_NOT_CONFIRMED;
  configs[jointIndex].configSource = aggregateSource(configs[jointIndex]);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" home offset updated (RAM-only, not applied to position or homing).");
  return true;
}

static bool parseStepsPerDegreeCommand(const char *command)
{
  static const char prefix[] = "joint_config_set_steps_per_deg ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_steps_per_deg requires N in range 1..6 and VALUE.");
    return true;
  }
  skipSpaces(&tail);

  float value = 0.0f;
  if (!parseFloat(tail, &value, &tail) || *tail != '\0') {
    SerialUSB.println("ERROR: joint_config_set_steps_per_deg requires numeric VALUE.");
    return true;
  }
  if (value <= 0.0f) {
    SerialUSB.println("ERROR: VALUE must be positive.");
    return true;
  }

  ensureInitialized();
  configs[jointIndex].stepsPerDegree = value;
  configs[jointIndex].stepsPerDegreeSource = PLACEHOLDER_NOT_CONFIRMED;
  configs[jointIndex].configSource = aggregateSource(configs[jointIndex]);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" steps_per_degree updated (RAM-only, not physically validated).");
  if (value < 1.0f || value > 10000.0f) {
    SerialUSB.println("WARNING: steps_per_degree looks unusual; validate mechanically before use.");
  }
  return true;
}

static bool parseDirInvertCommand(const char *command)
{
  static const char prefix[] = "joint_config_set_dir_invert ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_dir_invert requires N in range 1..6 and on/off.");
    return true;
  }
  skipSpaces(&tail);

  bool inverted = false;
  if (strcmp(tail, "on") == 0) {
    inverted = true;
  } else if (strcmp(tail, "off") == 0) {
    inverted = false;
  } else {
    SerialUSB.println("ERROR: joint_config_set_dir_invert requires on or off.");
    return true;
  }

  ensureInitialized();
  configs[jointIndex].directionInverted = inverted;
  configs[jointIndex].directionInvertSource = PLACEHOLDER_NOT_CONFIRMED;
  configs[jointIndex].configSource = aggregateSource(configs[jointIndex]);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.print(" dir_invert=");
  SerialUSB.print(onOff(inverted));
  SerialUSB.println(" (RAM-only)");
  SerialUSB.println("Direction inversion must be validated with safe single-joint testing before homing.");
  return true;
}

static bool parseHomingMaxTravelCommand(const char *command)
{
  static const char prefix[] = "joint_config_set_homing_max_travel ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("ERROR: joint_config_set_homing_max_travel requires N in range 1..6 and STEPS.");
    return true;
  }
  skipSpaces(&tail);

  char *end = nullptr;
  const long steps = strtol(tail, &end, 10);
  if (end == tail || *end != '\0' || steps <= 0) {
    SerialUSB.println("ERROR: STEPS must be positive.");
    return true;
  }
  if (steps > MAX_HOMING_TRAVEL_STEPS) {
    SerialUSB.print("Refusing homing max travel: STEPS exceeds safe limit ");
    SerialUSB.print(MAX_HOMING_TRAVEL_STEPS);
    SerialUSB.println(".");
    return true;
  }

  ensureInitialized();
  configs[jointIndex].homingMaxTravelSteps = steps;
  configs[jointIndex].homingMaxTravelSource = PLACEHOLDER_NOT_CONFIRMED;
  configs[jointIndex].configSource = aggregateSource(configs[jointIndex]);
  octopusHomingPreflightSetMaxTravel(jointIndex, steps);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" homing max travel updated (RAM-only) and synced to homing preflight.");
  return true;
}

static void clearConfig(uint8_t jointIndex)
{
  ensureInitialized();
  if (jointIndex >= JOINT_COUNT) {
    return;
  }

  configs[jointIndex] = defaultConfigs[jointIndex];
  octopusHomingPreflightSetMaxTravel(jointIndex, 0);
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" joint config RAM overrides cleared.");
}

bool octopusJointConfigHandleCommand(const char *command)
{
  uint8_t jointIndex = 0;

  if (strcmp(command, "joint_config_help") == 0) {
    printHelp();
  } else if (strcmp(command, "joint_config_safe") == 0) {
    printSafe();
  } else if (strcmp(command, "joint_config_status") == 0) {
    printStatus();
  } else if (strcmp(command, "joint_config_all") == 0) {
    ensureInitialized();
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
      printConfigDetail(i);
    }
  } else if (parseSingleJointCommand(command, "joint_config_show ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      printConfigDetail(jointIndex);
    }
  } else if (parseSoftLimitsCommand(command)) {
  } else if (parseHomeOffsetCommand(command)) {
  } else if (parseStepsPerDegreeCommand(command)) {
  } else if (parseDirInvertCommand(command)) {
  } else if (parseHomingMaxTravelCommand(command)) {
  } else if (parseSingleJointCommand(command, "joint_config_clear ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      clearConfig(jointIndex);
    }
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

void octopusJointConfigPrintStartup()
{
  SerialUSB.println("Joint config: available");
  SerialUSB.println("Calibration values require validation before real motion");
  SerialUSB.flush();
}

const OctopusJointConfig *octopusJointConfigGet(uint8_t jointIndex)
{
  ensureInitialized();
  if (jointIndex >= JOINT_COUNT) {
    return nullptr;
  }
  return &configs[jointIndex];
}

bool octopusJointConfigIsValid(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  if (config == nullptr) {
    return false;
  }
  return config->stepsPerDegree > 0.0f && config->softMinDeg < config->softMaxDeg && config->homingMaxTravelSteps > 0 &&
         config->stepsPerDegreeSource != PLACEHOLDER_NOT_CONFIRMED && config->softLimitsSource != PLACEHOLDER_NOT_CONFIRMED &&
         config->homeOffsetSource != PLACEHOLDER_NOT_CONFIRMED && config->homingMaxTravelSource != PLACEHOLDER_NOT_CONFIRMED;
}

float octopusJointConfigStepsPerDegree(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr ? config->stepsPerDegree : 0.0f;
}

bool octopusJointConfigDirInverted(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr && config->directionInverted;
}

long octopusJointConfigHomingMaxTravelSteps(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr ? config->homingMaxTravelSteps : 0;
}

float octopusJointConfigMinDeg(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr ? config->softMinDeg : 0.0f;
}

float octopusJointConfigMaxDeg(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr ? config->softMaxDeg : 0.0f;
}

float octopusJointConfigHomeOffsetDeg(uint8_t jointIndex)
{
  const OctopusJointConfig *config = octopusJointConfigGet(jointIndex);
  return config != nullptr ? config->homeOffsetDeg : 0.0f;
}

#endif
