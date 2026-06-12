#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include "octopus_safe_homing_dryrun.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "octopus_safe_homing_preflight.h"
#include "octopus_safe_limits.h"

#ifndef PAROL6_OCTOPUS_DRYRUN_MAX_TRAVEL_STEPS
#define PAROL6_OCTOPUS_DRYRUN_MAX_TRAVEL_STEPS 20000L
#endif

static const uint8_t JOINT_COUNT = 6;
static const long MAX_DRYRUN_TRAVEL_STEPS = PAROL6_OCTOPUS_DRYRUN_MAX_TRAVEL_STEPS;
static const unsigned long MAX_SIMULATION_ITERATIONS = 25000UL;

enum DryRunState {
  DRYRUN_IDLE = 0,
  DRYRUN_CONFIGURED,
  DRYRUN_PREFLIGHT_CHECK,
  DRYRUN_SEEK_LIMIT,
  DRYRUN_LIMIT_FOUND,
  DRYRUN_BACKOFF,
  DRYRUN_VERIFY_RELEASE,
  DRYRUN_COMPLETE,
  DRYRUN_FAIL_INVALID_JOINT,
  DRYRUN_FAIL_PREFLIGHT,
  DRYRUN_FAIL_LIMIT_ALREADY_ACTIVE,
  DRYRUN_FAIL_MAX_TRAVEL_EXCEEDED,
  DRYRUN_FAIL_INVALID_CONFIG,
  DRYRUN_FAIL_TIMEOUT
};

struct DryRunJoint {
  bool configured;
  long maxTravel;
  long limitAt;
  long backoff;
  long currentPosition;
  long virtualTravel;
  DryRunState state;
  char lastResult[128];
};

static DryRunJoint dryRuns[JOINT_COUNT] = {
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
    {false, 0, 0, 0, 0, 0, DRYRUN_IDLE, "not run"},
};

static const char *motorNames[JOINT_COUNT] = {"MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4", "MOTOR5"};
static const char *stopNames[JOINT_COUNT] = {"Stop0", "Stop1", "Stop2", "Stop3", "Stop4", "Stop5"};

static const char *stateName(DryRunState state)
{
  switch (state) {
  case DRYRUN_IDLE:
    return "IDLE";
  case DRYRUN_CONFIGURED:
    return "CONFIGURED";
  case DRYRUN_PREFLIGHT_CHECK:
    return "PREFLIGHT_CHECK";
  case DRYRUN_SEEK_LIMIT:
    return "SEEK_LIMIT";
  case DRYRUN_LIMIT_FOUND:
    return "LIMIT_FOUND";
  case DRYRUN_BACKOFF:
    return "BACKOFF";
  case DRYRUN_VERIFY_RELEASE:
    return "VERIFY_RELEASE";
  case DRYRUN_COMPLETE:
    return "COMPLETE";
  case DRYRUN_FAIL_INVALID_JOINT:
    return "FAIL_INVALID_JOINT";
  case DRYRUN_FAIL_PREFLIGHT:
    return "FAIL_PREFLIGHT";
  case DRYRUN_FAIL_LIMIT_ALREADY_ACTIVE:
    return "FAIL_LIMIT_ALREADY_ACTIVE";
  case DRYRUN_FAIL_MAX_TRAVEL_EXCEEDED:
    return "FAIL_MAX_TRAVEL_EXCEEDED";
  case DRYRUN_FAIL_INVALID_CONFIG:
    return "FAIL_INVALID_CONFIG";
  case DRYRUN_FAIL_TIMEOUT:
    return "FAIL_TIMEOUT";
  default:
    return "UNKNOWN";
  }
}

static void setResult(uint8_t jointIndex, DryRunState state, const char *result)
{
  dryRuns[jointIndex].state = state;
  strncpy(dryRuns[jointIndex].lastResult, result, sizeof(dryRuns[jointIndex].lastResult) - 1);
  dryRuns[jointIndex].lastResult[sizeof(dryRuns[jointIndex].lastResult) - 1] = '\0';
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

static bool parseLong(const char **text, long *value)
{
  while (**text == ' ') {
    ++(*text);
  }

  char *end = nullptr;
  const long parsed = strtol(*text, &end, 10);
  if (end == *text) {
    return false;
  }
  *value = parsed;
  *text = end;
  return true;
}

static void printHelp()
{
  SerialUSB.println("Safe homing dry-run commands:");
  SerialUSB.println("homing_dryrun_help");
  SerialUSB.println("homing_dryrun_safe");
  SerialUSB.println("homing_dryrun_status");
  SerialUSB.println("homing_dryrun_reset");
  SerialUSB.println("homing_dryrun_config N MAX_TRAVEL LIMIT_AT BACKOFF");
  SerialUSB.println("homing_dryrun_run N");
  SerialUSB.println("homing_dryrun_all");
}

static void printSafe()
{
  SerialUSB.println("SAFE HOMING DRY-RUN MODE");
  SerialUSB.println("This does not move the robot.");
  SerialUSB.println("This does not enable motors.");
  SerialUSB.println("This does not generate step pulses.");
  SerialUSB.println("This does not run real homing.");
  SerialUSB.println("It only simulates the homing state machine.");
}

static void printStatus()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    SerialUSB.print("Joint");
    SerialUSB.print(i + 1);
    SerialUSB.print(" / ");
    SerialUSB.print(motorNames[i]);
    SerialUSB.print(" / ");
    SerialUSB.print(stopNames[i]);
    SerialUSB.print(": configured=");
    SerialUSB.print(dryRuns[i].configured ? "YES" : "NO");
    SerialUSB.print(" max_travel=");
    SerialUSB.print(dryRuns[i].maxTravel);
    SerialUSB.print(" limit_at=");
    SerialUSB.print(dryRuns[i].limitAt);
    SerialUSB.print(" backoff=");
    SerialUSB.print(dryRuns[i].backoff);
    SerialUSB.print(" current=");
    SerialUSB.print(dryRuns[i].currentPosition);
    SerialUSB.print(" direction=");
    SerialUSB.print(octopusHomingPreflightDirectionName(i));
    SerialUSB.print(" state=");
    SerialUSB.print(stateName(dryRuns[i].state));
    SerialUSB.print(" last_result=");
    SerialUSB.println(dryRuns[i].lastResult);
  }
  SerialUSB.println("Real homing allowed: NO");
}

static void resetAll()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    dryRuns[i].configured = false;
    dryRuns[i].maxTravel = 0;
    dryRuns[i].limitAt = 0;
    dryRuns[i].backoff = 0;
    dryRuns[i].currentPosition = 0;
    dryRuns[i].virtualTravel = 0;
    setResult(i, DRYRUN_IDLE, "not run");
  }
  SerialUSB.println("Homing dry-run configs/results reset.");
}

static bool configureDryRun(const char *command)
{
  static const char prefix[] = "homing_dryrun_config ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != ' ') {
    SerialUSB.println("FAIL: invalid joint");
    return true;
  }

  long maxTravel = 0;
  long limitAt = 0;
  long backoff = 0;
  if (!parseLong(&tail, &maxTravel) || !parseLong(&tail, &limitAt) || !parseLong(&tail, &backoff)) {
    SerialUSB.println("FAIL: invalid config");
    return true;
  }
  while (*tail == ' ') {
    ++tail;
  }
  if (*tail != '\0') {
    SerialUSB.println("FAIL: invalid config");
    return true;
  }

  if (maxTravel <= 0) {
    SerialUSB.println("FAIL: max travel must be positive");
    return true;
  }
  if (maxTravel > MAX_DRYRUN_TRAVEL_STEPS) {
    SerialUSB.println("FAIL: max travel exceeds safe limit");
    return true;
  }
  if (limitAt <= 0) {
    SerialUSB.println("FAIL: virtual limit trigger must be positive");
    return true;
  }
  if (limitAt > maxTravel) {
    SerialUSB.println("FAIL: virtual limit trigger exceeds max travel");
    return true;
  }
  if (backoff < 0) {
    SerialUSB.println("FAIL: backoff must be >= 0");
    return true;
  }
  if (backoff > MAX_DRYRUN_TRAVEL_STEPS) {
    SerialUSB.println("FAIL: backoff exceeds safe limit");
    return true;
  }

  dryRuns[jointIndex].configured = true;
  dryRuns[jointIndex].maxTravel = maxTravel;
  dryRuns[jointIndex].limitAt = limitAt;
  dryRuns[jointIndex].backoff = backoff;
  dryRuns[jointIndex].currentPosition = 0;
  dryRuns[jointIndex].virtualTravel = 0;
  setResult(jointIndex, DRYRUN_CONFIGURED, "configured");

  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.print(" dry-run configured: max_travel=");
  SerialUSB.print(maxTravel);
  SerialUSB.print(" limit_at=");
  SerialUSB.print(limitAt);
  SerialUSB.print(" backoff=");
  SerialUSB.print(backoff);
  SerialUSB.println(" (RAM-only)");
  return true;
}

static bool runDryRun(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    SerialUSB.println("FAIL: invalid joint");
    return false;
  }

  DryRunJoint &dryRun = dryRuns[jointIndex];
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.print(" / ");
  SerialUSB.print(motorNames[jointIndex]);
  SerialUSB.print(" / ");
  SerialUSB.println(stopNames[jointIndex]);
  SerialUSB.println("This is a dry-run simulation only. Real homing allowed: NO.");

  if (!dryRun.configured) {
    setResult(jointIndex, DRYRUN_FAIL_INVALID_CONFIG, "dry-run config not configured");
    SerialUSB.println("FAIL: dry-run config not configured");
    return false;
  }
  if (dryRun.maxTravel <= 0) {
    setResult(jointIndex, DRYRUN_FAIL_INVALID_CONFIG, "max travel not configured");
    SerialUSB.println("FAIL: max travel not configured");
    return false;
  }
  if (dryRun.limitAt <= 0) {
    setResult(jointIndex, DRYRUN_FAIL_INVALID_CONFIG, "virtual limit trigger not configured");
    SerialUSB.println("FAIL: virtual limit trigger not configured");
    return false;
  }
  if (dryRun.limitAt > dryRun.maxTravel) {
    setResult(jointIndex, DRYRUN_FAIL_INVALID_CONFIG, "virtual limit trigger exceeds max travel");
    SerialUSB.println("FAIL: virtual limit trigger exceeds max travel");
    return false;
  }

  dryRun.state = DRYRUN_PREFLIGHT_CHECK;
  if (octopusSafeLimitsInitialized() && octopusSafeLimitsJointActive(jointIndex)) {
    setResult(jointIndex, DRYRUN_FAIL_LIMIT_ALREADY_ACTIVE, "limit already active at start");
    SerialUSB.println("PREFLIGHT_CHECK: FAIL - limit is active");
    SerialUSB.println("FAIL: limit already active at start");
    return false;
  }

  const bool preflightPass = octopusHomingPreflightPass(jointIndex);
  SerialUSB.print("PREFLIGHT_CHECK: ");
  SerialUSB.println(preflightPass ? "PASS" : "FAIL");
  if (!preflightPass) {
    SerialUSB.print("Preflight not fully satisfied, continuing dry-run simulation only. Real homing would still be blocked. Reason: ");
    SerialUSB.println(octopusHomingPreflightLastReason(jointIndex));
  }

  SerialUSB.print("Direction: ");
  SerialUSB.println(octopusHomingPreflightDirectionName(jointIndex));

  dryRun.state = DRYRUN_SEEK_LIMIT;
  dryRun.virtualTravel = 0;
  dryRun.currentPosition = 0;
  SerialUSB.print("SEEK_LIMIT: virtual travel 0 -> ");
  SerialUSB.println(dryRun.limitAt);

  unsigned long iterations = 0;
  while (dryRun.virtualTravel < dryRun.limitAt) {
    ++dryRun.virtualTravel;
    ++dryRun.currentPosition;
    ++iterations;
    if (dryRun.virtualTravel > dryRun.maxTravel) {
      setResult(jointIndex, DRYRUN_FAIL_MAX_TRAVEL_EXCEEDED, "virtual travel exceeds max travel");
      SerialUSB.println("FAIL: virtual travel exceeds max travel");
      return false;
    }
    if (iterations > MAX_SIMULATION_ITERATIONS) {
      setResult(jointIndex, DRYRUN_FAIL_TIMEOUT, "simulation timeout");
      SerialUSB.println("FAIL: simulation timeout");
      return false;
    }
  }

  dryRun.state = DRYRUN_LIMIT_FOUND;
  SerialUSB.print("LIMIT_FOUND at ");
  SerialUSB.print(dryRun.virtualTravel);
  SerialUSB.println(" steps");

  dryRun.state = DRYRUN_BACKOFF;
  SerialUSB.print("BACKOFF: ");
  SerialUSB.print(dryRun.backoff);
  SerialUSB.println(" steps");
  dryRun.currentPosition -= dryRun.backoff;
  if (dryRun.currentPosition < 0) {
    dryRun.currentPosition = 0;
  }

  dryRun.state = DRYRUN_VERIFY_RELEASE;
  SerialUSB.println("VERIFY_RELEASE: simulated");

  setResult(jointIndex, DRYRUN_COMPLETE, "dry-run complete");
  SerialUSB.println("COMPLETE");
  SerialUSB.println("Dry-run simulation PASS. Real homing allowed: still NO.");
  return true;
}

static bool runCommand(const char *command)
{
  static const char prefix[] = "homing_dryrun_run ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  uint8_t jointIndex = 0;
  const char *tail = nullptr;
  if (!parseJointNumber(command + prefixLen, &jointIndex, &tail) || *tail != '\0') {
    SerialUSB.println("FAIL: invalid joint");
    return true;
  }
  runDryRun(jointIndex);
  return true;
}

static void runAll()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    if (!dryRuns[i].configured) {
      SerialUSB.print("Joint");
      SerialUSB.print(i + 1);
      SerialUSB.println(": FAIL - dry-run config not configured");
      setResult(i, DRYRUN_FAIL_INVALID_CONFIG, "dry-run config not configured");
      continue;
    }
    runDryRun(i);
  }
}

bool octopusHomingDryRunHandleCommand(const char *command)
{
  if (strcmp(command, "homing_dryrun_help") == 0) {
    printHelp();
  } else if (strcmp(command, "homing_dryrun_safe") == 0) {
    printSafe();
  } else if (strcmp(command, "homing_dryrun_status") == 0) {
    printStatus();
  } else if (strcmp(command, "homing_dryrun_reset") == 0) {
    resetAll();
  } else if (configureDryRun(command)) {
  } else if (runCommand(command)) {
  } else if (strcmp(command, "homing_dryrun_all") == 0) {
    runAll();
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

void octopusHomingDryRunPrintStartup()
{
  SerialUSB.println("Homing dry-run: available");
  SerialUSB.println("Real homing: still BLOCKED");
  SerialUSB.flush();
}

bool octopusHomingDryRunConfigured(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return false;
  }
  return dryRuns[jointIndex].configured;
}

bool octopusHomingDryRunLastPass(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return false;
  }
  return dryRuns[jointIndex].state == DRYRUN_COMPLETE;
}

const char *octopusHomingDryRunLastResult(uint8_t jointIndex)
{
  if (jointIndex >= JOINT_COUNT) {
    return "invalid joint";
  }
  return dryRuns[jointIndex].lastResult;
}

#endif
