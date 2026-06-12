#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include "octopus_safe_homing_executor.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "octopus_safe_homing_dryrun.h"
#include "octopus_safe_homing_preflight.h"
#include "octopus_safe_limits.h"

static const uint8_t JOINT_COUNT = 6;

enum HomingExecState {
  HOMING_EXEC_IDLE = 0,
  HOMING_EXEC_REQUESTED,
  HOMING_EXEC_PREFLIGHT,
  HOMING_EXEC_DRYRUN_REQUIRED,
  HOMING_EXEC_HARDWARE_NOT_VALIDATED,
  HOMING_EXEC_BLOCKED,
  HOMING_EXEC_ARMED,
  HOMING_EXEC_SEEK_LIMIT,
  HOMING_EXEC_LIMIT_FOUND,
  HOMING_EXEC_BACKOFF,
  HOMING_EXEC_VERIFY_RELEASE,
  HOMING_EXEC_SET_HOME,
  HOMING_EXEC_COMPLETE,
  HOMING_EXEC_FAIL
};

struct HomingExecValidation {
  bool limitHardwareValidated;
  bool polarityValidated;
  bool dryRunPassed;
  bool requested;
  bool armed;
  HomingExecState state;
  char lastFailure[192];
};

static HomingExecValidation execs[JOINT_COUNT] = {
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
    {false, false, false, false, false, HOMING_EXEC_IDLE, "not checked"},
};

static const char *motorNames[JOINT_COUNT] = {"MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4", "MOTOR5"};
static const char *stopNames[JOINT_COUNT] = {"Stop0", "Stop1", "Stop2", "Stop3", "Stop4", "Stop5"};
static const char *limitNames[JOINT_COUNT] = {"LIMIT1", "LIMIT2", "LIMIT3", "LIMIT4", "LIMIT5", "LIMIT6"};

static bool octopusFutureRealHomingSeekLimit(uint8_t jointIndex);

static const char *yesNo(bool value)
{
  return value ? "YES" : "NO";
}

static const char *stateName(HomingExecState state)
{
  switch (state) {
  case HOMING_EXEC_IDLE:
    return "HOMING_EXEC_IDLE";
  case HOMING_EXEC_REQUESTED:
    return "HOMING_EXEC_REQUESTED";
  case HOMING_EXEC_PREFLIGHT:
    return "HOMING_EXEC_PREFLIGHT";
  case HOMING_EXEC_DRYRUN_REQUIRED:
    return "HOMING_EXEC_DRYRUN_REQUIRED";
  case HOMING_EXEC_HARDWARE_NOT_VALIDATED:
    return "HOMING_EXEC_HARDWARE_NOT_VALIDATED";
  case HOMING_EXEC_BLOCKED:
    return "HOMING_EXEC_BLOCKED";
  case HOMING_EXEC_ARMED:
    return "HOMING_EXEC_ARMED";
  case HOMING_EXEC_SEEK_LIMIT:
    return "HOMING_EXEC_SEEK_LIMIT";
  case HOMING_EXEC_LIMIT_FOUND:
    return "HOMING_EXEC_LIMIT_FOUND";
  case HOMING_EXEC_BACKOFF:
    return "HOMING_EXEC_BACKOFF";
  case HOMING_EXEC_VERIFY_RELEASE:
    return "HOMING_EXEC_VERIFY_RELEASE";
  case HOMING_EXEC_SET_HOME:
    return "HOMING_EXEC_SET_HOME";
  case HOMING_EXEC_COMPLETE:
    return "HOMING_EXEC_COMPLETE";
  case HOMING_EXEC_FAIL:
    return "HOMING_EXEC_FAIL";
  default:
    return "UNKNOWN";
  }
}

static bool realHomingCompileFlagEnabled()
{
#ifdef PAROL6_OCTOPUS_ENABLE_REAL_HOMING
  return true;
#else
  return false;
#endif
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

static void resetReason(uint8_t jointIndex)
{
  strncpy(execs[jointIndex].lastFailure, "none", sizeof(execs[jointIndex].lastFailure) - 1);
  execs[jointIndex].lastFailure[sizeof(execs[jointIndex].lastFailure) - 1] = '\0';
}

static void appendBlocker(char *buffer, size_t size, const char *reason)
{
  if (buffer[0] != '\0') {
    strncat(buffer, "; ", size - strlen(buffer) - 1);
  }
  strncat(buffer, reason, size - strlen(buffer) - 1);
}

static bool dryRunGatePassed(uint8_t jointIndex)
{
  return execs[jointIndex].dryRunPassed || octopusHomingDryRunLastPass(jointIndex);
}

static bool maxTravelConfigured(uint8_t jointIndex)
{
  return octopusHomingPreflightMaxTravel(jointIndex) > 0;
}

static bool runReadinessCheck(uint8_t jointIndex, bool printDetails)
{
  if (jointIndex >= JOINT_COUNT) {
    return false;
  }

  char blockers[sizeof(execs[0].lastFailure)] = "";
  const bool preflightPass = octopusHomingPreflightPass(jointIndex);
  const bool dryRunPass = dryRunGatePassed(jointIndex);
  const bool limitsReady = octopusSafeLimitsInitialized();
  const bool directionValidated = octopusHomingPreflightDirectionValidated(jointIndex);
  const bool maxTravelReady = maxTravelConfigured(jointIndex);

  execs[jointIndex].state = HOMING_EXEC_PREFLIGHT;

  if (!limitsReady) {
    appendBlocker(blockers, sizeof(blockers), "safe limits not initialized");
  }
  if (!preflightPass) {
    appendBlocker(blockers, sizeof(blockers), octopusHomingPreflightLastReason(jointIndex));
  }
  if (!dryRunPass) {
    if (octopusHomingDryRunConfigured(jointIndex)) {
      appendBlocker(blockers, sizeof(blockers), "dry-run configured but not passed");
    } else {
      appendBlocker(blockers, sizeof(blockers), "dry-run not configured or passed");
    }
  }
  if (!execs[jointIndex].limitHardwareValidated) {
    appendBlocker(blockers, sizeof(blockers), "limit hardware not validated");
  }
  if (!execs[jointIndex].polarityValidated) {
    appendBlocker(blockers, sizeof(blockers), "limit polarity not validated");
  }
  if (!directionValidated) {
    appendBlocker(blockers, sizeof(blockers), "direction not validated");
  }
  if (!maxTravelReady) {
    appendBlocker(blockers, sizeof(blockers), "max travel not configured");
  }
  if (!realHomingCompileFlagEnabled()) {
    appendBlocker(blockers, sizeof(blockers), "real homing compile-time enable flag is not defined");
  }

  const bool pass = blockers[0] == '\0';
  if (pass) {
    resetReason(jointIndex);
    execs[jointIndex].state = HOMING_EXEC_ARMED;
  } else {
    strncpy(execs[jointIndex].lastFailure, blockers, sizeof(execs[jointIndex].lastFailure) - 1);
    execs[jointIndex].lastFailure[sizeof(execs[jointIndex].lastFailure) - 1] = '\0';
    execs[jointIndex].state = !realHomingCompileFlagEnabled() ? HOMING_EXEC_BLOCKED : HOMING_EXEC_HARDWARE_NOT_VALIDATED;
  }

  if (printDetails) {
    SerialUSB.print("Joint");
    SerialUSB.print(jointIndex + 1);
    SerialUSB.print(" / ");
    SerialUSB.print(motorNames[jointIndex]);
    SerialUSB.print(" / ");
    SerialUSB.print(stopNames[jointIndex]);
    SerialUSB.print(" / ");
    SerialUSB.print(limitNames[jointIndex]);
    SerialUSB.print(": ");
    SerialUSB.println(pass ? "CHECK PASS" : "CHECK FAIL");
    if (!pass) {
      if (!realHomingCompileFlagEnabled()) {
        SerialUSB.println("CHECK FAIL: real homing compile-time enable flag is not defined.");
      }
      SerialUSB.print("Blockers: ");
      SerialUSB.println(execs[jointIndex].lastFailure);
    }
  }

  return pass;
}

static void printHelp()
{
  SerialUSB.println("Guarded safe homing executor commands:");
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
}

static void printSafe()
{
  SerialUSB.println("SAFE HOMING EXECUTION SKELETON");
  SerialUSB.println("This does not move the robot.");
  SerialUSB.println("This does not enable motors.");
  SerialUSB.println("This does not generate step pulses.");
  SerialUSB.println("Real homing remains blocked.");
  SerialUSB.println("This only prepares the future homing execution architecture.");
  SerialUSB.println("Real homing requires hardware validation and a separate explicit compile-time enable flag.");
}

static void printPolicy()
{
  SerialUSB.println("Real homing is blocked in all current envs.");
  SerialUSB.println("Preflight must pass.");
  SerialUSB.println("Dry-run must pass.");
  SerialUSB.println("Limit hardware must be validated.");
  SerialUSB.println("Limit polarity must be validated.");
  SerialUSB.println("Direction must be validated.");
  SerialUSB.println("Max travel must be configured.");
  SerialUSB.println("PAROL6_OCTOPUS_ENABLE_REAL_HOMING must be explicitly defined in a future env.");
  SerialUSB.println("No current env should define that flag.");
}

static void printStatus()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    const bool preflightPass = octopusHomingPreflightPass(i);
    SerialUSB.print("Joint");
    SerialUSB.print(i + 1);
    SerialUSB.print(" / ");
    SerialUSB.print(motorNames[i]);
    SerialUSB.print(" / ");
    SerialUSB.print(stopNames[i]);
    SerialUSB.print(" / ");
    SerialUSB.print(limitNames[i]);
    SerialUSB.print(": preflight=");
    SerialUSB.print(preflightPass ? "PASS" : "FAIL");
    SerialUSB.print(" dryrun=");
    SerialUSB.print(dryRunGatePassed(i) ? "PASS" : "FAIL");
    SerialUSB.print(" limit_hw_validated=");
    SerialUSB.print(yesNo(execs[i].limitHardwareValidated));
    SerialUSB.print(" polarity_validated=");
    SerialUSB.print(yesNo(execs[i].polarityValidated));
    SerialUSB.print(" direction_validated=");
    SerialUSB.print(yesNo(octopusHomingPreflightDirectionValidated(i)));
    SerialUSB.print(" max_travel_configured=");
    SerialUSB.print(yesNo(maxTravelConfigured(i)));
    SerialUSB.print(" real_homing_allowed=");
    SerialUSB.print(realHomingCompileFlagEnabled() ? "POSSIBLE_WITH_ALL_GATES" : "NO");
    SerialUSB.print(" state=");
    SerialUSB.print(stateName(execs[i].state));
    SerialUSB.print(" last_failure=");
    SerialUSB.println(execs[i].lastFailure);
  }
}

static void requestJoint(uint8_t jointIndex)
{
  execs[jointIndex].requested = true;
  execs[jointIndex].armed = false;
  execs[jointIndex].state = HOMING_EXEC_REQUESTED;
  SerialUSB.print("Future homing request recorded for Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" (RAM-only).");
  SerialUSB.println("Real homing still blocked.");
  SerialUSB.println("Run homing_exec_check N to see blockers.");
}

static void armJoint(uint8_t jointIndex)
{
  if (runReadinessCheck(jointIndex, true)) {
    execs[jointIndex].armed = true;
    execs[jointIndex].state = HOMING_EXEC_ARMED;
    SerialUSB.println("Future homing armed. This should only be possible in a future explicitly enabled env.");
  } else {
    execs[jointIndex].armed = false;
    execs[jointIndex].state = HOMING_EXEC_BLOCKED;
    SerialUSB.println("Homing arm refused safely.");
  }
}

static void runJoint(uint8_t jointIndex)
{
  (void)octopusFutureRealHomingSeekLimit(jointIndex);
  SerialUSB.println("Refusing real homing: PAROL6_OCTOPUS_ENABLE_REAL_HOMING is not defined.");
  runReadinessCheck(jointIndex, true);
  execs[jointIndex].armed = false;
  execs[jointIndex].state = HOMING_EXEC_BLOCKED;
}

static void cancelAll()
{
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    execs[i].requested = false;
    execs[i].armed = false;
    execs[i].state = HOMING_EXEC_IDLE;
  }
  SerialUSB.println("Homing execution request canceled. Executor state is IDLE.");
}

static void clearJoint(uint8_t jointIndex)
{
  execs[jointIndex].limitHardwareValidated = false;
  execs[jointIndex].polarityValidated = false;
  execs[jointIndex].dryRunPassed = false;
  execs[jointIndex].requested = false;
  execs[jointIndex].armed = false;
  execs[jointIndex].state = HOMING_EXEC_IDLE;
  strncpy(execs[jointIndex].lastFailure, "cleared", sizeof(execs[jointIndex].lastFailure) - 1);
  execs[jointIndex].lastFailure[sizeof(execs[jointIndex].lastFailure) - 1] = '\0';
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" homing executor validation flags cleared.");
}

static void markLimitValidated(uint8_t jointIndex)
{
  execs[jointIndex].limitHardwareValidated = true;
  SerialUSB.println("Only mark this after real optocoupler/limit wiring was tested safely.");
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" limit_hardware_validated=YES (RAM-only)");
}

static void markPolarityValidated(uint8_t jointIndex)
{
  execs[jointIndex].polarityValidated = true;
  SerialUSB.println("Only mark this after active/inactive limit polarity was confirmed physically.");
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" polarity_validated=YES (RAM-only)");
}

static void markDryRunPassed(uint8_t jointIndex)
{
  if (!octopusHomingDryRunLastPass(jointIndex)) {
    SerialUSB.print("Refusing dry-run validation mark: dry-run last result is not PASS. Last result: ");
    SerialUSB.println(octopusHomingDryRunLastResult(jointIndex));
    return;
  }

  execs[jointIndex].dryRunPassed = true;
  SerialUSB.print("Joint");
  SerialUSB.print(jointIndex + 1);
  SerialUSB.println(" dryrun_passed=YES (RAM-only)");
}

static bool octopusFutureRealHomingSeekLimit(uint8_t jointIndex)
{
  (void)jointIndex;
#ifndef PAROL6_OCTOPUS_ENABLE_REAL_HOMING
  return false;
#else
  // Future implementation will seek the validated limit using bounded motion.
  return false;
#endif
}

bool octopusHomingExecutorHandleCommand(const char *command)
{
  uint8_t jointIndex = 0;

  if (strcmp(command, "homing_exec_help") == 0) {
    printHelp();
  } else if (strcmp(command, "homing_exec_safe") == 0) {
    printSafe();
  } else if (strcmp(command, "homing_exec_status") == 0) {
    printStatus();
  } else if (parseSingleJointCommand(command, "homing_exec_check ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      runReadinessCheck(jointIndex, true);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_request ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      requestJoint(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_arm ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      armJoint(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_run ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      runJoint(jointIndex);
    }
  } else if (strcmp(command, "homing_exec_cancel") == 0) {
    cancelAll();
  } else if (parseSingleJointCommand(command, "homing_exec_clear ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      clearJoint(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_mark_limit_validated ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      markLimitValidated(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_mark_polarity_validated ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      markPolarityValidated(jointIndex);
    }
  } else if (parseSingleJointCommand(command, "homing_exec_mark_dryrun_passed ", &jointIndex)) {
    if (jointIndex < JOINT_COUNT) {
      markDryRunPassed(jointIndex);
    }
  } else if (strcmp(command, "homing_exec_policy") == 0) {
    printPolicy();
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

void octopusHomingExecutorPrintStartup()
{
  SerialUSB.println("Homing executor skeleton: available");
  SerialUSB.println("Real homing: still BLOCKED");
  SerialUSB.flush();
}

#endif
