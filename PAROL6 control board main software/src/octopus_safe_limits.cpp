#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446

#include "octopus_safe_limits.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "iodefs.h"

static const int LIMIT_COUNT = 6;
static const unsigned long DEBOUNCE_MS = 20;
static const bool USE_PULLUP = false;

struct OctopusSafeLimitState {
  const char *jointName;
  const char *stopName;
  const char *limitName;
  const char *pinName;
  uint32_t pin;
  bool rawState;
  bool debouncedState;
  bool lastRawState;
  bool invert;
  unsigned long transitionCount;
  unsigned long lastChangeMs;
};

static OctopusSafeLimitState limits[LIMIT_COUNT] = {
    {"Joint1", "Stop0", "LIMIT1", "PG6", LIMIT1, false, false, false, false, 0, 0},
    {"Joint2", "Stop1", "LIMIT2", "PG9", LIMIT2, false, false, false, false, 0, 0},
    {"Joint3", "Stop2", "LIMIT3", "PG10", LIMIT3, false, false, false, false, 0, 0},
    {"Joint4", "Stop3", "LIMIT4", "PG11", LIMIT4, false, false, false, false, 0, 0},
    {"Joint5", "Stop4", "LIMIT5", "PG12", LIMIT5, false, false, false, false, 0, 0},
    {"Joint6", "Stop5", "LIMIT6", "PG13", LIMIT6, false, false, false, false, 0, 0},
};

static bool initialized = false;

static const char *levelName(bool state)
{
  return state ? "HIGH" : "LOW";
}

static const char *onOff(bool state)
{
  return state ? "ON" : "OFF";
}

static const char *yesNo(bool state)
{
  return state ? "YES" : "NO";
}

static bool activeState(const OctopusSafeLimitState &limit)
{
  return limit.invert ? !limit.debouncedState : limit.debouncedState;
}

static bool parseJointInvertCommand(const char *command, int *jointIndex, bool *invert)
{
  static const char prefix[] = "limit_invert ";
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  const char *arg = command + prefixLen;
  char *end = nullptr;
  const long parsed = strtol(arg, &end, 10);
  if (end == arg || parsed < 1 || parsed > LIMIT_COUNT || *end != ' ') {
    return false;
  }

  while (*end == ' ') {
    ++end;
  }

  if (strcmp(end, "on") == 0) {
    *invert = true;
  } else if (strcmp(end, "off") == 0) {
    *invert = false;
  } else {
    return false;
  }

  *jointIndex = (int)parsed - 1;
  return true;
}

static void updateLimits()
{
  const unsigned long now = millis();

  for (int i = 0; i < LIMIT_COUNT; ++i) {
    const bool currentRaw = digitalRead(limits[i].pin) == HIGH;
    limits[i].rawState = currentRaw;

    if (currentRaw != limits[i].lastRawState) {
      limits[i].lastRawState = currentRaw;
      limits[i].lastChangeMs = now;
    }

    if ((now - limits[i].lastChangeMs) >= DEBOUNCE_MS && limits[i].debouncedState != currentRaw) {
      limits[i].debouncedState = currentRaw;
      limits[i].transitionCount++;
    }
  }
}

static void printLimitSafe()
{
  SerialUSB.println("LIMIT/OPTO SAFE MODE");
  SerialUSB.println("Stop0..Stop5 are read-only diagnostic inputs.");
  SerialUSB.println("Homing is still blocked.");
  SerialUSB.println("Limit polarity must be physically validated before homing.");
  SerialUSB.println("Never connect 24V directly to Octopus MCU inputs.");
}

static void printLimits()
{
  updateLimits();
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].jointName);
    SerialUSB.print(" / ");
    SerialUSB.print(limits[i].stopName);
    SerialUSB.print(" / ");
    SerialUSB.print(limits[i].limitName);
    SerialUSB.print(": raw=");
    SerialUSB.print(levelName(limits[i].rawState));
    SerialUSB.print(" debounced=");
    SerialUSB.print(levelName(limits[i].debouncedState));
    SerialUSB.print(" active=");
    SerialUSB.print(yesNo(activeState(limits[i])));
    SerialUSB.print(" count=");
    SerialUSB.print(limits[i].transitionCount);
    SerialUSB.print(" invert=");
    SerialUSB.println(onOff(limits[i].invert));
  }
}

static void printLimitsRaw()
{
  updateLimits();
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].stopName);
    SerialUSB.print("/");
    SerialUSB.print(limits[i].limitName);
    SerialUSB.print("=");
    SerialUSB.print(levelName(limits[i].rawState));
    if (i < LIMIT_COUNT - 1) {
      SerialUSB.print(" ");
    }
  }
  SerialUSB.println();
}

static void printLimitsBrief()
{
  updateLimits();
  SerialUSB.print("LIMITS active=");
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(activeState(limits[i]) ? '1' : '0');
  }
  SerialUSB.print(" raw=");
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].rawState ? '1' : '0');
  }
  SerialUSB.print(" debounced=");
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].debouncedState ? '1' : '0');
  }
  SerialUSB.println();
}

static void printLimitsConfig()
{
  updateLimits();
  SerialUSB.print("debounce_ms=");
  SerialUSB.println(DEBOUNCE_MS);
  SerialUSB.print("input_mode=");
  SerialUSB.println(USE_PULLUP ? "INPUT_PULLUP" : "INPUT");
  SerialUSB.print("invert_flags=");
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].invert ? '1' : '0');
  }
  SerialUSB.println();
  SerialUSB.println("Mapping:");
  SerialUSB.println("Stop0 / LIMIT1 -> Joint1");
  SerialUSB.println("Stop1 / LIMIT2 -> Joint2");
  SerialUSB.println("Stop2 / LIMIT3 -> Joint3");
  SerialUSB.println("Stop3 / LIMIT4 -> Joint4");
  SerialUSB.println("Stop4 / LIMIT5 -> Joint5");
  SerialUSB.println("Stop5 / LIMIT6 -> Joint6");
  SerialUSB.println("Invert flags are RAM-only and are not persisted.");
  SerialUSB.println("Limit polarity must be physically validated before homing.");
}

static void printLimitCounts()
{
  updateLimits();
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    SerialUSB.print(limits[i].jointName);
    SerialUSB.print("/");
    SerialUSB.print(limits[i].stopName);
    SerialUSB.print("=");
    SerialUSB.print(limits[i].transitionCount);
    if (i < LIMIT_COUNT - 1) {
      SerialUSB.print(" ");
    }
  }
  SerialUSB.println();
}

static void resetLimitCounts()
{
  for (int i = 0; i < LIMIT_COUNT; ++i) {
    limits[i].transitionCount = 0;
  }
  SerialUSB.println("Limit transition counts reset.");
}

static void setLimitInvert(int jointIndex, bool invert)
{
  limits[jointIndex].invert = invert;
  SerialUSB.print(limits[jointIndex].jointName);
  SerialUSB.print(" / ");
  SerialUSB.print(limits[jointIndex].stopName);
  SerialUSB.print(" / ");
  SerialUSB.print(limits[jointIndex].limitName);
  SerialUSB.print(" invert=");
  SerialUSB.println(onOff(limits[jointIndex].invert));
  SerialUSB.println("Invert setting is RAM-only and will not persist.");
}

void octopusSafeLimitsInit()
{
  const unsigned long now = millis();

  for (int i = 0; i < LIMIT_COUNT; ++i) {
    pinMode(limits[i].pin, USE_PULLUP ? INPUT_PULLUP : INPUT);
    const bool state = digitalRead(limits[i].pin) == HIGH;
    limits[i].rawState = state;
    limits[i].debouncedState = state;
    limits[i].lastRawState = state;
    limits[i].invert = false;
    limits[i].transitionCount = 0;
    limits[i].lastChangeMs = now;
  }

  initialized = true;
  SerialUSB.println("Octopus Stop0..Stop5 limit inputs initialized read-only.");
  SerialUSB.println("Limit polarity must be physically validated before homing.");
}

void octopusSafeLimitsPoll()
{
  if (!initialized) {
    return;
  }
  updateLimits();
}

bool octopusSafeLimitsHandleCommand(const char *command)
{
  int jointIndex = -1;
  bool invert = false;

  if (strcmp(command, "limits") == 0) {
    printLimits();
  } else if (strcmp(command, "limits_raw") == 0) {
    printLimitsRaw();
  } else if (strcmp(command, "limits_config") == 0) {
    printLimitsConfig();
  } else if (parseJointInvertCommand(command, &jointIndex, &invert)) {
    setLimitInvert(jointIndex, invert);
  } else if (strncmp(command, "limit_invert ", 13) == 0) {
    SerialUSB.println("ERROR: limit_invert requires N in range 1..6 and on/off.");
  } else if (strcmp(command, "limit_counts") == 0) {
    printLimitCounts();
  } else if (strcmp(command, "limit_reset_counts") == 0) {
    resetLimitCounts();
  } else if (strcmp(command, "limit_safe") == 0) {
    printLimitSafe();
  } else if (strcmp(command, "limits_brief") == 0) {
    printLimitsBrief();
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

bool octopusSafeLimitsInitialized()
{
  return initialized;
}

bool octopusSafeLimitsJointActive(int jointIndex)
{
  if (!initialized || jointIndex < 0 || jointIndex >= LIMIT_COUNT) {
    return false;
  }

  updateLimits();
  return activeState(limits[jointIndex]);
}

#endif
