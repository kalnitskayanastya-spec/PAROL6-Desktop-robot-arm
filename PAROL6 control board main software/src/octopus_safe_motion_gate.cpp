#include "octopus_safe_motion_gate.h"

#include <Arduino.h>
#include <string.h>

bool octopusSafeMotionEnabled()
{
#if defined(PAROL6_OCTOPUS_SAFE_MAIN) || defined(PAROL6_SAFE_NO_MOTION)
  return false;
#else
  return true;
#endif
}

void octopusReportBlockedMotion(const char *reason)
{
#if defined(PAROL6_OCTOPUS_SAFE_MAIN) || defined(PAROL6_SAFE_NO_MOTION)
  static const char *lastReason = nullptr;
  static unsigned long lastReportMs = 0;
  const unsigned long now = millis();

  if (lastReason != nullptr && strcmp(lastReason, reason) == 0 && (now - lastReportMs) < 1000) {
    return;
  }

  lastReason = reason;
  lastReportMs = now;

  SerialUSB.print("BLOCKED: motion command ignored in PAROL6_OCTOPUS_SAFE_MAIN: ");
  SerialUSB.println(reason);
  SerialUSB.flush();
#else
  (void)reason;
#endif
}

bool octopusAllowMotionCommand(const char *reason)
{
  if (!octopusSafeMotionEnabled()) {
    octopusReportBlockedMotion(reason);
    return false;
  }

  return true;
}
