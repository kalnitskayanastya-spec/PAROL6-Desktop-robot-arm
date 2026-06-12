#if defined(PAROL6_OCTOPUS_SAFE_MAIN) || defined(PAROL6_SAFE_NO_MOTION)

#include "octopus_commander_safety.h"

#include <Arduino.h>

struct CommanderSafetyStats {
  unsigned long totalSeen = 0;
  unsigned long totalBlocked = 0;
  unsigned long readOnlyAllowed = 0;
  unsigned long lowAllowed = 0;
  unsigned long mediumBlocked = 0;
  unsigned long highBlocked = 0;
  unsigned long dangerousBlocked = 0;
  unsigned long unknownBlocked = 0;
  int lastCommand = -1;
  OctopusCommandRisk lastRisk = OCTO_CMD_UNKNOWN;
  bool lastAllowed = false;
};

static CommanderSafetyStats stats;

OctopusCommandRisk octopusClassifyCommanderCommand(int command)
{
  switch (command) {
  case 255:
    return OCTO_CMD_READ_ONLY; // Commander heartbeat/status packet; no movement handler.
  case 102:
  case 103:
    return OCTO_CMD_LOW; // Disable and clear-error/reset state are safe in no-motion mode.
  case 101:
    return OCTO_CMD_MEDIUM; // Enable robot/motors.
  case 123:
  case 156:
    return OCTO_CMD_HIGH; // Jog and go-to-position.
  case 69:
  case 100:
    return OCTO_CMD_DANGEROUS; // Repeatability/test motion and homing.
  default:
    return OCTO_CMD_UNKNOWN;
  }
}

const char *octopusCommanderRiskName(OctopusCommandRisk risk)
{
  switch (risk) {
  case OCTO_CMD_READ_ONLY:
    return "READ_ONLY";
  case OCTO_CMD_LOW:
    return "LOW";
  case OCTO_CMD_MEDIUM:
    return "MEDIUM";
  case OCTO_CMD_HIGH:
    return "HIGH";
  case OCTO_CMD_DANGEROUS:
    return "DANGEROUS";
  case OCTO_CMD_UNKNOWN:
  default:
    return "UNKNOWN";
  }
}

static bool isAllowedRisk(OctopusCommandRisk risk)
{
  return risk == OCTO_CMD_READ_ONLY || risk == OCTO_CMD_LOW;
}

void octopusReportCommanderCommand(int command, OctopusCommandRisk risk, const char *context)
{
  SerialUSB.print("Commander command: ");
  SerialUSB.print(command);
  SerialUSB.print(" risk=");
  SerialUSB.print(octopusCommanderRiskName(risk));
  SerialUSB.print(" context=");
  SerialUSB.println(context);
  SerialUSB.flush();
}

void octopusReportBlockedCommanderCommand(int command, OctopusCommandRisk risk, const char *context)
{
  SerialUSB.print("BLOCKED: Commander command ignored in PAROL6_OCTOPUS_SAFE_MAIN: ");
  SerialUSB.print(command);
  SerialUSB.print(" risk=");
  SerialUSB.print(octopusCommanderRiskName(risk));
  SerialUSB.print(" context=");
  SerialUSB.println(context);
  SerialUSB.flush();
}

bool octopusAllowCommanderCommand(int command, const char *context)
{
  const OctopusCommandRisk risk = octopusClassifyCommanderCommand(command);
  const bool allowed = isAllowedRisk(risk);

  stats.totalSeen++;
  stats.lastCommand = command;
  stats.lastRisk = risk;
  stats.lastAllowed = allowed;

  if (allowed) {
    if (risk == OCTO_CMD_READ_ONLY) {
      stats.readOnlyAllowed++;
    } else if (risk == OCTO_CMD_LOW) {
      stats.lowAllowed++;
    }
    octopusReportCommanderCommand(command, risk, context);
    return true;
  }

  stats.totalBlocked++;
  if (risk == OCTO_CMD_MEDIUM) {
    stats.mediumBlocked++;
  } else if (risk == OCTO_CMD_HIGH) {
    stats.highBlocked++;
  } else if (risk == OCTO_CMD_DANGEROUS) {
    stats.dangerousBlocked++;
  } else {
    stats.unknownBlocked++;
  }

  octopusReportBlockedCommanderCommand(command, risk, context);
  return false;
}

void octopusResetCommanderStats()
{
  stats = CommanderSafetyStats();
  SerialUSB.println("Commander safety stats reset.");
  SerialUSB.flush();
}

void octopusPrintLastCommanderCommand()
{
  SerialUSB.print("last_command=");
  SerialUSB.println(stats.lastCommand);
  SerialUSB.print("last_risk=");
  SerialUSB.println(octopusCommanderRiskName(stats.lastRisk));
  SerialUSB.print("last_allowed=");
  SerialUSB.println(stats.lastAllowed ? "YES" : "NO");
}

void octopusPrintCommanderStats()
{
  SerialUSB.print("read-only allowed=");
  SerialUSB.println(stats.readOnlyAllowed);
  SerialUSB.print("low allowed=");
  SerialUSB.println(stats.lowAllowed);
  SerialUSB.print("medium blocked=");
  SerialUSB.println(stats.mediumBlocked);
  SerialUSB.print("high blocked=");
  SerialUSB.println(stats.highBlocked);
  SerialUSB.print("dangerous blocked=");
  SerialUSB.println(stats.dangerousBlocked);
  SerialUSB.print("unknown blocked=");
  SerialUSB.println(stats.unknownBlocked);
}

void octopusPrintCommanderStatus()
{
  SerialUSB.println("Commander safety/sniffer: ENABLED");
  SerialUSB.println("policy: safe main blocks motion, homing, enable, and unknown commands");
  SerialUSB.print("total commands seen=");
  SerialUSB.println(stats.totalSeen);
  SerialUSB.print("total blocked=");
  SerialUSB.println(stats.totalBlocked);
  octopusPrintLastCommanderCommand();
}

void octopusPrintCommanderPolicy()
{
  SerialUSB.println("Safe Commander Policy:");
  SerialUSB.println("- read-only commands may be allowed only if explicitly classified.");
  SerialUSB.println("- motion commands are blocked.");
  SerialUSB.println("- homing commands are blocked.");
  SerialUSB.println("- motor enable commands are blocked.");
  SerialUSB.println("- unknown commands are blocked.");
}

#endif
