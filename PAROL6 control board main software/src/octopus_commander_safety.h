#ifndef OCTOPUS_COMMANDER_SAFETY_H
#define OCTOPUS_COMMANDER_SAFETY_H

#if defined(PAROL6_OCTOPUS_SAFE_MAIN) || defined(PAROL6_SAFE_NO_MOTION)

enum OctopusCommandRisk {
  OCTO_CMD_READ_ONLY,
  OCTO_CMD_LOW,
  OCTO_CMD_MEDIUM,
  OCTO_CMD_HIGH,
  OCTO_CMD_DANGEROUS,
  OCTO_CMD_UNKNOWN
};

OctopusCommandRisk octopusClassifyCommanderCommand(int command);
const char *octopusCommanderRiskName(OctopusCommandRisk risk);
bool octopusAllowCommanderCommand(int command, const char *context);
void octopusReportCommanderCommand(int command, OctopusCommandRisk risk, const char *context);
void octopusReportBlockedCommanderCommand(int command, OctopusCommandRisk risk, const char *context);
void octopusResetCommanderStats();
void octopusPrintCommanderStats();
void octopusPrintLastCommanderCommand();
void octopusPrintCommanderStatus();
void octopusPrintCommanderPolicy();

#endif

#endif
