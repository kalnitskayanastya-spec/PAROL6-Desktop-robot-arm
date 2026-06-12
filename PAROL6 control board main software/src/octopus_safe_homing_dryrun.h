#ifndef OCTOPUS_SAFE_HOMING_DRYRUN_H
#define OCTOPUS_SAFE_HOMING_DRYRUN_H

#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include <stdint.h>

bool octopusHomingDryRunHandleCommand(const char *command);
void octopusHomingDryRunPrintStartup();
bool octopusHomingDryRunConfigured(uint8_t jointIndex);
bool octopusHomingDryRunLastPass(uint8_t jointIndex);
const char *octopusHomingDryRunLastResult(uint8_t jointIndex);

#endif

#endif
