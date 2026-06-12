#ifndef OCTOPUS_SAFE_HOMING_EXECUTOR_H
#define OCTOPUS_SAFE_HOMING_EXECUTOR_H

#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

bool octopusHomingExecutorHandleCommand(const char *command);
void octopusHomingExecutorPrintStartup();

#endif

#endif
