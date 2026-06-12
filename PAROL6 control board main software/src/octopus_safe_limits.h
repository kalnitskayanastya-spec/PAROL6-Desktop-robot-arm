#ifndef OCTOPUS_SAFE_LIMITS_H
#define OCTOPUS_SAFE_LIMITS_H

#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446

void octopusSafeLimitsInit();
void octopusSafeLimitsPoll();
bool octopusSafeLimitsHandleCommand(const char *command);
bool octopusSafeLimitsInitialized();
bool octopusSafeLimitsJointActive(int jointIndex);
bool octopusSafeLimitsJointRaw(int jointIndex);
bool octopusSafeLimitsJointDebounced(int jointIndex);
bool octopusSafeLimitsJointInvert(int jointIndex);
unsigned long octopusSafeLimitsDebounceMs();

#endif

#endif
