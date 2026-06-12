#ifndef OCTOPUS_SAFE_MAIN_DIAG_H
#define OCTOPUS_SAFE_MAIN_DIAG_H

#ifdef PAROL6_OCTOPUS_SAFE_MAIN

#include "structs.h"

void octopusSafeMainDiagPrintStartupHint();
bool octopusSafeMainDiagPoll(Robot &robot, MotorStruct joints[], int jointCount);

#endif

#endif
