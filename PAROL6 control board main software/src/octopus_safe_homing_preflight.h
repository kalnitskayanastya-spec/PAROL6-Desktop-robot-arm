#ifndef OCTOPUS_SAFE_HOMING_PREFLIGHT_H
#define OCTOPUS_SAFE_HOMING_PREFLIGHT_H

#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include <Arduino.h>
#include <stdint.h>

bool octopusHomingPreflightHandleCommand(const char *command);
bool octopusHomingPreflightPass(uint8_t jointIndex);
const char *octopusHomingPreflightLastReason(uint8_t jointIndex);
void octopusPrintHomingPreflight(Stream &out, uint8_t jointIndex);
void octopusHomingPreflightPrintStartup();

#endif

#endif
