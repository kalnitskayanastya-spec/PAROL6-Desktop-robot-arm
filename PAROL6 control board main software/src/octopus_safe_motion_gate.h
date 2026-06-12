#ifndef OCTOPUS_SAFE_MOTION_GATE_H
#define OCTOPUS_SAFE_MOTION_GATE_H

bool octopusSafeMotionEnabled();
bool octopusAllowMotionCommand(const char *reason);
void octopusReportBlockedMotion(const char *reason);

#endif
