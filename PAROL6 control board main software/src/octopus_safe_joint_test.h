#ifndef OCTOPUS_SAFE_JOINT_TEST_H
#define OCTOPUS_SAFE_JOINT_TEST_H

#ifdef PAROL6_OCTOPUS_SAFE_JOINT_TEST

#include <AccelStepper.h>
#include "structs.h"

AccelStepper *octopusSafeJointTestSteppers();
MotorStruct *octopusSafeJointTestJoints();
void octopusSafeJointTestDisableAllMotors();
void octopusSafeJointTestSetMotorEnable(int jointIndex, bool enable);

void octopusSafeJointTestInit();
void octopusSafeJointTestPoll();
void octopusSafeJointTestPrintStartup();
bool octopusSafeJointTestHandleCommand(const char *command);

#endif

#endif
