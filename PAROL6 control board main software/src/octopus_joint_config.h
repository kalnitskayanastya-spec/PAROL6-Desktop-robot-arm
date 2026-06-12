#ifndef OCTOPUS_JOINT_CONFIG_H
#define OCTOPUS_JOINT_CONFIG_H

#if defined(PAROL6_BOARD_OCTOPUS_PRO_F446) && defined(PAROL6_OCTOPUS_SAFE_MAIN)

#include <stdint.h>

enum OctopusJointConfigSource {
  ORIGINAL_PAROL6_CONFIRMED = 0,
  OCTOPUS_PORT_CONFIRMED,
  PLACEHOLDER_NOT_CONFIRMED,
  MEASUREMENT_REQUIRED,
  HARDWARE_VALIDATION_REQUIRED
};

struct OctopusJointConfig {
  uint8_t jointNumber;
  const char *motorConnector;
  const char *limitInput;
  const char *limitAlias;
  float stepsPerDegree;
  bool directionInverted;
  float softMinDeg;
  float softMaxDeg;
  float homeOffsetDeg;
  float maxSpeed;
  float maxAcceleration;
  long homingMaxTravelSteps;
  long safeJointTestMaxStepCount;
  int tmcCurrentMa;
  OctopusJointConfigSource configSource;
  OctopusJointConfigSource stepsPerDegreeSource;
  OctopusJointConfigSource directionInvertSource;
  OctopusJointConfigSource softLimitsSource;
  OctopusJointConfigSource homeOffsetSource;
  OctopusJointConfigSource maxSpeedSource;
  OctopusJointConfigSource maxAccelerationSource;
  OctopusJointConfigSource homingMaxTravelSource;
  OctopusJointConfigSource safeJointTestMaxStepSource;
  OctopusJointConfigSource tmcCurrentSource;
};

bool octopusJointConfigHandleCommand(const char *command);
void octopusJointConfigPrintStartup();
const OctopusJointConfig *octopusJointConfigGet(uint8_t jointIndex);
const char *octopusJointConfigSourceName(OctopusJointConfigSource source);

bool octopusJointConfigIsValid(uint8_t jointIndex);
float octopusJointConfigStepsPerDegree(uint8_t jointIndex);
bool octopusJointConfigDirInverted(uint8_t jointIndex);
long octopusJointConfigHomingMaxTravelSteps(uint8_t jointIndex);
float octopusJointConfigMinDeg(uint8_t jointIndex);
float octopusJointConfigMaxDeg(uint8_t jointIndex);
float octopusJointConfigHomeOffsetDeg(uint8_t jointIndex);

#endif

#endif
