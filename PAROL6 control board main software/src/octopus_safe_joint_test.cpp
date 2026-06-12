#ifdef PAROL6_OCTOPUS_SAFE_JOINT_TEST

#include "octopus_safe_joint_test.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
#include "octopus_safe_homing_dryrun.h"
#include "octopus_safe_homing_executor.h"
#include "octopus_safe_homing_preflight.h"
#include "octopus_safe_limits.h"
#endif

static const int JOINT_COUNT = 6;
static const int NO_JOINT = -1;
static const int MAX_SAFE_STEPS = 200;
static const unsigned long ARM_TIMEOUT_MS = 10000;
static const unsigned long MOVE_TIMEOUT_MS = 10000;
static const float SAFE_MAX_SPEED = 250.0f;
static const float SAFE_ACCELERATION = 100.0f;

static int selectedJoint = NO_JOINT;
static bool armed = false;
static bool selectedJointEnabled = false;
static unsigned long armedAtMs = 0;

static const char *motorNameForJoint(int jointIndex)
{
  static const char *names[] = {"MOTOR0", "MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4", "MOTOR5"};
  if (jointIndex < 0 || jointIndex >= JOINT_COUNT) {
    return "NONE";
  }
  return names[jointIndex];
}

static void disableAndDisarm()
{
  octopusSafeJointTestDisableAllMotors();
  selectedJointEnabled = false;
  armed = false;
  armedAtMs = 0;
}

static bool armExpired()
{
  return armed && (millis() - armedAtMs >= ARM_TIMEOUT_MS);
}

static void expireIfNeeded()
{
  if (armExpired()) {
    disableAndDisarm();
    SerialUSB.println("Joint test arm expired; all motors disabled.");
    SerialUSB.flush();
  }
}

static bool parseIntegerArgument(const char *command, const char *prefix, long *value)
{
  const size_t prefixLen = strlen(prefix);
  if (strncmp(command, prefix, prefixLen) != 0) {
    return false;
  }

  const char *arg = command + prefixLen;
  if (*arg == '\0') {
    return false;
  }

  char *end = nullptr;
  const long parsed = strtol(arg, &end, 10);
  if (end == arg || *end != '\0') {
    return false;
  }

  *value = parsed;
  return true;
}

static bool ensureSelectedForAction(const char *action)
{
  if (selectedJoint == NO_JOINT) {
    SerialUSB.print("Refusing ");
    SerialUSB.print(action);
    SerialUSB.println(": select joint first.");
    return false;
  }
  return true;
}

static bool ensureArmedForAction(const char *action)
{
  expireIfNeeded();
  if (!armed) {
    SerialUSB.print("Refusing ");
    SerialUSB.print(action);
    SerialUSB.println(": run joint_arm first.");
    return false;
  }
  return true;
}

static void printJointTestHelp()
{
  SerialUSB.println("SAFE JOINT TEST MODE");
  SerialUSB.println("Only one selected joint can be enabled.");
  SerialUSB.println("Homing is blocked. Commander motion is blocked. Cartesian motion is blocked.");
  SerialUSB.println("Max step command is limited to 200.");
  SerialUSB.println("Limit active blocks joint_step until polarity/direction are validated.");
  SerialUSB.println();
  SerialUSB.println("Commands:");
  SerialUSB.println("joint_test_help");
  SerialUSB.println("joint_test_status");
  SerialUSB.println("joint_select N");
  SerialUSB.println("joint_arm");
  SerialUSB.println("joint_disarm");
  SerialUSB.println("joint_enable");
  SerialUSB.println("joint_disable");
  SerialUSB.println("joint_step N");
  SerialUSB.println("joint_safe");
  SerialUSB.println("homing_preflight_help");
  SerialUSB.println("homing_dryrun_help");
  SerialUSB.println("homing_exec_help");
}

static void printJointSafe()
{
  SerialUSB.println("SAFE JOINT TEST MODE");
  SerialUSB.println("Only one selected joint can be enabled.");
  SerialUSB.println("Homing is blocked.");
  SerialUSB.println("Commander motion commands are blocked.");
  SerialUSB.println("Cartesian motion is blocked.");
  SerialUSB.println("Max step command is limited.");
  SerialUSB.println("Limit active blocks joint_step until polarity/direction are validated.");
  SerialUSB.println("Motors are disabled on startup.");
  SerialUSB.println("This mode is for controlled bring-up only.");
}

static void printJointStatus()
{
  expireIfNeeded();
  SerialUSB.println("joint_test_compiled=YES");
  SerialUSB.print("selected_joint=");
  if (selectedJoint == NO_JOINT) {
    SerialUSB.println("NONE");
  } else {
    SerialUSB.print(selectedJoint + 1);
    SerialUSB.print(" / ");
    SerialUSB.println(motorNameForJoint(selectedJoint));
  }
  SerialUSB.print("armed=");
  SerialUSB.println(armed ? "YES" : "NO");
  SerialUSB.print("any_joint_enabled=");
  SerialUSB.println(selectedJointEnabled ? "YES" : "NO");
  SerialUSB.print("max_step_limit=");
  SerialUSB.println(MAX_SAFE_STEPS);
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  SerialUSB.print("limit_blocks_joint_step=");
  SerialUSB.println(octopusSafeLimitsInitialized() ? "YES" : "NO");
#endif
  SerialUSB.println("Mapping:");
  SerialUSB.println("Joint1 -> MOTOR0");
  SerialUSB.println("Joint2 -> MOTOR1");
  SerialUSB.println("Joint3 -> MOTOR2");
  SerialUSB.println("Joint4 -> MOTOR3");
  SerialUSB.println("Joint5 -> MOTOR4");
  SerialUSB.println("Joint6 -> MOTOR5");
}

static void selectJoint(int jointNumber)
{
  if (jointNumber < 1 || jointNumber > JOINT_COUNT) {
    SerialUSB.println("ERROR: joint_select requires N in range 1..6.");
    return;
  }

  disableAndDisarm();
  selectedJoint = jointNumber - 1;
  SerialUSB.print("Selected Joint");
  SerialUSB.print(jointNumber);
  SerialUSB.print(" -> ");
  SerialUSB.println(motorNameForJoint(selectedJoint));
  SerialUSB.println("Disarmed. Run joint_arm before enabling or stepping.");
}

static void armJoint()
{
  if (!ensureSelectedForAction("joint_arm")) {
    return;
  }

  armed = true;
  armedAtMs = millis();
  selectedJointEnabled = false;
  SerialUSB.println("Joint test armed for one bounded enable/movement session.");
  SerialUSB.println("Arm expires after one movement command or 10 seconds.");
}

static void enableSelectedJoint()
{
  if (!ensureSelectedForAction("joint_enable") || !ensureArmedForAction("joint_enable")) {
    return;
  }

  octopusSafeJointTestDisableAllMotors();
  octopusSafeJointTestSetMotorEnable(selectedJoint, true);
  selectedJointEnabled = true;
  SerialUSB.print("Enabled Joint");
  SerialUSB.print(selectedJoint + 1);
  SerialUSB.print(" only -> ");
  SerialUSB.println(motorNameForJoint(selectedJoint));
}

static void disableSelectedJoint()
{
  (void)selectedJoint;
  disableAndDisarm();
  SerialUSB.println("All motors disabled. Joint test disarmed.");
}

static void stepSelectedJoint(long steps)
{
  if (selectedJoint == NO_JOINT) {
    SerialUSB.println("Refusing joint_step: select joint first.");
    return;
  }
  if (!armed) {
    SerialUSB.println("Refusing joint_step: run joint_arm first.");
    return;
  }
  expireIfNeeded();
  if (!armed) {
    SerialUSB.println("Refusing joint_step: run joint_arm first.");
    return;
  }
  if (labs(steps) > MAX_SAFE_STEPS) {
    SerialUSB.println("Refusing joint_step: abs(N) exceeds safe limit 200.");
    return;
  }
  if (steps == 0) {
    SerialUSB.println("Refusing joint_step: N must be non-zero.");
    return;
  }
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  if (octopusSafeLimitsInitialized() && octopusSafeLimitsJointActive(selectedJoint)) {
    SerialUSB.println("Refusing joint_step: selected joint limit is active.");
    return;
  }
#endif

  AccelStepper *steppers = octopusSafeJointTestSteppers();
  MotorStruct *joints = octopusSafeJointTestJoints();
  AccelStepper &selectedStepper = steppers[selectedJoint];
  const unsigned long startMs = millis();
  bool timedOut = false;

  octopusSafeJointTestDisableAllMotors();
  octopusSafeJointTestSetMotorEnable(selectedJoint, true);
  selectedJointEnabled = true;

  selectedStepper.setMaxSpeed(SAFE_MAX_SPEED);
  selectedStepper.setAcceleration(SAFE_ACCELERATION);
  selectedStepper.move(steps);

  while (selectedStepper.distanceToGo() != 0) {
    selectedStepper.run();
    if (millis() - startMs >= MOVE_TIMEOUT_MS) {
      selectedStepper.stop();
      timedOut = true;
      break;
    }
  }

  joints[selectedJoint].position = selectedStepper.currentPosition();
  joints[selectedJoint].speed = selectedStepper.speed();
  disableAndDisarm();

  if (timedOut) {
    SerialUSB.println("ERROR: joint_step timed out; all motors disabled and joint test disarmed.");
    return;
  }

  SerialUSB.print("Moved Joint");
  SerialUSB.print(selectedJoint + 1);
  SerialUSB.print(" by ");
  SerialUSB.print(steps);
  SerialUSB.print(" steps on ");
  SerialUSB.print(motorNameForJoint(selectedJoint));
  SerialUSB.println(". All motors disabled. Joint test disarmed.");
}

void octopusSafeJointTestInit()
{
  selectedJoint = NO_JOINT;
  armed = false;
  selectedJointEnabled = false;
  armedAtMs = 0;
  octopusSafeJointTestDisableAllMotors();
}

void octopusSafeJointTestPoll()
{
  expireIfNeeded();
}

void octopusSafeJointTestPrintStartup()
{
  SerialUSB.println("--- PAROL6 OCTOPUS SAFE JOINT TEST ---");
  SerialUSB.println("Main firmware motor backend is used.");
  SerialUSB.println("Only bounded single-joint movement is allowed.");
  SerialUSB.println("Homing blocked. Commander motion blocked. Cartesian motion blocked.");
  SerialUSB.println("Homing preflight: available");
  SerialUSB.println("Homing dry-run: available");
  SerialUSB.println("Homing executor skeleton: available");
  SerialUSB.println("Real homing: still BLOCKED");
  SerialUSB.println("Type joint_test_help.");
  SerialUSB.flush();
}

bool octopusSafeJointTestHandleCommand(const char *command)
{
  long value = 0;

  if (strcmp(command, "joint_test_help") == 0) {
    printJointTestHelp();
  } else if (strcmp(command, "joint_safe") == 0) {
    printJointSafe();
  } else if (strcmp(command, "joint_test_status") == 0) {
    printJointStatus();
  } else if (parseIntegerArgument(command, "joint_select ", &value)) {
    selectJoint((int)value);
  } else if (strcmp(command, "joint_arm") == 0) {
    armJoint();
  } else if (strcmp(command, "joint_disarm") == 0) {
    disableSelectedJoint();
  } else if (strcmp(command, "joint_enable") == 0) {
    enableSelectedJoint();
  } else if (strcmp(command, "joint_disable") == 0) {
    disableSelectedJoint();
  } else if (parseIntegerArgument(command, "joint_step ", &value)) {
    stepSelectedJoint(value);
#ifdef PAROL6_BOARD_OCTOPUS_PRO_F446
  } else if (octopusHomingExecutorHandleCommand(command)) {
    return true;
#endif
  } else {
    return false;
  }

  SerialUSB.flush();
  return true;
}

#endif
