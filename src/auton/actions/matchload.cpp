#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "fieldDimensions.h"
#include <vector>

using namespace fieldDimensions;
using namespace auton::utils;

float trigAngleToHeading(float angle) { return -(angle * 180 / M_PI) + 90; }

bool exitForDriver = false;
const int inputThreshold = 24;

void exitBecauseDriver() {
  exitForDriver = true;
  Robot::chassis->cancelMotion();
}

bool checkDriverExit() {
  if (exitForDriver) {
    exitBecauseDriver();
    return true;
  }
  exitForDriver |=
      std::abs(Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) >
      inputThreshold;
  exitForDriver |=
      std::abs(Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >
      inputThreshold;
  const static std::vector<pros::controller_digital_e_t> buttons = {
      pros::E_CONTROLLER_DIGITAL_A,    pros::E_CONTROLLER_DIGITAL_B,
      pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_Y,
      pros::E_CONTROLLER_DIGITAL_L1,   pros::E_CONTROLLER_DIGITAL_L2,
      pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_R2,
      pros::E_CONTROLLER_DIGITAL_UP,   pros::E_CONTROLLER_DIGITAL_DOWN,
      pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT};
  if (exitForDriver) {
    exitBecauseDriver();
    return true;
  }
  for (const auto& button : buttons) {
    exitForDriver |= Robot::control.get_digital(button);
    if (exitForDriver) {
      exitBecauseDriver();
      return true;
    }
  }
  return false;
}

bool betterIsMotionRunning() { return isMotionRunning() && !checkDriverExit(); }

void betterWaitUntilDone() {
  while (betterIsMotionRunning()) pros::delay(10);
}

void auton::actions::matchload(bool wait) {
  // weird angled set (see discord)
  Robot::chassis->setPose(fieldDimensions::skillsStartingPose, false);

  // push alliance balls into goal
  Robot::chassis->moveToPose(MIN_Y + TILE_RADIUS, -TILE_RADIUS, DOWN, 8000,
                             {.forwards = false, .minSpeed = 127});
  if (checkDriverExit()) return;
  pros::delay(1000);
  Robot::chassis->cancelMotion();

  // POSES
  // where the robot should be shooting to
  const lemlib::Pose shootingTarget {MAX_X - TILE_LENGTH, -6};

  // where the robot should go to to matchload
  lemlib::Pose matchloadTarget = {MIN_X + TILE_LENGTH - 5.5,
                                  MIN_Y + TILE_LENGTH - 2};
  const float trigMatchloadTargetTheta = matchloadTarget.angle(shootingTarget);
  // calculate the angle to the shooting target
  matchloadTarget.theta = trigAngleToHeading(trigMatchloadTargetTheta);

  // where the robot should go to to smoothly go to matchload bar
  const float stagingTargetDistance = 12;
  lemlib::Pose stagingTarget =
      matchloadTarget +
      lemlib::Pose {cos(trigMatchloadTargetTheta) * stagingTargetDistance,
                    sin(trigMatchloadTargetTheta) * stagingTargetDistance};
  stagingTarget.theta = matchloadTarget.theta;

  // calculate the angle to the shooting target
  stagingTarget.theta = trigAngleToHeading(stagingTarget.angle(shootingTarget));

  printf("stage x: %f\n", stagingTarget.x);
  printf("stage y: %f\n", stagingTarget.y);
  printf("stage theta: %f\n", stagingTarget.theta);

  printf("load x: %f\n", matchloadTarget.x);
  printf("load y: %f\n", matchloadTarget.y);
  printf("load theta: %f\n", matchloadTarget.theta);

  // move in front of matchload bar
  Robot::chassis->moveToPose(stagingTarget.x, stagingTarget.y,
                             stagingTarget.theta, 3000, {.minSpeed = 64});

  // wait until robot is angled correctly
  waitUntil([stagingTarget] {
    return !betterIsMotionRunning() || robotAngDist(stagingTarget.theta) < 5;
  });
  if (checkDriverExit()) return;

  printf("staging done\n");
  Robot::chassis->cancelMotion();

  Robot::Subsystems::catapult->matchload(35000, 46);
  // then move to matchload bar
  Robot::chassis->moveToPose(matchloadTarget.x, matchloadTarget.y,
                             matchloadTarget.theta, 750, {.forwards = false});
  betterWaitUntilDone();
  if (checkDriverExit()) return;
  printf("matchload boomerang touch done\n");
  // make sure the robot is touching the matchload bar
  tank(-32, -32, 0, 0);

  const float startingTheta = Robot::chassis->getPose().theta;

  // when the robot touches the bar it should begin to turn
  waitUntil([startingTheta] {
    return robotAngDist(startingTheta) > 1 || checkDriverExit();
  });

  if (checkDriverExit()) return;
  // adjust for any momentum
  tank(12, -16, 100, 0);
  // keep robot angled
  
  tank(0, -16, 0, 0);
  printf("matchload touch done\n");

  // wait until done matchloading or driver exits
  if (wait)
    waitUntil([] {
      return !Robot::Subsystems::catapult->getIsMatchloading() ||
             // takes the max of the absolute value of the left and right
             // joystick and compares it to the input threshold
             checkDriverExit();
    });
  printf("exit\n");
}