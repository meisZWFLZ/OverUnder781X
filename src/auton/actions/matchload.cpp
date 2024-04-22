#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "fieldDimensions.h"
#include <vector>

using namespace fieldDimensions;
using namespace auton::utils;

/**
 * @brief
 * Lemlib uses heading, which is like a compass, whereas trig functions like
 * sine and cosine use trig angles. Therefore we must convert the trig angle to
 * a heading.
 *
 * @param angle trig angle
 * @return heading angle
 */
float trigAngleToHeading(float angle) { return -(angle * 180 / M_PI) + 90; }

/**
 * @brief whether the driver has pressed any buttons or moved the joysticks
 */
bool exitForDriver = false;
/**
 * @brief how much the driver must move the joysticks for the macro to exit
 */
const int inputThreshold = 127*0.375;

/**
 * @brief exits the macro because the driver has exited and ensure motion stops
 */
void exitBecauseDriver() {
  exitForDriver = true;
  Robot::chassis->cancelMotion();
}

/**
 * @brief checks that the driver has not exited the macro by checking all the
 * controller inputs
 *
 * @return true if the driver has exited the macro
 * @return false if the driver has not exited the macro
 */
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

/**
 * @returns whether a motion currently is running and the driver has not exited
 * the macro
 */
bool betterIsMotionRunning() { return isMotionRunning() && !checkDriverExit(); }

/**
 * @brief waits until driver exits or motion is done
 */
void betterWaitUntilDone() {
  while (betterIsMotionRunning()) pros::delay(10);
}

void auton::actions::matchload(int triballs, int until) {
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
  const lemlib::Pose shootingTarget {MAX_X - TILE_LENGTH, -2};

  // where the robot should go to to matchload
  lemlib::Pose matchloadTarget = {MIN_X + TILE_RADIUS + 0.5,
                                  MIN_Y + TILE_LENGTH + 8.5};
  const float trigMatchloadTargetTheta = matchloadTarget.angle(shootingTarget);
  // calculate the angle to the shooting target
  matchloadTarget.theta = trigAngleToHeading(trigMatchloadTargetTheta);

  // where the robot should go to to smoothly go to matchload bar
  const float stagingTargetDistance = 9;
  lemlib::Pose stagingTarget =
      matchloadTarget +
      lemlib::Pose {
          float(cos(trigMatchloadTargetTheta) * stagingTargetDistance),
          float(sin(trigMatchloadTargetTheta) * stagingTargetDistance)};
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

  Robot::Subsystems::catapult->matchload(until - pros::millis(), triballs);
  // then move to matchload bar
  Robot::chassis->moveToPose(matchloadTarget.x, matchloadTarget.y,
                             matchloadTarget.theta, 750, {.forwards = false});
  betterWaitUntilDone();
  if (checkDriverExit()) return;
  printf("matchload boomerang touch done\n");
  // make sure the robot is touching the matchload bar
  tank(-64, -64, 0, 0);

  const float startingTheta = Robot::chassis->getPose().theta;

  // when the robot touches the bar it should begin to turn
  waitUntil([startingTheta] {
    return robotAngDist(startingTheta) > 1 || checkDriverExit();
  });

  if (checkDriverExit()) return;
  printf("matchload touch done\n");

  // touch matchload bar
  Robot::Actions::expandBackWing();

  // run turn pid until done matchloading or driver exits
  lemlib::PID turnPID {
      Robot::Tunables::angularController.kP,
      0.05,
      Robot::Tunables::angularController.kD,
      Robot::Tunables::angularController.windupRange,
  };
  lemlib::PID lateralPID {
      Robot::Tunables::lateralController.kP,
      0.2,
      Robot::Tunables::lateralController.kD,
      Robot::Tunables::lateralController.windupRange,
  };

  // prevent robot from turning for too fast
  const float maxSpeed = 127;
  while (Robot::Subsystems::catapult->getIsMatchloading() &&
         !checkDriverExit()) {
    const float targetTheta =
        trigAngleToHeading(Robot::chassis->getPose().angle(shootingTarget));
    const float angularError =
        lemlib::angleError(targetTheta, Robot::chassis->getPose().theta, false);
    const float matchloadTargetAngularErrorRadians =
        lemlib::angleError(Robot::chassis->getPose().angle(matchloadTarget),
                           Robot::chassis->getPose(true).theta, true);
    const float angleErrorRad = lemlib::degToRad(90 - angularError);
    const float lateralError =
        Robot::chassis->getPose().distance(matchloadTarget) *
        cos(matchloadTargetAngularErrorRadians);

    float angularPower = turnPID.update(angularError);
    float lateralPower = lateralPID.update(lateralError) /* 0 */;

    // limit pid output
    angularPower = std::clamp(angularPower, -127.0f, 127.0f);
    lateralPower = std::clamp(lateralPower, -127.0f, 127.0f);

    // overturn
    if (std::abs(angularError) + std::abs(lateralPower) > maxSpeed) {
      lateralPower =
          (maxSpeed - std::abs(angularPower)) * lemlib::sgn(lateralPower);
    }
    printf("power:\t%4.2f\t%4.2f\n", angularPower, lateralPower);

    // prevent the robot from turning too fast

    tank(lateralPower + angularPower, lateralPower - angularPower, 0, 0);
    pros::delay(10);
  };

  if (checkDriverExit()) return;

  Robot::Actions::retractBackWing();
  // set slew to 5 for skills
  Robot::chassis->lateralSettings.slew = 5;

  constexpr float halfRobotWidthWithWingsExpanded =
      Robot::Dimensions::drivetrainWidth / 2 +
      Robot::Dimensions::frontWingLength;
  constexpr float xForRunningAlongLongBarrier =
      -halfRobotWidthWithWingsExpanded;

  // don't hit close short barrier
  Robot::chassis->moveToPose(
      -TILE_LENGTH, MIN_Y + TILE_LENGTH - 2 + halfRobotWidthWithWingsExpanded,
      RIGHT, 1750, {.minSpeed = 96});
  // wait until past short barrier
  waitUntil([] {
    return !betterIsMotionRunning() ||
           Robot::chassis->getPose().x > -TILE_LENGTH;
  });
  if (checkDriverExit()) return;

  // then smack wings into balls
  Robot::Subsystems::wings->front->enable();
  Robot::Actions::outtake();

  // arc to face parallel to long barrier and push balls beside it
  Robot::chassis->moveToPose(xForRunningAlongLongBarrier, -TILE_RADIUS + 6, UP,
                             1500, {.minSpeed = 72});

  // wait until bot is almost mid way
  waitUntil([] {
    return Robot::chassis->getPose().y > -TILE_RADIUS ||
           !betterIsMotionRunning();
  });
  Robot::chassis->cancelMotion();
  if (checkDriverExit()) return;

  // push balls towards short barrier
  Robot::chassis->moveToPose(xForRunningAlongLongBarrier, 2 * TILE_LENGTH + 3,
                             UP, 2000, {.minSpeed = 96});
  waitUntil([] {
    return !betterIsMotionRunning() ||
           Robot::chassis->getPose().y >
               TILE_LENGTH + Robot::Dimensions::drivetrainLength / 2;
  });
  Robot::chassis->cancelMotion();
  if (checkDriverExit()) return;
  stop();
  printf("exit\n");
}