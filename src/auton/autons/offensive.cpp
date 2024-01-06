#include "auton.h"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

ASSET(ball6_matchload_sweep_txt);
ASSET(ball6_center_plow_1_txt);

const float DEFAULT_SLEW = 5;

void tankUpdate(float left, float right, float slew) {
  const int prevLeft = Robot::Motors::leftDrive.get_voltages()[0];
  const int prevRight = Robot::Motors::rightDrive.get_voltages()[0];
  const int leftLimited = lemlib::slew(left, prevLeft, slew);
  const int rightLimited = lemlib::slew(right, prevRight, slew);
  Robot::chassis->tank(leftLimited, rightLimited);
}

/**
 * @brief Tank drive with slew rate control
 * Slew rate control is a method of limiting the rate of change of the voltage
 * sent to the motors. This is useful for ensuring that the tracking wheels
 * always contact the ground.
 *
 * @param left left side power
 * @param right right side power
 * @param ms time to run for
 * @param slew slew rate
 */
void tank(float left, float right, int ms, float slew = DEFAULT_SLEW) {
  const int startTime = pros::millis();
  do {
    tankUpdate(left, right, slew);
    pros::delay(10);
  } while (pros::millis() - startTime < ms);
}

/**
 * @brief Stops sending voltage to the drivetrain motors
 */
void stop() { Robot::chassis->tank(0, 0); }

/**
 * @brief Wait until the robot is within a circle with a radius of error and
 * centered at pose for time milliseconds.
 *
 * @param pose pose to find distance from robot to
 * @param error
 * @param time
 */
void waitUntilDistToPose(lemlib::Pose pose, float error, int time = 0,
                         bool checkMotionRunning = false) {
  printf("start wait\n");
  const int start = pros::millis();
  int inRangeStartTime = 0;
  pros::delay(100);
  do {
    if (inRangeStartTime == 0) {
      if (Robot::chassis->getPose().distance(pose) < error)
        inRangeStartTime = pros::millis();
    } else if (pros::millis() - inRangeStartTime > time) {
      printf("breaking\n");
      break;
    }
    pros::delay(10);
  } while (!checkMotionRunning || Robot::chassis->isInMotion());
  printf("time in wait: %i\n", pros::millis() - start);
}

void runOffensive() {
  Robot::chassis->setPose(
      {0 + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       MIN_Y + TILE_RADIUS, LEFT},
      false);

  // intake triball
  Robot::Actions::intake();
  tank(127, 127, 200);

  // get away from other quadrant and begin turning
  tank(-64, -127, 200);

  // dont constantly spin the intake which could damage the motor
  Robot::Actions::stopIntake();

  printf("intook triball\n");

  // turn to face right side of field (I use (10000000,0) as the target since
  // its basically a turn to 90 deg)
  Robot::chassis->turnTo(10000000, 0, 400);
  Robot::chassis->waitUntilDone();
  printf("turn done\n");

  // Path to smoothly remove triball in matchload zone and from the matchload
  // zone and plow the three balls into the goal
  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 2500);

  // wait until the robot is near the matchload zone to expand wings
  waitUntilDistToPose({MAX_X - TILE_LENGTH, MIN_Y + TILE_LENGTH - 3}, 6, 250);
  Robot::Actions::expandWings();
  printf("expand wings\n");

  // retract wings soon after removing the triball from the matchload zone
  pros::delay(500);
  Robot::Actions::retractWings();
  printf("retract wings\n");

  // wait until the robot is near the goal to outtake
  Robot::chassis->waitUntil(60);
  Robot::Actions::outtake();
  printf("outtake\n");

  // fully ram into goal
  tank(127, 127, 100, 0);

  // stop ramming
  stop();

  // back out of goal and turn towards next path
  tank(-127, -64, 100);
  Robot::Actions::stopIntake();

  // intake triball closest to elevation pole
  const lemlib::Pose intakeCloseTriballTarget {
      0 + Robot::Dimensions::drivetrainLength / 2 + 2, 0 - TILE_LENGTH, LEFT};
  Robot::chassis->moveToPose(intakeCloseTriballTarget.x,
                             intakeCloseTriballTarget.y,
                             intakeCloseTriballTarget.theta, 2000);

  // Wait until close to the triball to intake. This prevents the robot from
  // intaking a triball that has been flung over from another robot
  waitUntilDistToPose(intakeCloseTriballTarget, 5, 0);
  Robot::Actions::intake();

  // wait until triball is intaked
  Robot::chassis->waitUntilDone();

  // turn to face the triball in the center of the field
  tank(0, -127, 150);

  // push currently held triball and center triball into goal
  Robot::chassis->follow(ball6_center_plow_1_txt, 10, 2000);

  // wait until triball is fully intaked to stop
  pros::delay(300);
  Robot::Actions::stopIntake();

  // wait until near the second triball to expand wings
  waitUntilDistToPose({TILE_LENGTH, 0}, 10, 0);
  Robot::Actions::expandWings();

  // wait until near the goal to outtake
  waitUntilDistToPose({2 * TILE_LENGTH, 0}, 10, 0);
  Robot::Actions::outtake();

  // wait until path follow is done to stop intake and retract wings
  Robot::chassis->waitUntilDone();
  Robot::Actions::stopIntake();
  Robot::Actions::retractWings();

  // turn to face last triball
  tank(-64, -127, 200);

  // intake last triball
  const lemlib::Pose intakeLastTriballTarget {intakeCloseTriballTarget.x, 0,
                                              LEFT};
  Robot::chassis->moveToPose(intakeLastTriballTarget.x,
                             intakeLastTriballTarget.y,
                             intakeLastTriballTarget.theta, 2000);

  // Wait until close to the triball to intake. This prevents the robot from
  // intaking a triball that has been flung over from another robot
  waitUntilDistToPose(intakeLastTriballTarget, 5, 0);
  Robot::Actions::intake();

  // wait until triball is intaked
  Robot::chassis->waitUntilDone();

  // turn to face the goal
  tank(-127, -64, 200);

  Robot::chassis->turnTo(1000000, 0, 1000);

  // give time to intake
  pros::delay(200);
  Robot::Actions::stopIntake();

  // once within 10 degrees of facing towards the goal, stop turning
  while (Robot::chassis->isInMotion() &&
         lemlib::angleError(Robot::chassis->getPose().theta, RIGHT) > 10) {
    pros::delay(10);
  }
  Robot::chassis->cancelMotion();

  // full power ram into goal
  tank(127, 127, 350);

  // wait until near goal to outtake
  waitUntilDistToPose({TILE_LENGTH * 1.5, 0}, 10);
  Robot::Actions::outtake();

  // wait until at goal to stop
  waitUntilDistToPose({TILE_LENGTH * 2, 0}, 10, 400);
  stop();

  // back out of goal and turn towards elevation pole
  tank(-64, -127, 150);

  // touch elevation bar

  // outtake
  printf("stop\n");
}

auton::Auton auton::autons::offensive = {(char*)("offensive / right"),
                                         runOffensive};