#include "auton.h"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

ASSET(ball6_matchload_sweep_txt);
ASSET(ball6_center_plow_1_txt);

void run6Ball() {
  Robot::chassis->setPose(
      {0 + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       MIN_Y + TILE_RADIUS, LEFT},
      false);

  // intake triball
  Robot::Actions::intake();
  tank(127, 127, 150, 0);

  // get away from other quadrant and begin turning
  tank(0, 0, 180, 127.0 / 18);
  pros::delay(100);
  tank(-127, -127, 220, 127.0 / 18);
  tank(127, -127, 200, 0);

  // dont constantly spin the intake which could damage the motor
  printf("intook triball\n");

  // turn to face right side of field (I use (10000000,0) as the target since
  // its basically a turn to 90 deg)
  Robot::chassis->turnTo(10000000, 0, 400);
  Robot::chassis->waitUntilDone();
  Robot::Actions::stopIntake();
  printf("turn done\n");

  // Path to smoothly remove triball in matchload zone and from the matchload
  // zone and plow the three balls into the goal
  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the matchload zone to expand wings
  waitUntilDistToPose({MAX_X - TILE_LENGTH, MIN_Y + TILE_LENGTH - 3}, 6, 0,
                      true);
  Robot::Actions::expandWings();
  printf("expand wings\n");
  pros::delay(200);
  Robot::chassis->cancelMotion();

  // retract wings soon after removing the triball from the matchload zone
  tank(24, 36, 200);
  Robot::Actions::retractWings();
  printf("retract wings\n");
  tank(24, 36, 200);

  printf("back to follow\n");

  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the goal to outtake

  pros::delay(450);
  Robot::Actions::outtake();
  printf("outtake\n");

  while ((std::abs(Robot::chassis->getPose().x - (MAX_X - TILE_RADIUS)) > 10 ||
          robotAngDist(0) > 20) &&
         Robot::chassis->isInMotion() &&
         Robot::chassis->getPose().theta < 420) {
    pros::delay(10);
  }
  printf("cancel follow\n");
  // fully ram into goal
  Robot::chassis->cancelMotion();
  tank(127, 100, 0, 0);
  waitUntilDistToPose({MAX_X - TILE_RADIUS, -TILE_LENGTH}, 10, 0, false);
  pros::delay(100);
  tank(-127, -127, 200, 0);
  tank(90, 127, 500, 0);
  pros::delay(150);

  Robot::chassis->setPose(
      {Robot::chassis->getPose().x + 3.5f,
       0 - TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2 + 5, UP},
      false);

  tank(0, 0, 0, 0);
  // back out of goal and turn towards next path
  tank(-127, -92, 400, 127.0 / 18);
  Robot::Actions::stopIntake();

  // intake triball closest to elevation pole
  // Robot::chassis->moveToPoint(TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5, 1000);
  // waitUntilDistToPose({TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5, 0}, 8, 0,
  // true);

  const lemlib::Pose intakeCloseTriballTarget {0 + 3, 0 - TILE_LENGTH - 2,
                                               LEFT};

  Robot::chassis->moveToPose(
      intakeCloseTriballTarget.x, intakeCloseTriballTarget.y,
      intakeCloseTriballTarget.theta, 2500, {.lead = 0.5});

  // Wait until close to the triball to intake. This prevents the robot from
  // intaking a triball that has been flung over from another robot
  waitUntilDistToPose(intakeCloseTriballTarget, 12, 0, true);
  Robot::Actions::intake();

  // wait until triball is intaked
  waitUntilDistToPose(intakeCloseTriballTarget, 8, 100, true);
  Robot::chassis->cancelMotion();
  // turn to face the triball in the center of the field
  tank(-64, -127, 200, 0);

  // push currently held triball and center triball into goal
  Robot::chassis->follow(ball6_center_plow_1_txt, 10, 3000);

  // wait until triball is fully intaked to stop
  pros::delay(300);
  Robot::Actions::stopIntake();

  // wait until near the second triball to expand wings
  waitUntilDistToPose({TILE_LENGTH, 0}, 10, 0);
  Robot::Actions::expandWings();

  // wait until near the goal to outtake
  Robot::chassis->waitUntil(24);
  Robot::Actions::outtake();

  // wait until path follow is done to stop intake and retract wings
  Robot::chassis->waitUntilDone();
  tank(127, 127, 100, 0);
  Robot::Actions::stopIntake();
  Robot::Actions::retractWings();

  // turn to face last triball
  tank(-64, -127, 200);

  Robot::chassis->moveToPoint(TILE_RADIUS - 1.2, -TILE_LENGTH * 1.5 - 0.5, 2000);
  Robot::chassis->turnTo(-1 * pow(10, 6), .68 * pow(10, 6), 1000, true, 64);
  pros::delay(40);
  Robot::Actions::expandWings();
  Robot::chassis->waitUntilDone();
  // outtake
  printf("stop\n");
}

auton::Auton auton::autons::sixBall = {(char*)("6 ball / offensive / right"),
                                       run6Ball};