#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

ASSET(def_score_alliance_txt);
ASSET(def_elevation_bar_txt);

using namespace fieldDimensions;

// bool waitUntilMotionDone(int timeout = -1) {
//   const int startTime = pros::millis();
//   while (Robot::chassis->distTravelled != -1 &&
//          (timeout == -1 || pros::millis() - startTime < timeout)) {
//     printf("dist: %f\n", Robot::chassis->distTravelled);
//     pros::delay(100);
//   }
// }

void runDefensive() {
  using namespace fieldDimensions;
  using namespace auton::actions;

  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2 - (1 - 1.0 / 8) / 2,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainWidth / 2 - (5.0 / 8),
       LEFT},
      false);

  const lemlib::Pose scoreAllyTarget = {-42.5, -12.75};
  const lemlib::Pose intakeMiddleTarget = {
      TILE_LENGTH, Robot::Dimensions::drivetrainLength / 2 - 5, UP};
  const lemlib::Pose outtakeMiddleTarget = {
      0 - Robot::Dimensions::drivetrainLength / 2, -TILE_LENGTH / 2.0, RIGHT};

  Robot::Actions::lowerIntake();
  pros::delay(200);
  Robot::chassis->tank(-127, 127);

  while (Robot::chassis->getPose().theta > 90) pros::delay(10);

  Robot::Actions::raiseIntake();

  Robot::chassis->turnTo(0, 0, 500);
  Robot::chassis->waitUntilDone();
  // Curve around and score into large part of goal
  Robot::chassis->follow(def_score_alliance_txt, 13, 2750);

  // wait until 5 inches away from, then outtake
  while (Robot::chassis->getPose().distance(scoreAllyTarget) > 18)
    pros::delay(10);
  Robot::Actions::outtake();
  printf("time: %ims\n", pros::millis());

  // wait until done with pure pursuit motion
  Robot::chassis->waitUntilDone();

  // // back up for ram
  // Robot::chassis->tank(-127, -127);
  // pros::delay(200);
  // ram triball into goal
  Robot::chassis->tank(127, 127);
  pros::delay(100);

  // get out of goal
  Robot::chassis->tank(-127, -127);
  pros::delay(200);
  Robot::Actions::stopIntake();

  printf("done with ally triball\n");

  Robot::chassis->turnTo(0, -1000000, 750);
  Robot::chassis->waitUntilDone();
  Robot::chassis->follow(def_elevation_bar_txt, 12, 3000);

  Robot::chassis->turnTo(10000, 10000, 1000);
  Robot::chassis->waitUntilDone();

  Robot::chassis->tank(48, 48);
  Robot::Actions::expandWings();
  pros::delay(500);
  Robot::chassis->tank(0, 0);
  pros::delay(2000);
  Robot::Actions::retractWings();
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"),
                                         runDefensive};