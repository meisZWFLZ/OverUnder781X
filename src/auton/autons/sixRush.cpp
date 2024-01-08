#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

// ASSET(def_score_alliance_txt);
ASSET(rush_6_intake_txt);

using namespace fieldDimensions;
using namespace auton::utils;

// bool waitUntilMotionDone(int timeout = -1) {
//   const int startTime = pros::millis();
//   while (Robot::chassis->distTravelled != -1 &&
//          (timeout == -1 || pros::millis() - startTime < timeout)) {
//     printf("dist: %f\n", Robot::chassis->distTravelled);
//     pros::delay(100);
//   }111111111111
// }

void runSixRush() {
  using namespace fieldDimensions;
  using namespace auton::actions;

  Robot::chassis->setPose(
      {0 + TILE_LENGTH * 1.5, MIN_Y + TILE_LENGTH - 12.0 / 2, UP}, false);
  Robot::Actions::expandWings();
  Robot::chassis->follow(rush_6_intake_txt, 13, 2750);
  pros::delay(300);
  Robot::Actions::retractWings();
  Robot::chassis->waitUntil(36);
  Robot::Actions::intake();
  Robot::chassis->waitUntilDone();
  pros::delay(250);

  tank(-64, -127, 100, 0);

  Robot::chassis->turnTo(10000000, 0, 1000);
  Robot::chassis->waitUntilDone();
  Robot::Actions::expandWings();
  tank(127, 127, 300, 0);
  Robot::Actions::outtake();
  tank(127, 127, 400, 0);
  tank(-48, -127, 300, 0);
  Robot::Actions::retractWings();
  Robot::Actions::stopIntake();
  Robot::chassis->moveToPose(0 + 9, -36,LEFT, 2000);

  Robot::chassis->waitUntil(10);
  Robot::Actions::intake();

  Robot::chassis->waitUntilDone();
  stop();
}

auton::Auton auton::autons::defensive = {(char*)("6 ball rush / offensive / right"),
                                         runSixRush};