#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

ASSET(off_score_alliance_txt);
ASSET(off_elevation_bar_txt);

void runOffensive() {
  Robot::chassis->setPose(rightStartingPose, false);
  // @todo add offensive auton

  Robot::chassis->follow(off_score_alliance_txt, 10, 2000);
  Robot::chassis->waitUntil(25);
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();

  // ram it again
  Robot::chassis->tank(-127, -127);
  pros::delay(250);
  Robot::chassis->tank(127, 127);
  pros::delay(500);
  // get outta there
  Robot::chassis->tank(-127, -127);
  Robot::Actions::stopIntake();
  pros::delay(400);

  Robot::chassis->turnTo(-10000, 0, 1000);
  Robot::chassis->waitUntilDone();

  Robot::chassis->follow(off_elevation_bar_txt, 10, 2750);
  Robot::chassis->waitUntilDone();

  Robot::chassis->tank(127, 127);
  // Robot::Actions::lowerIntake();
  pros::delay(400);
  Robot::chassis->tank(0, 0);
}

auton::Auton auton::autons::offensive = {(char*)("offensive / right"),
                                         runOffensive};