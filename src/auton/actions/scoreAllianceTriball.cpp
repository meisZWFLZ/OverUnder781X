#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::scoreAllianceTriball() {
  Robot::chassis->moveTo(leftOrRight() * (MAX_X - TILE_RADIUS + 1_in),
                         (MIN_Y + (TILE_LENGTH * 1.5 - 3_in)), UP, 5000_ms, false,
                         0, 0.2);
  Robot::Actions::outtake();
  pros::delay(200);
  Robot::chassis->tank(127, 127);
  pros::delay(100);
  Robot::Actions::stopIntake();
  pros::delay(250);
  Robot::chassis->tank(-127, -172);
  pros::delay(225);
  Robot::chassis->tank(0, 0);
}