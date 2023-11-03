#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void auton::actions::shootTriballIntoOffensiveZone() {
  Robot::chassis->turnTo(MAX_X-TILE_RADIUS,0, 5000, false, true);
  Robot::Actions::shoot();
  Robot::Actions::intake();
  pros::delay(200);
  Robot::chassis->tank(-127,-127);
  pros::delay(100);
  Robot::chassis->tank(127,127);
  pros::delay(100);
  Robot::chassis->tank(0,0);
  pros::delay(350);
  Robot::Actions::stopShooter();
  Robot::Actions::stopIntake();
}