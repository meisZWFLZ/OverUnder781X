#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void auton::actions::scoreAllianceTriball() {
  Robot::chassis->moveTo(/* leftOrRight()* */60/* (MAX_X - TILE_RADIUS) */,-45/* (MIN_Y + ( TILE_LENGTH*1.5 - 9)) */, UP, 5000,false,true,0,0.6);
  pros::delay(100);
  Robot::Actions::outtake();
  pros::delay(250);
  Robot::chassis->tank(127,127);
  pros::delay(100);
  Robot::Actions::stopIntake();
  pros::delay(900);
  Robot::chassis->tank(-32,-32);
  pros::delay(500);
  Robot::chassis->tank(0,0);

}