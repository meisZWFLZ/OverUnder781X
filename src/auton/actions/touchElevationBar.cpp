#include "auton.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::touchElevationBar() {
  const lemlib::Pose target {leftOrRight() * (0_in + TILE_RADIUS - 6_in),
                         (MIN_Y + TILE_LENGTH * 1.5 - 3.5_in),
                         leftOrRight() * (LEFT + 10_deg)};
                         
  Robot::chassis->moveTo(target.x,target.y,target.theta, 5000_ms, false, 0, 0, 0.7);
  while(Robot::chassis->getPose().distance(target) > 12_in)
    pros::delay(20);
  Robot::Actions::expandWings();
  Robot::chassis->waitUntilDone();
  
  pros::delay(500);
  // Robot::chassis->turnTo(-leftOrRight()*100000, 100000, 5000, false, false, 56);
  // while(pros::competition::is_autonomous()) {
  //   pros::delay(10);
  // }
  Robot::chassis->tank(leftOrRight(-30, 56), leftOrRight(56, -30), 0);
  // pros::delay(600);
  // Robot::chassis->tank(0, 0);
}