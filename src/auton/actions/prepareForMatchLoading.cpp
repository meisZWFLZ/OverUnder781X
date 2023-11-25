#include "auton.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::prepareForMatchloading() {
  Robot::Actions::lowerIntake();
  Robot::Actions::shoot();
  Robot::chassis->moveTo(MIN_X + TILE_RADIUS + 1_in, MIN_Y + TILE_LENGTH - 5_in, -112_deg,
                         5000_ms, false, 0, 0.4);
  // Robot::chassis->moveTo(target.x,target.y,target.theta, 5000, true, true, 0,
  // 0.7); while(Robot::chassis->getPose().distance(target) > 12)
  //   pros::delay(20);
  // Robot::Actions::expandWings();
  // while(Robot::chassis->getPose().distance(target) > 3 ||
  // Robot::Motors::leftDrive.at(0).get_voltage() > 2  ||
  // Robot::Motors::rightDrive.at(0).get_voltage() > 2)
  //   pros::delay(20);

  // pros::delay(500);
  // // Robot::chassis->turnTo(-leftOrRight()*100000, 100000, 5000, false,
  // false, 56);
  // // while(pros::competition::is_autonomous()) {
  // //   pros::delay(10);
  // // }
  // Robot::chassis->tank(leftOrRight(-30, 56), leftOrRight(56, -30), 0);
  // // pros::delay(600);
  // // Robot::chassis->tank(0, 0);
}