#include "auton.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::touchElevationBar() {
  // const lemlib::Pose target {static_cast<float>(leftOrRight() * (0 + TILE_RADIUS - 6)),
  //                        (MIN_Y + TILE_LENGTH * 1.5 - 3.5),
  //                        static_cast<float>(leftOrRight() * (LEFT + 10))};
                         
  // Robot::chassis->moveToPose(target.x,target.y,target.theta, 5000, true, true, 0, 0.7);
  // while(Robot::chassis->getPose().distance(target) > 12)
  //   pros::delay(20);
  // Robot::Actions::raiseIntake();
  // while(Robot::chassis->getPose().distance(target) > 3 || Robot::Motors::leftDrive.at(0).get_voltage() > 2  || Robot::Motors::rightDrive.at(0).get_voltage() > 2)
  //   pros::delay(20);
  
  // pros::delay(500);
  // // Robot::chassis->turnTo(-leftOrRight()*100000, 100000, 5000, false, false, 56);
  // // while(pros::competition::is_autonomous()) {
  // //   pros::delay(10);
  // // }
  // Robot::chassis->tank(leftOrRight(56, -30), leftOrRight(-30, 56),0);
  // // pros::delay(600);
  // // Robot::chassis->tank(0, 0);
}