#include "auton.h"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::intakeTriball(const lemlib::Pose pose) {
  Robot::chassis->moveTo(pose.x,
                         pose.y,
                         pose.theta, 5000_ms);
  pros::delay(500);
  while(Robot::chassis->getPose().distance(pose) > 12_in)
    pros::delay(20);
  Robot::Actions::lowerIntake();
  Robot::Actions::intake();
  
  Robot::chassis->waitUntilDone();
  
  pros::delay(1000);
  Robot::chassis->tank(127, 127);
  pros::delay(175);
  Robot::chassis->tank(-127, -127);
  pros::delay(175);
  Robot::chassis->tank(0, 0);
  // Robot::Actions::stopIntake();
  Robot::Actions::raiseIntake();
}