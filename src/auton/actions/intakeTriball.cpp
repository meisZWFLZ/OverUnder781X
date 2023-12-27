#include "auton.h"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::intakeTriball(const lemlib::Pose pose) {
  // Robot::chassis->moveToPose(pose.x,
  //                        pose.y,
  //                        pose.theta, 5000, true);
  // pros::delay(500);
  // while(Robot::chassis->getPose().distance(pose) > 12)
  //   pros::delay(20);
  // Robot::Actions::lowerIntake();
  // Robot::Actions::intake();
  // while(Robot::chassis->getPose().distance(pose) > 6 || Robot::Motors::leftDrive.at(0).get_voltage() > 1.5  || Robot::Motors::rightDrive.at(0).get_voltage() > 1.5)
  //   pros::delay(20);
  // pros::delay(1000);
  // Robot::chassis->tank(127, 127);
  // pros::delay(175);
  // Robot::chassis->tank(-127, -127);
  // pros::delay(175);
  // Robot::chassis->tank(0, 0);
  // // Robot::Actions::stopIntake();
  // Robot::Actions::raiseIntake();
}