#include "auton.h"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::intakeTriball(const lemlib::Pose pose) {
  Robot::chassis->moveTo(pose.x,
                         pose.y,
                         pose.theta, 5000, true);
  pros::delay(500);
  while(Robot::chassis->getPose().distance(pose) > 8)
    pros::delay(20);
  Robot::Actions::intake();
  while(Robot::chassis->getPose().distance(pose) > 3 || Robot::Motors::leftDrive.at(0).get_voltage() > 2  || Robot::Motors::rightDrive.at(0).get_voltage() > 2)
    pros::delay(20);
  pros::delay(500);
  Robot::chassis->tank(127, 127);
  pros::delay(175);
  Robot::chassis->tank(-127, -127);
  pros::delay(175);
  Robot::chassis->tank(0, 0);
  Robot::Actions::stopIntake();
}