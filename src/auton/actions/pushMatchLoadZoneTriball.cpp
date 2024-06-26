#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::pushMatchLoadZoneTriball() {
  // const lemlib::Pose target = Robot::chassis->getPose() + lemlib::Pose(0, 12, 0);
  // printf("y:%f\n", target.y);
  // Robot::chassis->moveToPose(target.x, target.y, target.theta, 5000, true, 0,
  //                        0.2);
  // Robot::Actions::expandBothWings();
  Robot::chassis->tank(127, 172);
  pros::delay(100);
  Robot::chassis->tank(0, 0);
  pros::delay(196);
  Robot::Actions::outtake();
  pros::delay(54);
  Robot::chassis->tank(-127, 172);
  pros::delay(500);
  Robot::Actions::stopIntake();
  Robot::chassis->tank(0, 0);
  Robot::Actions::retractBothWings();
}