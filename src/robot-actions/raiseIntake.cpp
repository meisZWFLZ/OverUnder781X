#include "robot.h"

void Robot::Actions::raiseIntake() {
  Robot::Pistons::intakeElevator.set_value(false);
}