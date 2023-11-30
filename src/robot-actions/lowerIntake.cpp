#include "robot.h"

void Robot::Actions::lowerIntake() {
  Robot::Pistons::intakeElevator.set_value(false);
}