#include "robot.h"

void Robot::Actions::stopIntake() {
  Robot::Motors::intake.move_voltage(0);
}