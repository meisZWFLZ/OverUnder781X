#include "robot.h"

void Robot::Actions::intake() {
  Robot::Motors::intake.move_voltage(12000);
}