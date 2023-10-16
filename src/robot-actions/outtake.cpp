#include "robot.h"

void Robot::Actions::outtake() {
  Robot::Motors::intake.move_voltage(-12000);
}