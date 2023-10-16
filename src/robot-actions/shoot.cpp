#include "robot.h"

void Robot::Actions::shoot() {
  Robot::Motors::shooter.move_voltage(12000);
}