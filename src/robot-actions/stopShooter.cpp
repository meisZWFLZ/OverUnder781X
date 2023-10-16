#include "robot.h"

void Robot::Actions::stopShooter() {
  Robot::Motors::shooter.move_voltage(0);
}