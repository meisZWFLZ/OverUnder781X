#include "robot.h"

void Robot::Actions::unshoot() {
  Robot::Motors::shooter.move_voltage(-12000);
}