#include "robot.h"

void Robot::Actions::unshoot() {
  Robot::Motors::topShooter.move(-127);
  Robot::Motors::bottomShooter.move(-127);
}