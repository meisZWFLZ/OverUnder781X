#include "robot.h"

void Robot::Actions::stopShooter() {
  Robot::Motors::topShooter.move(0);
  Robot::Motors::bottomShooter.move(0);
}