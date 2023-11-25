#include "robot.h"

void Robot::Actions::shoot() {
  Robot::Motors::topShooter.move(127);
  Robot::Motors::bottomShooter.move(127);
}