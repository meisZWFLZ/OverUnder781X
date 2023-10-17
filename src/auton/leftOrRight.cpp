#include "auton.h"
#include "robot.h"

int auton::leftOrRight() {
  return Robot::chassis->getPose().x < 0 ? -1 : 1;
}