#include "auton.h"
#include "robot.h"

int auton::leftOrRight(int ifLeft, int ifRight) {
  return Robot::chassis->getPose().x < 0_in ? ifLeft : ifRight;
}