#include "robot.h"

void Robot::Actions::expandBlocker() {
  Robot::Pistons::blocker.set_value(true);
}