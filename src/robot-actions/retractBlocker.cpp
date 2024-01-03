#include "robot.h"

void Robot::Actions::retractBlocker() {
  Robot::Pistons::blocker.set_value(false);
}