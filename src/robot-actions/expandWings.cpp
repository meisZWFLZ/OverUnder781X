#include "robot.h"

void Robot::Actions::expandWings() {
  Robot::Pistons::wings.set_value(true);
}