#include "robot.h"

void Robot::Actions::retractWings() {
  Robot::Pistons::wings.set_value(false);
}