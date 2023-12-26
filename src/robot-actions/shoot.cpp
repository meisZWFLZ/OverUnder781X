#include "robot.h"

void Robot::Actions::shoot() {
  Robot::Subsystems::catapult->fire();
}