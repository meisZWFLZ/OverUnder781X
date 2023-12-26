#include "robot.h"

void Robot::Actions::matchload() {
  Robot::Subsystems::catapult->matchload();
}