#include "robot.h"

void Robot::Actions::stopShooter() {
  Robot::Subsystems::catapult->stop();
}