#include "robot.h"

void Robot::Actions::stopMatchloading() {
  Robot::Subsystems::catapult->stopMatchloading();
}