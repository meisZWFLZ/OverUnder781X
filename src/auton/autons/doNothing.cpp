#include "auton.h"
#include "lemlib/chassis/chassis.hpp"
#include "robot.h"

void runNothing() {
  // Robot::chassis->swingToHeading(-45, lemlib::DriveSide::RIGHT, 4000);
  Robot::chassis->swingToPoint(-100000, 100000, lemlib::DriveSide::RIGHT, 4000);
  Robot::chassis->waitUntilDone();
  printf("done nothing\n");
}

auton::Auton auton::autons::doNothing = {(char*)("do nothing"), runNothing};