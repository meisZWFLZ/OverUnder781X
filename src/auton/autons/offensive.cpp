#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void runOffensive() {
    Robot::chassis->setPose(-nextToMatchLoadZone.x, nextToMatchLoadZone.y, nextToMatchLoadZone.theta, false);
    // @todo add offensive auton
}

auton::Auton auton::autons::offensive = {"offensive / right", runOffensive};