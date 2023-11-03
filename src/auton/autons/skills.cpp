#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void runSkills() {
    Robot::chassis->setPose(nextToMatchLoadZone.x, nextToMatchLoadZone.y, nextToMatchLoadZone.theta, false);
    // @todo add skills auton
}