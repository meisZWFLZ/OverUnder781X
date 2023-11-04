#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void runDefensive() {
    Robot::chassis->setPose(nextToMatchLoadZone.x, nextToMatchLoadZone.y, nextToMatchLoadZone.theta, false);
    // @todo add defensive auton
}

auton::Auton auton::autons::defensive = {"defensive / left", runDefensive};