#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void runDefensive() {
    Robot::chassis->setPose(leftStartingPose, false);
    // @todo add defensive auton
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"), runDefensive};