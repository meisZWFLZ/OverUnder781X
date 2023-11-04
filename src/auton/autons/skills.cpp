#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void runSkills() {
    Robot::chassis->setPose(leftStartingPose, false);
    // @todo add skills auton
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
