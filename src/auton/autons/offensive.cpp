#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void runOffensive() {
  Robot::chassis->setPose(rightStartingPose, false);
  // @todo add offensive auton

  auton::actions::scoreAllianceTriball();
  auton::actions::touchElevationBar();
}

auton::Auton auton::autons::offensive = {(char*)("offensive / right"),
                                         runOffensive};