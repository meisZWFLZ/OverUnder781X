#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void auton::actions::shootTriballIntoOffensiveZone() {
  Robot::chassis->turnToPose(MAX_X-TILE_RADIUS,0_in, 5000_ms, true);
  Robot::chassis->waitUntilDone();
  Robot::Actions::shootMacro();
}