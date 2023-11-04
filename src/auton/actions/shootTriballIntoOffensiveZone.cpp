#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
void auton::actions::shootTriballIntoOffensiveZone() {
  Robot::chassis->turnTo(MAX_X-TILE_RADIUS,0, 5000, false, true);
  Robot::Actions::shootMacro();
}