#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::prepareForMatchloading() {
   Robot::chassis->moveTo(MIN_X + TILE_LENGTH - 8, MIN_Y + TILE_LENGTH - 2,
                         LEFT - 22.5, 3000);

  Robot::chassis->waitUntil(6);
  Robot::Actions::shoot();
  Robot::chassis->waitUntilDone();
  Robot::Actions::lowerIntake();

}