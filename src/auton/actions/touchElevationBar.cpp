#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void auton::actions::touchElevationBar() {
  Robot::chassis->moveTo(
      -(0 + TILE_LENGTH + Robot::Dimensions::drivetrainLength / 2),
      (MIN_Y + TILE_LENGTH + 1.35 + Robot::Dimensions::drivetrainWidth), RIGHT,
      5000);
  // Robot::chassis->moveTo(/* auton?::leftOrRight()*
  // */0-Robot::Dimensions::drivetrainLength/2, MIN_Y + TILE_RADIUS, RIGHT,
  // 5000);
  Robot::chassis->tank(127, 127);;
  // Robot::chassis->moveTo(-(0 + Robot::Dimensions::drivetrainLength/2),(MIN_Y
  // + TILE_LENGTH + 1 + Robot::Dimensions::drivetrainWidth), RIGHT, 5000, 0, 1,
  // 0, 0);
  pros::delay(500);
  Robot::chassis->tank(24, 24);
  pros::delay(400);
  Robot::chassis->tank(0, 0);
}