#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void delayForMatchLoading(int delay) {
  printf("delay: %i\n", delay);
  pros::delay(delay - 3000);
  Robot::control.rumble(".");
  pros::delay(1000);
  Robot::control.rumble(".");
  pros::delay(1000);
  Robot::control.rumble(".");
  pros::delay(1000);
  Robot::control.rumble("-");
}

void runSkills() {
  printf("skills\n");
  using namespace fieldDimensions;
  using namespace auton::actions;
  Robot::chassis->setPose(leftStartingPose);
  // @todo add skills auton

  prepareForMatchloading();
  Robot::chassis->tank(24, 0);
  // pros::delay(500);
  // Robot::chassis->tank(20, 0);
  delayForMatchLoading(45000);
  Robot::Actions::raiseIntake();
  Robot::Actions::stopShooter();
  const lemlib::Pose pastBarrier {0_in - TILE_LENGTH * 1.5, MIN_Y + TILE_RADIUS};
  const lemlib::Pose pastElevationBar {0_in + TILE_LENGTH, MIN_Y + TILE_RADIUS};
  printf("goToBarrier\n");
  // while (Robot::chassis->getPose().distance(pastBarrier) > 3)
  Robot::chassis->moveTo(pastBarrier.x, pastBarrier.y, RIGHT, 2000_ms, true);
  Robot::chassis->waitUntilDone();
  Robot::chassis->turnToPose(pastElevationBar.x, pastElevationBar.y, 5000_ms);
  Robot::chassis->waitUntilDone();

  Robot::chassis->tank(-127, -127);
  while (Robot::chassis->getPose().x < 0_in + TILE_LENGTH) pros::delay(20);
  printf("goToStagingPoint\n");
  Robot::chassis->moveTo(MAX_X - TILE_LENGTH * 1.5, MIN_Y + TILE_RADIUS, UP,
                         7500_ms);
  Robot::chassis->waitUntilDone();
  Robot::chassis->moveTo(0_in - TILE_LENGTH, -5_in, RIGHT, 7500_ms);
  Robot::chassis->waitUntilDone();
  Robot::Actions::expandWings();
  Robot::chassis->turnToPose(100000_in, 0_in, 2500_ms);
  Robot::chassis->waitUntilDone();
  Robot::chassis->tank(127, 127);
  pros::delay(1000);
  Robot::chassis->tank(-127, -127);
  pros::delay(500);
  Robot::chassis->tank(127, 127);
  pros::delay(1000);
  Robot::chassis->tank(-127, -127);
  pros::delay(1000);
  Robot::chassis->tank(0, 0);
  Robot::Actions::retractWings();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
