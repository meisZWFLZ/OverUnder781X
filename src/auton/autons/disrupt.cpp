#include "auton.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

void runDisrupt() {
  // front of drivetrain should be aligned with closer edge of puzzle pattern
  // x is where neil decided so 
  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2 + 6,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2, UP},
      false);

  // position the robot to disrupt center triballs
  Robot::chassis->moveToPose(-TILE_LENGTH - 3, -TILE_LENGTH + 6, RIGHT, 3000,
                             {.lead = .3, .minSpeed = 64});
  // wait until facing right to push triballs                             
  waitUntil([] { return robotAngDist(RIGHT) < 20 || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  // prepare to push triballs
  Robot::Actions::expandBothWings();
  Robot::Actions::outtake();

  // full speed into the triballs
  Robot::chassis->moveToPoint(10000000, 0, 3000, {.minSpeed = 127});

  // wait until near barrier
  waitUntil([] {
    return Robot::chassis->getPose().x > -12.5 || !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // done disrupting
  // position the robot to clear matchload zone
  Robot::chassis->moveToPoint(-TILE_LENGTH * 2 - 3, -TILE_LENGTH * 2 - 6, 3000,
                              {.forwards = false});

  // retract wings
  Robot::chassis->waitUntil(5);
  Robot::Actions::retractBothWings();

  Robot::chassis->waitUntilDone();

  // turn to be parallel to the matchload pipe
  Robot::chassis->turnTo(1000000, -1000000, 2000);
  Robot::chassis->waitUntilDone();

  // expand wing
  Robot::Actions::expandBackWing();

  // move in an arc to sweep ball out
  tank(32, 127, 0, 0);
  // wait until facing right or if we are far from the matchload zone
  waitUntil(
      [] {
        return robotAngDist(RIGHT) < 10 ||
               Robot::chassis->getPose().x > -TILE_LENGTH * 1.5;
      },
      0, 1000);
  stop();

  // ensure we don't bend back wing
  Robot::Actions::retractBackWing();

  // intake any straggler balls
  Robot::Actions::intake();

  // let matchload zone triball get ahead of us
  pros::delay(1000);

  // touch horizontal elevation bar
  Robot::chassis->moveToPose(0 - Robot::Dimensions::drivetrainLength / 2 - 3.5,
                             MIN_Y + TILE_RADIUS, RIGHT, 2000);

  // if a triball enters the intake, outtake it
  while (pros::competition::is_autonomous()) {
    if (isTriballInIntake()) {
      Robot::Actions::outtake();
      pros::delay(500);
    }
    pros::delay(10);
  }
}

auton::Auton auton::autons::disrupt = {(char*)("disrupt / left"), runDisrupt};