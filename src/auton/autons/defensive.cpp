#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

// ASSET(def_score_alliance_txt);
// ASSET(def_elevation_bar_txt);

using namespace fieldDimensions;
using namespace auton::utils;

void runDefensive() {
  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2, MIN_Y + TILE_LENGTH - 7.125, UP}, false);

  // Robot::chassis->turnTo(TILE_LENGTH * 2, 0, 1500, true, 40);
  // pros::delay(20);
  // Robot::Pistons::blocker.set_value(false);
  // Robot::chassis->waitUntilDone();
  // pros::delay(250);

  // Robot::Subsystems::catapult->matchload(1000);
  pros::delay(1000);
  // Robot::Actions::prepareRobot();

  Robot::Actions::expandWings();
  tank(-127, 127, 1000);
  Robot::Actions::retractWings();

  // const lemlib::Pose targetPose {0 - 10, MIN_Y + TILE_RADIUS + 2, RIGHT};

  // Robot::chassis->turnTo(targetPose.x, targetPose.y, 1500);
  // Robot::chassis->waitUntilDone();
  // Robot::chassis->moveToPose(targetPose.x + 20, targetPose.y, targetPose.theta,
  //                            3500, {.maxSpeed = 64});
  // Robot::Actions::outtake();
  // waitUntilDistToPose({targetPose.x + 3, targetPose.y}, 3, 0, true);
  // Robot::chassis->cancelMotion();
  // tank(32, 32, 100);
  // stop();
  // pros::delay(500);
  // Robot::Actions::stopIntake();
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"),
                                         runDefensive};