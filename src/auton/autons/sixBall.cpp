#include "auton.h"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

ASSET(ball6_matchload_sweep_txt);
ASSET(ball6_center_plow_1_txt);

void run6Ball() {
  // front left corner of the drivetrain aligned with the inside of the puzzling
  // with alliance triball in intake
  Robot::chassis->setPose(
      {MAX_X - TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainWidth / 2, RIGHT},
      false);

  // keep alliance triball in intake
  Robot::Actions::intake();

  // go into goal
  Robot::chassis->moveToPose(
      MAX_X - TILE_RADIUS,
      0 - TILE_LENGTH - Robot::Dimensions::drivetrainLength, UP, 3000);

  // once near the goal stop intaking
  Robot::chassis->waitUntil(20);
  Robot::Actions::stopIntake();

  Robot::chassis->waitUntilDone();

  // make sure triball goes into goal
  Robot::Actions::outtake();
  tank(127, 127, 500, 0);

  // back out of goal
  tank(-127, -127, 400, 0);
  Robot::Actions::stopIntake();

  // touch horizontal elevation bar
  Robot::chassis->moveToPose(0 + Robot::Dimensions::drivetrainLength / 2 + 1.5,
                             MIN_Y + TILE_RADIUS - 0.5, LEFT, 4000, {.maxSpeed = 64});

  // intake ball under elevation bar
  Robot::Actions::intake();
}

auton::Auton auton::autons::sixBall = {(char*)("6 ball / offensive / right"),
                                       run6Ball};