#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"
#include <cmath>

ASSET(disrupt_sweep_txt);
// ASSET(def_elevation_bar_txt);

using namespace fieldDimensions;
using namespace auton::utils;

void runDisrupt() {
  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2, UP},
      false);

  // Robot::chassis->moveToPoint(-TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5, 5000,
  //                             {.minSpeed = 127});
  // waitUntilDistToPose({-TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5}, 10, 500,
  // true); Robot::chassis->cancelMotion();
  // Robot::chassis->moveToPose(-TILE_LENGTH, -8, RIGHT, 5000,
  //                            {.minSpeed = 127, .earlyExitRange = 6});
  // waitUntil([] {
  //   return !isMotionRunning() || Robot::chassis->getPose().y > -TILE_LENGTH;
  // });
  // Robot::Actions::expandBothWings();
  // Robot::Actions::intake();
  // Robot::chassis->cancelMotion();
  // Robot::chassis->moveToPoint(72, 0, 2000, {.minSpeed = 127});
  // pros::delay(500);
  // Robot::Actions::outtake();
  // waitUntilDistToPose({0, 0}, 10, 500, true);
  pros::delay(1000);
  Robot::chassis->follow(disrupt_sweep_txt, 10, 5000);
  pros::delay(100);
  if (robotAngDist(0) > 45)
    Robot::chassis->setPose(Robot::chassis->getPose().x,
                            Robot::chassis->getPose().y, 0);
  Robot::chassis->waitUntil(30);
  Robot::Actions::expandBothWings();
  Robot::Actions::outtake();
  waitUntilDistToPose({-TILE_LENGTH, -8}, 10, 800, true);
  Robot::chassis->cancelMotion();
  if (robotAngDist(180) < 45) {
    Robot::control.print(1, 1, "imu->stop");
    return;
  }

  Robot::Actions::retractBothWings();
  Robot::chassis->moveToPose(
      0 - TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2,
      MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2 - 2.5, UP,
      3000, {.forwards = false});
  Robot::chassis->waitUntilDone();
  Robot::Actions::expandBothWings();
  tank(-127, 127, 0, 0);
  printf("wait for ang\n");
  waitUntil([] { return robotAngDist(DOWN) < 60; });
  printf("retract wings + turn to bar\n");
  Robot::Actions::retractBothWings();
  Robot::chassis->turnTo(100000, 0, 2000);
  Robot::chassis->waitUntilDone();
  printf("move to bar\n");
  // Robot::chassis->moveToPoint(float x, float y, int timeout);
  Robot::chassis->moveToPoint(0, MIN_Y, 3000, {.minSpeed = 64});
  waitUntilDistToPose({-TILE_RADIUS, MIN_Y + TILE_RADIUS}, 12);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(-Robot::Dimensions::drivetrainLength / 2 - 4.6,
                              -TILE_LENGTH * 2 -
                                  Robot::Dimensions::drivetrainWidth / 2 - 4,
                              3000, {.maxSpeed = 64});
  Robot::chassis->waitUntilDone();

  constexpr float angle = (RIGHT + 37.5) * M_PI / 180;
  constexpr float bigNum = 100000000;
  printf("turn to bar\n");
  Robot::chassis->turnTo(bigNum * cos(angle), bigNum * sin(angle), 3000);
  Robot::chassis->waitUntil(70);
  Robot::Actions::expandBothWings();
  Robot::Actions::stopIntake();
  Robot::chassis->waitUntilDone();
  tank(0, 24, 0, 0);
  printf("done\n");

  // Robot::chassis->turnTo(TILE_LENGTH * 2, 0, 1500, true, 40);
  // pros::delay(20);
  // Robot::Pistons::blocker.set_value(false);
  // Robot::chassis->waitUntilDone();
  // pros::delay(250);

  // Robot::Subsystems::catapult->matchload(1000);
  // pros::delay(1000);
  // Robot::Actions::prepareRobot();

  // Robot::Actions::expandBothWings();
  // tank(-127, 127, 750);
  // Robot::Actions::retractBothWings();

  // const lemlib::Pose targetPose {0 - 10, MIN_Y + TILE_RADIUS + 2, RIGHT};

  // Robot::chassis->turnTo(targetPose.x, targetPose.y, 1500);
  // Robot::chassis->waitUntilDone();
  // Robot::chassis->moveToPose(targetPose.x + 20, targetPose.y,
  // targetPose.theta,
  //                            3500, {.maxSpeed = 64});
  // Robot::Actions::outtake();
  // waitUntilDistToPose({targetPose.x + 3, targetPose.y}, 3, 0, true);
  // Robot::chassis->cancelMotion();
  // tank(32, 32, 100);
  // stop();
  // pros::delay(500);
  // Robot::Actions::stopIntake();
}

auton::Auton auton::autons::disrupt = {(char*)("disrupt / left"), runDisrupt};