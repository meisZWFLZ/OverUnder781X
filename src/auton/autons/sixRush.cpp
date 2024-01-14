#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

// ASSET(def_score_alliance_txt);
ASSET(rush_6_intake_txt);
ASSET(rush_6_elevation_triball_txt);
ASSET(ball6_matchload_sweep_txt);

using namespace fieldDimensions;
using namespace auton::utils;

// bool waitUntilMotionDone(int timeout = -1) {
//   const int startTime = pros::millis();
//   while (Robot::chassis->distTravelled != -1 &&
//          (timeout == -1 || pros::millis() - startTime < timeout)) {
//     printf("dist: %f\n", Robot::chassis->distTravelled);
//     pros::delay(100);
//   }111111111111
// }

void runSixRush() {
  using namespace fieldDimensions;
  using namespace auton::actions;

  Robot::chassis->setPose(
      {0 + TILE_LENGTH * 1.5, MIN_Y + TILE_LENGTH - 12.0 / 2, UP}, false);
  Robot::Actions::expandWings();
  Robot::chassis->follow(rush_6_intake_txt, 13, 2750);
  pros::delay(300);
  Robot::Actions::retractWings();
  Robot::chassis->waitUntil(36);
  Robot::Actions::intake();

  waitUntilDistToPose({9, -9}, 3);
  Robot::chassis->cancelMotion();

  Robot::chassis->waitUntilDone();

  tank(127, -64, 70, 0);

  Robot::chassis->turnTo(10000, 1000, 1000);
  // pros::delay(100);
  // while (Robot::chassis->isInMotion() && robotAngDist(RIGHT) < 5) {
  //   pros::delay(10);
  // }
  // Robot::chassis->cancelMotion();

  Robot::chassis->waitUntilDone();

  Robot::Actions::expandWings();
  tank(127, 127, 300, 0);
  Robot::Actions::outtake();
  tank(127, 127, 600, 0);
  tank(-48, -127, 300, 0);
  Robot::Actions::retractWings();
  Robot::Actions::stopIntake();

  const lemlib::Pose targetPose {10, -24};
  Robot::chassis->turnTo(targetPose.x, targetPose.y, 150);
  Robot::chassis->waitUntilDone();
  tank(127, 127, 350, 127.0 / 18);
  Robot::chassis->moveToPoint(targetPose.x, targetPose.y, 2000);

  waitUntilDistToPose(targetPose, 10, 0, true);
  Robot::Actions::intake();
  pros::delay(500);
  Robot::chassis->cancelMotion();
  tank(-127, -64, 200, 0);
  
  Robot::chassis->follow(rush_6_elevation_triball_txt, 7, 5000);

  Robot::chassis->waitUntil(10);
  Robot::Actions::outtake();
  Robot::chassis->waitUntil(20);
  Robot::Actions::intake();

  pros::delay(150);

  tank(-64, -127, 100, 0);
  Robot::chassis->turnTo(10000, 0, 1000);
  Robot::chassis->waitUntilDone();
  Robot::Actions::stopIntake();
  printf("turn done\n");

  //
  //
  //
  //
  //
  //
  //
  //
  // Path to smoothly remove triball in matchload zone and from the matchload
  // zone and plow the three balls into the goal
  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the matchload zone to expand wings
  waitUntilDistToPose({MAX_X - TILE_LENGTH, MIN_Y + TILE_LENGTH - 3}, 6, 0,
                      true);
  Robot::Actions::expandWings();
  printf("expand wings\n");
  pros::delay(200);
  Robot::chassis->cancelMotion();

  // retract wings soon after removing the triball from the matchload zone
  tank(24, 36, 200);
  Robot::Actions::retractWings();
  printf("retract wings\n");
  tank(24, 36, 200);

  printf("back to follow\n");

  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the goal to outtake

  pros::delay(450);
  Robot::Actions::outtake();
  printf("outtake\n");

  while ((std::abs(Robot::chassis->getPose().x - (MAX_X - TILE_RADIUS)) > 10 ||
          robotAngDist(0) > 20) &&
         Robot::chassis->isInMotion() &&
         Robot::chassis->getPose().theta < 420) {
    pros::delay(10);
  }
  printf("cancel follow\n");
  // fully ram into goal
  Robot::chassis->cancelMotion();
  tank(127, 100, 0, 0);
  waitUntilDistToPose({MAX_X - TILE_RADIUS, -TILE_LENGTH}, 10, 0, false);
  pros::delay(100);
  tank(-127, -127, 200, 0);
  tank(90, 127, 500, 0);
  pros::delay(150);
}

auton::Auton auton::autons::sixRush = {
    (char*)("6 ball rush / offensive / right"), runSixRush};