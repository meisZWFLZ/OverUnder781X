#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

// ASSET(def_score_alliance_txt);
ASSET(rush_6_intake_txt);
ASSET(rush_6_elevation_triball_txt);
ASSET(rush_6_matchload_sweep_txt);

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
      {0 + TILE_LENGTH * 1.5 + 2, MIN_Y + TILE_LENGTH - 12.0 / 2 + 3, UP},
      false);
  Robot::Actions::expandWings();

  Robot::chassis->follow(rush_6_intake_txt, 13, 2750);
  pros::delay(300);
  Robot::Actions::retractWings();
  Robot::chassis->waitUntil(36);
  Robot::Actions::intake();

  waitUntilDistToPose({9, -6}, 3);
  Robot::chassis->cancelMotion();
  pros::delay(100);

  tank(127, -64, 70, 0);

  Robot::chassis->turnTo(10000, 1000, 1000);
  pros::delay(100);
  while (Robot::chassis->isInMotion() && robotAngDist(RIGHT - 5) > 3) {
    pros::delay(10);
  }
  Robot::chassis->cancelMotion();

  stop();
  Robot::Actions::expandWings();
  tank(127, 127, 300, 127 / 18.0);
  Robot::Actions::outtake();
  tank(127, 127, 600, 127 / 18.0);

  tank(-48, -127, 300, 0);
  Robot::Actions::retractWings();
  Robot::Actions::stopIntake();

  const lemlib::Pose thirdTriball {10, -24};
  Robot::chassis->turnTo(thirdTriball.x, thirdTriball.y, 150);
  Robot::chassis->waitUntilDone();
  tank(127, 127, 200, 127.0 / 18);
  // stop();
  // return;
  Robot::chassis->moveToPoint(thirdTriball.x, thirdTriball.y, 2000,
                              {.minSpeed = 30});
  Robot::Actions::intake();

  waitUntil(isTriballInIntake, 30, 2000, true);
  printf("triball in intake %i\n", isTriballInIntake());

  Robot::chassis->cancelMotion();
  // Robot::chassis->setPose({Robot::chassis->getPose().x,
  //                          Robot::chassis->getPose().y,
  //                          Robot::chassis->getPose().theta + 2.0F},
  //                         false);
  tank(-127, -127, 100, 0);

  Robot::chassis->moveToPose(TILE_LENGTH * 1.5 + 2, MIN_Y + TILE_LENGTH - 9,
                             UP + 45, 2000,
                             {
                                 .forwards = false,
                                 .minSpeed = 72,
                                 .earlyExitRange = 8,
                             });
  // Robot::chassis->setPose({Robot::chassis->getPose().x,
  //                          Robot::chassis->getPose().y,
  //                          Robot::chassis->getPose().theta + 7.5F},
  //                         false);
  // stop();
  // return;
  // Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();
  Robot::Actions::outtake();
  tank(-127, -127, 150, 0);
  // tank(32, -127, 0, 0);
  // waitUntil([] { return robotAngDist(LEFT) < 35; });
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 10, 5000);
  // pros::delay(10);
  // Robot::chassis->waitUntil(3);

  // waitUntil([] { return !isMotionRunning() || robotAngDist(RIGHT) < 20; },
  // 100,
  //           5000, false);
  // Robot::chassis->cancelMotion();
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 5, 5000);

  // Robot::chassis->cancelMotion();
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 8, 5000);

  // waitUntil([] { return !isMotionRunning() || robotAngDist(LEFT) < 10; },
  // 100,
  //           5000, false);
  // printf("theta: %4.2f\n", Robot::chassis->getPose().theta);

  Robot::chassis->moveToPoint(-10, MIN_Y + TILE_RADIUS + 2, 3000);
  pros::delay(100);

  waitUntilDistToPose({12, MIN_Y + TILE_RADIUS + 2}, 5, 0, true);
  printf("slow down\n");
  printf("pose: (%4.2f,%4.2f,%4.2f)\n", Robot::chassis->getPose().x,
         Robot::chassis->getPose().y, Robot::chassis->getPose().theta);

  Robot::chassis->cancelMotion();
  Robot::chassis->moveToPoint(8, MIN_Y + TILE_RADIUS + 2, 3000,
                              {.maxSpeed = 48, .minSpeed = 15});

  Robot::Actions::intake();
  waitUntil([] { return isTriballInIntake() || !isMotionRunning(); }, 100, 5000,
            true);
  printf("triball in intake %i\n", isTriballInIntake());
  printf("pose: (%4.2f,%4.2f,%4.2f)\n", Robot::chassis->getPose().x,
         Robot::chassis->getPose().y, Robot::chassis->getPose().theta);
  Robot::chassis->cancelMotion();

  tank(-127, -127, 250, 0);

  tank(64, -127, 0, 0);
  waitUntil([] { return robotAngDist(RIGHT) < 20; });

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
  Robot::chassis->follow(rush_6_matchload_sweep_txt, 13, 2750);

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

  Robot::chassis->follow(rush_6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the goal to outtake

  pros::delay(450);
  Robot::Actions::outtake();
  printf("outtake\n");

  while ((std::abs(Robot::chassis->getPose().x - (MAX_X - TILE_RADIUS)) > 10 ||
          robotAngDist(0) > 20) &&
         Robot::chassis->isInMotion()) {
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

  tank(-127, 32, 0, 0);
  waitUntil([] { return robotAngDist(LEFT) < 35; });

  Robot::chassis->moveToPoint(TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5, 3000,
                              {.minSpeed = 48, .earlyExitRange = 100});
  Robot::chassis->moveToPose(TILE_RADIUS - 2, -TILE_LENGTH * 1.5 - 2, LEFT - 15,
                             2000);
  Robot::chassis->waitUntilDone();
  Robot::Actions::expandWings();
  tank(127, -127, 0, 0);
  waitUntil([] { return robotAngDist(LEFT + 45) < 10; });
  stop();
}

auton::Auton auton::autons::sixRush = {
    (char*)("6 ball rush / offensive / right"), runSixRush};