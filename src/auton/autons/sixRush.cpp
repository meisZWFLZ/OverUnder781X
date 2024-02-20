#include "auton.h"
#include "lemlib/asset.hpp"
#include "lemlib/util.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"
#include <cmath>

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

void printRobotPose() {
  const lemlib::Pose pose = Robot::chassis->getPose();

  printf("pose: (%4.2f,%4.2f,%4.2f)\n", pose.x, pose.y, pose.theta);
}

void test() {
  // // test imus
  // Robot::chassis->setPose(0, 0, 0);
  // Robot::chassis->moveToPoint(0, 24, 10000);
  // Robot::chassis->waitUntilDone();
  // tank(-127, 127, 0, 0);
  // waitUntil([] { return std::abs(Robot::chassis->getPose().theta - -360 * 10)
  // < 70; }); Robot::chassis->turnTo(0, 100000000, 1000);
  // Robot::chassis->waitUntilDone();
  // stop();

  // test pid
  Robot::chassis->setPose(0, 0, 0);
  Robot::chassis->turnTo(1000000, 0, 10000);
}

void runSixRush() {
  using namespace fieldDimensions;
  using namespace auton::actions;

  const int startTime = pros::millis();

  // same set as disrupt
  Robot::chassis->setPose(
      {0 - (- TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2 + 6),
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2, UP},
      false);

  // intake center triball next to barrier
  Robot::Actions::intake();
  Robot::chassis->moveToPose(Robot::Dimensions::drivetrainLength / 2, 0, LEFT,
                             3000, {.lead=0.2,.minSpeed = 127});
  pros::delay(200);

  // wait until triball in intake
  waitUntil([] { return isTriballInIntake() || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(72, 0, 3000, {.forwards=false,.minSpeed = 127});
  pros::delay(500);

  // wait until robot is stopped by goal
  waitUntil([] {
    static float lastX = 10000000;
    pros::delay(30);
    float diff = std::abs(lastX - Robot::chassis->getPose().x);
    diff *= 20;
    const bool out = diff < 0.5 ||
                     !isMotionRunning();
    printf("diff: %f\n", diff);
    lastX = Robot::chassis->getPose().x;
    
    return out;
  });
  Robot::chassis->cancelMotion();

  tank(127, 127, 200);
  Robot::chassis->turnTo(1000000, 0, 1000);
  Robot::chassis->waitUntilDone();
  Robot::Actions::outtake();
  Robot::chassis->moveToPoint(1000000, 0, 1000, {.minSpeed = 127});
  pros::delay(750);

  // wait until robot is stopped by goal
  waitUntil([] {
    static float lastX = 10000000;
    const float diff = std::abs(lastX - Robot::chassis->getPose().x);
    const bool out = diff < 0.002 ||
                     !isMotionRunning();
    printf("diff: %f\n", diff);
    lastX = Robot::chassis->getPose().x;
    return out;
  });
  Robot::chassis->cancelMotion();
  stop();

  return;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Everything below is old code that is not used in the current auton
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Robot::chassis->setPose(
  //     {0 + TILE_LENGTH * 2 -8, -TILE_LENGTH * 2 - 5.42, UP},
  //     false);

  // Robot::chassis->moveToPoint(Robot::chassis->getPose().x,
  //                             48 + Robot::chassis->getPose().y, 10000,
  //                             {.maxSpeed = 32});
  // return;
  // Robot::Actions::expandBothWings();
  // tank(64,127,5000,0);
  Robot::chassis->moveToPose(0, 4, LEFT + 45, 5000, {.minSpeed = 127});
  // Robot::chassis->follow(rush_6_intake_txt, 13, 2500);
  pros::delay(10);
  if (robotAngDist(0) > 45)
    Robot::chassis->setPose(
        {Robot::chassis->getPose().x, Robot::chassis->getPose().y, UP}, false);
  pros::delay(300);

  Robot::chassis->waitUntil(36);

  Robot::Actions::intake();

  // waitUntilDistToPose({9, -6}, 3);
  waitUntil(
      [] {
        return /* Robot::chassis->getPose().distance({9, 0}) < 6 || */
            isTriballInIntake() || !Robot::chassis->isInMotion();
      },
      300, 5000);

  Robot::chassis->cancelMotion();
  pros::delay(100);
  if (robotAngDist(UP - 45) > 45) {
    Robot::control.print(1, 1, "imu->stop");
    return;
  }
  // Robot::chassis->turnTo(10000, 1000, 1000);
  // pros::delay(100);
  // while (Robot::chassis->isInMotion() && robotAngDist(RIGHT) > 10) {
  //   pros::delay(10);
  // }
  // Robot::chassis->cancelMotion();

  // stop();
  // Robot::Actions::expandBothWings();
  Robot::chassis->moveToPoint(96, 0, 1000,
                              {
                                  .forwards = false,
                                  .minSpeed = 127,
                              });
  pros::delay(300);
  // Robot::Actions::outtake();

  Robot::chassis->waitUntilDone();
  // Robot::Actions::retractBothWings();
  Robot::Actions::stopIntake();

  tank(64, 127, 300);
  Robot::chassis->turnTo(100000, 0, 700);
  Robot::chassis->moveToPoint(10000, 0, 1000, {.minSpeed = 127});
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();
  // printRobotPose();
  // Robot::chassis->setPose(Robot::chassis->getPose().x + 1,
  //                         Robot::chassis->getPose().y - 3,
  //                         Robot::chassis->getPose().theta);
  // printRobotPose();
  // return;

  // const lemlib::Pose thirdTriball {2, -24, LEFT - 30};
  tank(-127, -127, 200, 0);
  tank(64, -127, 300, 0);
  Robot::Actions::stopIntake();
  Robot::chassis->moveToPose(0, -TILE_LENGTH, LEFT, 3000);
  Robot::Actions::intake();
  pros::delay(500);
  waitUntil([] { return isTriballInIntake() || !isMotionRunning(); }, 0, 3000);
  Robot::chassis->cancelMotion();
  pros::delay(300);

  if (isTriballInIntake()) {
    tank(-127, -127, 300);
    Robot::chassis->turnTo(100000, 0, 1000);
    waitUntil([] { return robotAngDist(RIGHT) < 30 || !isMotionRunning(); }, 0,
              1000);
    Robot::chassis->cancelMotion();
    Robot::chassis->moveToPose(TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5, RIGHT,
                               3000, {.minSpeed = 32, .earlyExitRange = 5});
    Robot::chassis->waitUntil(18);
    Robot::Actions::outtake();
    Robot::chassis->waitUntilDone();

    // Robot::chassis->moveToPoint(TILE_LENGTH * 3, 0, 3000, {.minSpeed = 127});
    // pros::delay(500);
    // lemlib::Pose lastPose = Robot::chassis->getPose();
    // Robot::Actions::outtake();
    // waitUntil(
    //     [&lastPose] {
    //       return Robot::chassis->getPose().distance(lastPose) < 5 ||
    //              isMotionRunning();
    //       lastPose = Robot::chassis->getPose();
    //     },
    //     300, 3000);
    // Robot::chassis->cancelMotion();
    // tank(-127, -64, 500, 0);
    // stop();
  }

  // waitUntil([&thirdTriball] {
  //   const lemlib::Pose pose = Robot::chassis->getPose();
  //   printf("theta: %f\n", pose.theta);
  //   const float angTo = pose.angle(thirdTriball) * 360 / M_PI_2;
  //   const float angErr = fabs(std::remainder(angTo - pose.theta, 360));
  //   printf("angErr: %f\n", angErr);
  //   printf("angTo: %f\n", angTo);
  //   return angErr < 30;
  // });
  // Robot::chassis->waitUntilDone();
  // Robot::chassis->moveToPose(thirdTriball.x, thirdTriball.y,
  // thirdTriball.theta,
  //                            3000,
  //                            {
  //                                .chasePower = 8,
  //                                .minSpeed = 80,
  //                            });
  // Robot::Actions::intake();

  // waitUntil(isTriballInIntake, 50, 3000, true);
  // printf("triball in intake %i\n", isTriballInIntake());

  // Robot::chassis->cancelMotion();
  // // Robot::chassis->setPose({Robot::chassis->getPose().x,
  // //                          Robot::chassis->getPose().y,
  // //                          Robot::chassis->getPose().theta + 2.0F},
  // //                         false);
  // tank(-127, -64, 100, 0);
  // tank(-127, 127, 0, 0);

  // waitUntil([] { return robotAngDist(RIGHT) < 100; });

  // Robot::chassis->moveToPose(TILE_LENGTH * 1.3, -TILE_LENGTH * 1.3, RIGHT -
  // 10,
  //                            2000,
  //                            {
  //                                .forwards = true,
  //                                .minSpeed = 127,
  //                                .earlyExitRange = 6,
  //                            });
  // Robot::chassis->setPose({Robot::chassis->getPose().x,
  //                          Robot::chassis->getPose().y,
  //                          Robot::chassis->getPose().theta + 7.5F},
  //                         false);
  // stop();
  // return;
  // Robot::Actions::outtake();
  // waitUntil([] {
  //   return (Robot::chassis->getPose().distance(
  //               {TILE_LENGTH, -TILE_LENGTH * 1.3}) < 10 &&
  //           robotAngDist(RIGHT) < 20) ||
  //          !Robot::chassis->isInMotion();
  // });
  // waitUntilDistToPose({TILE_LENGTH, -TILE_LENGTH * 1.3}, 6, 300, true);
  // Robot::Actions::outtake();
  // Robot::chassis->cancelMotion();
  // tank(0, 0, 500);
  // printRobotPose();

  Robot::chassis->moveToPoint(TILE_LENGTH * 2 - 2, -TILE_LENGTH * 2 + 2, 3000);
  pros::delay(500);
  Robot::Actions::intake();
  waitUntil([] { return isTriballInIntake() || !Robot::chassis->isInMotion(); },
            50);
  Robot::chassis->cancelMotion();

  // orient towards goal
  Robot::chassis->moveToPose(TILE_LENGTH * 2.6, -TILE_LENGTH * 1.5, UP, 3000,
                             {.minSpeed = 40, .earlyExitRange = 6});
  waitUntil(
      [] { return (robotAngDist(UP) < 10) || !Robot::chassis->isInMotion(); });
  printRobotPose();
  printf("inMotion: %i\n", Robot::chassis->isInMotion());
  Robot::chassis->cancelMotion();
  Robot::chassis->waitUntilDone();
  Robot::Actions::outtake();

  // tank(-127, -127, 130, 0);
  // tank(32, -127, 0, 0);
  // waitUntil([] { return robotAngDist(LEFT) < 35; });
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 10, 5000);
  // pros::delay(10);
  // Robot::chassis->waitUntil(3);

  // waitUntil([] { return !isMotionRunning() || robotAngDist(RIGHT) < 20;
  // }, 100,
  //           5000, false);
  // Robot::chassis->cancelMotion();
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 5, 5000);

  // Robot::chassis->cancelMotion();
  // Robot::chassis->follow(rush_6_elevation_triball_txt, 8, 5000);

  // waitUntil([] { return !isMotionRunning() || robotAngDist(LEFT) < 10; },
  // 100,
  //           5000, false);
  // printf("theta: %4.2f\n", Robot::chassis->getPose().theta);

  // Robot::chassis->moveToPoint(-10, MIN_Y + TILE_RADIUS + 1, 3000);
  // pros::delay(100);

  // waitUntilDistToPose({12, MIN_Y + TILE_RADIUS + 1}, 5, 0, true);
  // printf("slow down\n");
  // printf("pose: (%4.2f,%4.2f,%4.2f)\n", Robot::chassis->getPose().x,
  //        Robot::chassis->getPose().y, Robot::chassis->getPose().theta);

  // Robot::chassis->cancelMotion();
  // Robot::chassis->moveToPoint(6, MIN_Y + TILE_RADIUS + 2, 4000,
  //                             {.maxSpeed = 64, .minSpeed = 15});

  // Robot::Actions::intake();
  // waitUntil([] { return isTriballInIntake() || !isMotionRunning(); },
  // 100, 5000,
  //           true);
  // printf("triball in intake %i\n", isTriballInIntake());
  // printf("pose: (%4.2f,%4.2f,%4.2f)\n", Robot::chassis->getPose().x,
  //        Robot::chassis->getPose().y, Robot::chassis->getPose().theta);
  // Robot::chassis->cancelMotion();

  // tank(-127, -127, 250, 0);

  // tank(64, -127, 0, 0);
  // waitUntil([] { return robotAngDist(RIGHT) < 45; });

  // Robot::Actions::stopIntake();
  // printf("turn done\n");

  // //
  // //
  // //
  // //
  // //
  // //
  // //
  // //
  // // Path to smoothly remove triball in matchload zone and from the
  // matchload
  // // zone and plow the three balls into the goal
  // Robot::chassis->follow(rush_6_matchload_sweep_txt, 13, 2750);

  // // wait until the robot is near the matchload zone to expand wings
  // waitUntilDistToPose({MAX_X - TILE_LENGTH - 2, MIN_Y + TILE_LENGTH - 3 -
  // 0.5},
  //                     6, 0, true);
  // Robot::chassis->cancelMotion();
  // Robot::Actions::expandBothWings();
  // printf("expand wings\n");
  // pros::delay(210);
  // Robot::chassis->cancelMotion();

  // // retract wings soon after removing the triball from the matchload
  // zone Robot::chassis->turnTo(MAX_X, 0, 1000); pros::delay(200);
  // printf("retract wings\n");
  // pros::delay(200);

  // printf("back to follow\n");

  // Robot::chassis->follow(rush_6_matchload_sweep_txt, 13, 2750);

  // wait until the robot is near the goal to outtake

  // pros::delay(450);
  Robot::Actions::outtake();
  // printf("outtake\n");

  // Robot::chassis->moveToPose(0, -TILE_LENGTH, LEFT, 3000);
  // while ((std::abs(Robot::chassis->getPose().x - (MAX_X - TILE_RADIUS)) > 10
  // ||
  //         robotAngDist(0) > 20) &&
  //        Robot::chassis->isInMotion()) {
  //   pros::delay(10);
  // }
  // printf("cancel follow\n");
  // // fully ram into goal
  // Robot::chassis->cancelMotion();
  Robot::chassis->moveToPoint(MAX_X + 6, 0, 1500,

                              {.maxSpeed = 127, .minSpeed = 127});
  const int waitDur = (15000 - 1100) - (pros::millis() - startTime);
  printf("wait: %i\n", waitDur);
  pros::delay(waitDur);
  Robot::chassis->cancelMotion();
  tank(-127, -127, 200);
  tank(127, 127, 600);
  Robot::Actions::stopIntake();
  tank(-127, -64, 300);
  stop();
  // pros::delay(100);
  // tank(-127, -127, 300, 0);
  // Robot::chassis->moveToPoint(MAX_X, 0, 800,
  //                             {.maxSpeed = 127, .minSpeed = 127});
  // Robot::chassis->waitUntilDone();

  // tank(-127, 32, 0, 0);
  // printf("turn away goal\n");
  // waitUntil([] { return robotAngDist(LEFT) < 60; });

  // Robot::chassis->moveToPoint(TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5 + 6,
  // 3000,
  //                             {.minSpeed = 72});
  // waitUntilDistToPose({TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5 + 6}, 5);
  // Robot::chassis->cancelMotion();

  // Robot::chassis->moveToPose(TILE_RADIUS - 6, -TILE_LENGTH * 1.5 - 2,
  //                            LEFT - 15, 2000, {.minSpeed = 48});
  // Robot::chassis->waitUntilDone();

  // Robot::Actions::expandBothWings();
  // tank(64, -64, 0, 0);
  // waitUntil([] { return robotAngDist(LEFT + 15) < 10; }, 0, 1000);
  // tank(32, 0, 0, 0);
}

auton::Auton auton::autons::sixRush = {
    (char*)("6 ball rush / offensive / right"), runSixRush};