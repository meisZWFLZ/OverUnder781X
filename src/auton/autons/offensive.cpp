#include "auton.h"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

ASSET(ball6_matchload_sweep_txt);

void tank(float left, float right, int ms) {
  Robot::chassis->tank(left, right);
  pros::delay(ms);
}

void waitUntilDistTarget(lemlib::Pose target, float error = 3, int time = 300) {
  printf("start wait\n");
  const int start = pros::millis();
  int inRangeStartTime = 0;
  pros::delay(100);
  do {
    if (inRangeStartTime == 0) {
      if (Robot::chassis->getPose().distance(target) < error)
        inRangeStartTime = pros::millis();
    } else if (pros::millis() - inRangeStartTime > time) {
      printf("breaking\n");
      break;
    }
    pros::delay(10);
  } while (true);
  printf("time in wait: %i\n", pros::millis() - start);
}

void runOffensive() {
  Robot::chassis->setPose(
      {0 + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       MIN_Y + TILE_RADIUS, LEFT},
      false);

  // intake triball
  Robot::Actions::intake();
  tank(36, 36, 75);
  tank(72, 72, 200);
  tank(36, 36, 75);
  tank(0, 0, 50);

  // get away from scary zone
  tank(-36, -36, 75);
  tank(-72, -72, 200);
  tank(-36, -36, 75);
  tank(0, 0, 50);
  Robot::Motors::intake.move_voltage(40);

  // // start turn
  // tank(127, -127, 100);
  // turn towards the path
  Robot::chassis->turnTo(10000000, 0, 400);

  // while (lemlib::angleError(Robot::chassis->getPose().theta, RIGHT, false) >
  // 10)
  //   pros::delay(10);
  // Robot::chassis->cancelMotion();

  Robot::chassis->waitUntilDone();
  printf("cancel turn\n");
  Robot::chassis->follow(ball6_matchload_sweep_txt, 13, 5000);

  waitUntilDistTarget({MAX_X - TILE_LENGTH, MIN_Y + TILE_LENGTH - 3}, 6,
                      250);
  Robot::Actions::expandWings();
  printf("expand wings\n");

  pros::delay(500);
  Robot::Actions::retractWings();
  printf("retract wings\n");

  Robot::chassis->waitUntil(60);
  Robot::Actions::outtake();
  printf("outtake\n");

  waitUntilDistTarget({MAX_X - TILE_RADIUS, -TILE_LENGTH}, 8, 750);
  Robot::chassis->cancelMotion();
  printf("stop follow\n");

  tank(-127, -127, 100);
  Robot::Actions::stopIntake();
  tank(0, 0, 0);
  printf("stop\n");
}

auton::Auton auton::autons::offensive = {(char*)("offensive / right"),
                                         runOffensive};