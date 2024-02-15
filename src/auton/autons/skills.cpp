#include "auton.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

// ASSET(skills_right_side_txt);
// ASSET(skills_front_1_txt);
// ASSET(skills_front_2_txt);

using namespace fieldDimensions;
using namespace auton::utils;
using namespace auton::actions;

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
  Robot::chassis->setPose(
      {-TILE_LENGTH * 2 + Robot::Dimensions::drivetrainLength / 2,
       -TILE_LENGTH * 2 - Robot::Dimensions::drivetrainWidth / 2, RIGHT},
      false);
  // @todo add skills auton

  prepareForMatchloading();
//   tank(0, -10, 0, 0);
  Robot::Subsystems::catapult->matchload(40000, 46);
  Robot::Subsystems::catapult->waitUntilDoneMatchloading();
  Robot::Motors::leftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  Robot::Motors::rightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  stop();

  Robot::chassis->moveToPoint(-TILE_LENGTH, MIN_Y + 6, 5000, {.minSpeed = 127});
  waitUntilDistToPose({-TILE_LENGTH, MIN_Y + TILE_RADIUS}, 12, 0, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(0, MIN_Y + 6, 5000, {.minSpeed = 127});
  waitUntilDistToPose({0, MIN_Y + TILE_RADIUS}, 12, 0, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(MAX_X - TILE_LENGTH, MIN_Y + TILE_RADIUS + 6,
                              5000, {.minSpeed = 127});
  waitUntilDistToPose({MAX_X - TILE_LENGTH, MIN_Y + TILE_RADIUS}, 12, 0, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(MAX_X - TILE_RADIUS, 0, 5000, {.minSpeed = 127});
  waitUntilDistToPose({MAX_X - TILE_RADIUS, -TILE_LENGTH * 1.5}, 12, 0, true);
  Robot::chassis->cancelMotion();

  tank(127, 127, 200, 0);
  tank(-127, -127, 200, 0);
  tank(127, 127, 400, 0);
  tank(-127, -127, 250, 0);

  Robot::chassis->moveToPoint(TILE_LENGTH, -TILE_LENGTH * 1.5, 5000,
                              {.minSpeed = 127});
  waitUntilDistToPose({TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5}, 12, 0, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPose(TILE_LENGTH, -9, RIGHT, 5000,
                             {.minSpeed = 127, .earlyExitRange = 8});
  waitUntil(
      [] { return robotAngDist(RIGHT) < 60 || !Robot::chassis->isInMotion(); });
  Robot::Actions::expandBothWings();

  waitUntilDistToPose({TILE_LENGTH, -9, RIGHT}, 4, 200, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(10000000, 0, 5000, {.minSpeed = 127});
  waitUntil(
      [] {
        return fabs(Robot::chassis->getPose().x - TILE_LENGTH * 2) < 4 ||
               !Robot::chassis->isInMotion();
      },
      300);
  Robot::chassis->cancelMotion();

  Robot::Actions::retractBothWings();
  Robot::chassis->moveToPose(
      TILE_LENGTH, 9, RIGHT, 5000,
      {.forwards = true, .minSpeed = 127, .earlyExitRange = 8});

  waitUntilDistToPose({TILE_LENGTH, 9, RIGHT}, 4, 200, true);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(10000000, 0, 5000, {.minSpeed = 127});
  waitUntil(
      [] {
        return fabs(Robot::chassis->getPose().x - TILE_LENGTH * 2) < 4 ||
               !Robot::chassis->isInMotion();
      },
      300);
  tank(-127, -127, 250, 0);
  Robot::Actions::retractBothWings();

  Robot::chassis->moveToPose(TILE_LENGTH * 1.5, TILE_LENGTH * 2, DOWN, 5000,
                             {
                                 .forwards = false,
                                 .minSpeed = 127,
                                 .earlyExitRange = 8,
                             });

  Robot::chassis->moveToPoint(TILE_LENGTH * 2.5, 0, 5000,
                              {
                                  .minSpeed = 127,
                                  .earlyExitRange = 8,
                              });
  waitUntilDistToPose({-TILE_LENGTH * 1.5, -TILE_LENGTH * 1.5}, 12, 0, true);
  Robot::chassis->cancelMotion();

  stop();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
