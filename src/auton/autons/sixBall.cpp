#include "auton.h"
#include "lemlib/chassis/chassis.hpp"
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
  // with alliance triball in intake facing right
  Robot::chassis->setPose(
      {MAX_X - TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainWidth / 2, RIGHT},
      false);

  // drop intake
  tank(48, 48, 150, 0);
  stop();
  // let vibrations dissipate
  pros::delay(1000);

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
  tank(-40, -40, 500, 0);
  Robot::Actions::stopIntake();
  lemlib::Pose elevationBarTarget {Robot::Dimensions::drivetrainLength / 2 + 2,
                                   MIN_Y + TILE_RADIUS - 0.5, LEFT};
  // go under elevation bar to touch triball
  Robot::chassis->moveToPose(elevationBarTarget.x, elevationBarTarget.y,
                             elevationBarTarget.theta, 4000, {.maxSpeed = 64});

  // intake ball under elevation bar
  Robot::Actions::intake();
  // let intake get up to speed
  pros::delay(500);

  // wait until triball sensed in intake
  waitUntil([] { return isTriballInIntake(); }, 50);
  // let triball get fully intaked
  pros::delay(300);

  // stop intaking
  Robot::Actions::stopIntake();
  Robot::chassis->cancelMotion();

  // give space to back up while outtaking
  tank(-48, -48, 1000, 0);

  // turn away from elevation bar
  Robot::chassis->turnToHeading(
      RIGHT, 2000,
      {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 64});
  Robot::chassis->waitUntilDone();

  // outtake the triball for 1000 ms
  Robot::Actions::outtake();
  // go back to help outtaking the triball
  tank(-40, -40, 500, 0);
  stop();
  pros::delay(500);

  // stop outtaking
  Robot::Actions::stopIntake();

  // turn back towards elevation bar
  Robot::chassis->turnToHeading(
      LEFT, 2000,
      {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 64});
  Robot::chassis->waitUntilDone();

  // intake triballs from other bot
  Robot::Actions::intake();
  // touch horizontal elevation bar
  Robot::chassis->moveToPose(elevationBarTarget.x, elevationBarTarget.y,
                             elevationBarTarget.theta, 4000, {.maxSpeed = 64});

  // let intake get up to speed
  pros::delay(500);

  // if triball intaked
  waitUntil([] { return isTriballInIntake(); }, 50);
  // let triball get fully intaked
  pros::delay(300);
  // then dont constantly spin intake
  Robot::Actions::stopIntake();
}

auton::Auton auton::autons::sixBall = {(char*)("6 ball / offensive / right"),
                                       run6Ball};