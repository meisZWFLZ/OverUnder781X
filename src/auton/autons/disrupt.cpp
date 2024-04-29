#include "auton.h"
#include "fieldDimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "wings.h"
#include <climits>

using namespace fieldDimensions;
using namespace auton::utils;

void runDisrupt() {
  const int startTime = pros::millis();
  // front of drivetrain should be aligned with closer edge of puzzle pattern
  // x is where neil decided so
  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2 + 6,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2 - 2, UP},
      false);

  const lemlib::Pose intakeCenterTriballTarget = {-TILE_LENGTH - 3, -5.25, UP};

  // intake center triball
  Robot::chassis->moveToPose(intakeCenterTriballTarget.x,
                             intakeCenterTriballTarget.y,
                             intakeCenterTriballTarget.theta, 1500,
                             {.chasePower = 5, .lead = 0.5, .minSpeed = 127});
  Robot::Actions::intake();
  // let intake speed up
  pros::delay(500);
  // wait until triball is intaked or we are past y = -18
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().y > -24;
      },
      50);
  // then slow down
  Robot::chassis->cancelMotion();
  Robot::chassis->moveToPose(
      intakeCenterTriballTarget.x, intakeCenterTriballTarget.y,
      intakeCenterTriballTarget.theta, 1000,
      {.maxSpeed = 56}); // minSpeed is not set in order to slow down
  // wait until triball is intaked or drive train is past neutral zone
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().y >
                   -Robot::Dimensions::drivetrainLength / 2 - 8;
      },
      50);
  Robot::chassis->cancelMotion();
  // let intake fully grab ahold that triball and cancel forwards motion
  tank(-32, -48, 300, 0);

  // go backwards away from neutral zone
  Robot::chassis->moveToPoint(Robot::chassis->getPose().x + 1,
                              Robot::chassis->getPose().y - 7.5, 1000,
                              {
                                  .forwards = false,
                                  .minSpeed = 48,
                              });
  pros::delay(300);
  // don't overstress intake
  Robot::Actions::stopIntake();
  Robot::chassis->waitUntilDone();

  // wait until facing RIGHT, then expand left front wing
  Robot::Subsystems::wings->front->setIthState(int(WING_PAIR_INDEX::LEFT),
                                               true);
  // swing to face right
  Robot::chassis->swingToHeading(
      /* DOWN */ RIGHT, lemlib::DriveSide::RIGHT, 1000,
      {
          .direction = AngularDirection::CW_CLOCKWISE,
          .maxSpeed = 127,
          .minSpeed = 127,
          .earlyExitRange = 20,
      });

  // turn the rest of the way to face down
  Robot::chassis->turnToHeading(DOWN + 30, 1000,
                                {.direction = AngularDirection::CW_CLOCKWISE,
                                 .minSpeed = 127,
                                 .earlyExitRange = 20});
  Robot::chassis->waitUntilDone();

  // done disrupting
  // position the robot to clear matchload zone
  Robot::chassis->moveToPoint(-TILE_LENGTH * 2 - 2, -TILE_LENGTH * 2 - 11,
                              6000, {.maxSpeed = 64});

  // retract wings
  Robot::chassis->waitUntil(5);
  Robot::Actions::retractBothWings();

  Robot::chassis->waitUntilDone();

  // turn to be parallel to the matchload pipe
  Robot::chassis->turnToHeading(DOWN - 45, 750);
  Robot::chassis->waitUntilDone();

  // expand wing
  Robot::Actions::expandBackWing();
  pros::delay((startTime + 15000) - pros::millis() - 5000);

  // move in an arc to sweep ball out
  tank(0, 127, 0, 0);
  // wait until facing right or if we are far from the matchload zone
  waitUntil(
      [] {
        return robotAngDist(RIGHT - 15) < 10 ||
               Robot::chassis->getPose().x > -TILE_LENGTH * 1.5;
      },
      0, 1000);

  Robot::chassis->turnToHeading(UP + 45, 500);
  Robot::chassis->turnToHeading(RIGHT, 500);

  // ensure we don't bend back wing
  Robot::Actions::retractBackWing();

  // get rid of intaked triball
  Robot::Actions::outtake();

  // let matchload zone triball get ahead of us
  pros::delay(500);
  // intake any straggler balls
  Robot::Actions::intake();

  // touch horizontal elevation bar
  Robot::chassis->moveToPose(0 - Robot::Dimensions::drivetrainLength / 2 - 5.5,
                             MIN_Y + Robot::Dimensions::drivetrainWidth / 2 + 2,
                             RIGHT, 2000);
  // waitUntil([] {
  //   return (Robot::chassis->getPose().y < MIN_Y - TILE_RADIUS &&
  //           Robot::chassis->getPose().x > -TILE_LENGTH) ||
  //          !isMotionRunning();
  // });

  // if a triball enters the intake, outtake it
  Robot::chassis->moveToPoint(-TILE_LENGTH * 2 - 4, -TILE_LENGTH * 2.5 - 3,
                              2000, {.forwards = false, .minSpeed = 127});
  Robot::Actions::outtake();
  Robot::chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

auton::Auton auton::autons::disrupt = {(char*)("disrupt / left"), runDisrupt};