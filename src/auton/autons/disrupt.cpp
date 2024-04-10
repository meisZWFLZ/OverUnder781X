#include "auton.h"
#include "fieldDimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "robot.h"
#include <climits>

using namespace fieldDimensions;
using namespace auton::utils;

void runDisrupt() {
  // front of drivetrain should be aligned with closer edge of puzzle pattern
  // x is where neil decided so
  Robot::chassis->setPose(
      {0 - TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2 + 6,
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2, UP},
      false);

  // go to intermediate target to avoid hitting the short barrier
  const lemlib::Pose intermediateTarget {-TILE_LENGTH * 1.15,
                                         -TILE_LENGTH * 1.35, 30};
  Robot::chassis->moveToPose(intermediateTarget.x, intermediateTarget.y,
                             intermediateTarget.theta, 2000, {.minSpeed = 127});

  // wait until we are near the intermediate target
  waitUntilDistToPose(intermediateTarget, 7, 0, true);
  Robot::chassis->cancelMotion();

  // then go to the center triball by the barrier and intake it
  Robot::chassis->moveToPose(0 - Robot::Dimensions::drivetrainWidth / 2 - 2,
                             -Robot::Dimensions::drivetrainLength / 2 - 3, UP,
                             3000, {.minSpeed = 72});
  Robot::Actions::intake();
  // let intake speed up
  pros::delay(500);
  // wait until triball is intaked or we are past y = -4
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().y > -5;
      },
      50);
  Robot::chassis->cancelMotion();

  // if we have already intaked the triball, don't intake further
  if (!isTriballInIntake()) {
    Robot::chassis->swingToHeading(
        RIGHT, lemlib::DriveSide::RIGHT, 1000,
        {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 64});
    // let motion start
    pros::delay(50);
    // wait until triball is intaked
    waitUntil([] { return isTriballInIntake() || !isMotionRunning(); }, 50,
              INT_MAX, true);
    Robot::chassis->cancelMotion();
  }
  // delay for 250 ms to let triball get fully intaked
  tank(-64, -96, 250, 2);
  // don't stress intake
  Robot::Actions::stopIntake();

  // ensure we are going full power
  tank(-64, -96, 0, 0);
  // wait until past y = -12
  waitUntil([] { return Robot::chassis->getPose().y < -12; }, 0, 750);

  // quickly turn to face down
  Robot::chassis->turnToHeading(
      DOWN, 750,
      {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 127});
  // wait until somewhat facing down
  waitUntil([] { return robotAngDist(DOWN) < 20 || !isMotionRunning(); });
  Robot::Actions::outtake();
  Robot::chassis->cancelMotion();

  const lemlib::Pose centerTriballTarget {-TILE_LENGTH * 2, 0, 0};

  // turn towards second triball
  Robot::chassis->turnToPoint(
      centerTriballTarget.x, centerTriballTarget.y, 1000,
      {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 127});

  // wait until we are mostly facing towards second center triball
  waitUntil([centerTriballTarget] {
    return robotAngDist(Robot::chassis->getPose().angle(centerTriballTarget) *
                        180 / M_PI) < 20 ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // intake second triball
  Robot::chassis->moveToPoint(centerTriballTarget.x, centerTriballTarget.y,
                              2000, {.minSpeed = 64});
  Robot::Actions::intake();

  // let intake get up to speed
  pros::delay(300);

  // wait until triball has been intaked or we are past the neutral zone line
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().y <
                   -std::max(Robot::Dimensions::drivetrainWidth,
                             Robot::Dimensions::drivetrainLength) /
                       2;
      },
      50, INT_MAX, true);
  Robot::chassis->cancelMotion();

  // delay for 250 ms to let triball get fully intaked
  tank(-64, -96, 250, 2);
  // don't stress intake
  Robot::Actions::stopIntake();

  // ensure we are going full power
  tank(-64, -96, 250, 0);

  // // old disrupt
  // // position the robot to disrupt center triballs
  // Robot::chassis->moveToPose(-TILE_LENGTH - 3, -TILE_LENGTH + 6, RIGHT, 3000,
  //                            {.lead = .3, .minSpeed = 127});
  // // wait until facing right to push triballs
  // waitUntil([] { return robotAngDist(RIGHT) < 30 || !isMotionRunning(); });
  // Robot::chassis->cancelMotion();

  // // prepare to push triballs
  // Robot::Actions::expandBothWings();
  // Robot::Actions::outtake();

  // // full speed into the triballs
  // Robot::chassis->moveToPoint(10000000, 0, 3000, {.minSpeed = 127});

  // // wait until near barrier
  // waitUntil(
  //     [] { return Robot::chassis->getPose().x > -12.5 || !isMotionRunning();
  //     });
  // Robot::chassis->cancelMotion();

  // done disrupting
  // position the robot to clear matchload zone
  Robot::chassis->moveToPoint(-TILE_LENGTH * 2 - 1.5, -TILE_LENGTH * 2 - 6.5,
                              3000, {.forwards = false});

  // retract wings
  Robot::chassis->waitUntil(5);
  Robot::Actions::retractBothWings();

  Robot::chassis->waitUntilDone();

  // turn to be parallel to the matchload pipe
  Robot::chassis->turnToPoint(1000000, -1000000, 2000);
  Robot::chassis->waitUntilDone();

  // expand wing
  // Robot::Actions::expandBackWing();
  pros::delay(1000);

  // move in an arc to sweep ball out
  // tank(32, 127, 0, 0);
  // wait until facing right or if we are far from the matchload zone
  // waitUntil(
  //     [] {
  //       return robotAngDist(RIGHT) < 10 ||
  //              Robot::chassis->getPose().x > -TILE_LENGTH * 1.5;
  //     },
  //     0, 1000);
  // stop();

  // // ensure we don't bend back wing
  // Robot::Actions::retractBackWing();

  // intake any straggler balls
  Robot::Actions::intake();

  // // let matchload zone triball get ahead of us
  // pros::delay(1000);

  // touch horizontal elevation bar
  Robot::chassis->moveToPose(0 - Robot::Dimensions::drivetrainLength / 2 - 3,
                             MIN_Y + TILE_RADIUS, RIGHT, 2100);

  // if a triball enters the intake, outtake it
  while (pros::competition::is_autonomous() && isMotionRunning()) {
    if (isTriballInIntake()) {
      Robot::Actions::outtake();
      pros::delay(500);
    }
    pros::delay(10);
  }
  pros::delay(1000);
  Robot::Actions::outtake();
  tank(-64, -64, 0, 0);
  pros::delay(500);
  const float startingTheta = Robot::chassis->getPose().theta;

  // when the robot touches the bar it should begin to turn
  waitUntil([startingTheta] { return robotAngDist(startingTheta) > 5; });
  stop();
}

auton::Auton auton::autons::disrupt = {(char*)("disrupt / left"), runDisrupt};