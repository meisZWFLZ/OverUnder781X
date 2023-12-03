#include "auton.h"
#include "lemlib/asset.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

ASSET(def_score_alliance_txt);

using namespace fieldDimensions;

void runDefensive() {
  using namespace fieldDimensions;
  using namespace auton::actions;
  Robot::chassis->setPose(leftStartingPose, false);
  const lemlib::Pose scoreAllyTarget = {-36, -15};
  const lemlib::Pose intakeMiddleTarget = {
      TILE_LENGTH, Robot::Dimensions::drivetrainLength / 2 - 5, UP};
  const lemlib::Pose outtakeMiddleTarget = {
      0 - Robot::Dimensions::drivetrainLength / 2, -TILE_LENGTH / 2.0, RIGHT};

  // Curve around and score into large part of goal
  Robot::chassis->follow(def_score_alliance_txt, 3250, 10, true);

  // wait until 5 inches away from, then outtake
  while (Robot::chassis->getPose().distance(scoreAllyTarget) > 5)
    pros::delay(10);
  Robot::Actions::outtake();
  printf("time: %ims\n", pros::millis());

  // wait until done with pure pursuit motion
  Robot::chassis->waitUntilDist(-1);

  // push triball into goal
  Robot::chassis->tank(127, 127);
  pros::delay(500);
  // get out of goal
  Robot::chassis->tank(-127, -127);
  pros::delay(500);
  Robot::Actions::stopIntake();

  // UNTESTED:
  // go to middle triball
  Robot::chassis->moveTo(intakeMiddleTarget.x, intakeMiddleTarget.y,
                         intakeMiddleTarget.theta, 2000, true);
  // wait until within 5 inches to start intaking
  while (Robot::chassis->getPose().distance(intakeMiddleTarget) > 5)
    pros::delay(10);
  Robot::Actions::intake();
  printf("time: %ims\n", pros::millis());

  // wait until done moving to middle triball
  Robot::chassis->waitUntilDist(-1);
  pros::delay(500); // give some time to intake triball

  // get outta there
  Robot::chassis->tank(-64, -64);
  pros::delay(250);
  Robot::Actions::stopIntake();

  // outtake triball over barrier
  Robot::chassis->moveTo(outtakeMiddleTarget.x, outtakeMiddleTarget.y,
                         outtakeMiddleTarget.theta, 2000, true);
  // wait until 5 inches away from outtake spot
  while (Robot::chassis->getPose().distance(outtakeMiddleTarget) > 5)
    pros::delay(10);
  Robot::Actions::outtake();
  // wait until done with move to motion
  Robot::chassis->waitUntilDist(-1);

  // make sure it gets outtook
  Robot::chassis->tank(127, 127);
  pros::delay(500);
  Robot::Actions::stopIntake();

  // get outta there
  Robot::chassis->tank(-127, -127);
  pros::delay(250);

  // @todo remove match load zone triball

  // touch elevation bar
  Robot::chassis->moveTo(-TILE_RADIUS, TILE_LENGTH * 2 - TILE_RADIUS,
                         RIGHT - 45, 2500);
  // zip ties should touch elevation bar
  Robot::Actions::lowerIntake();

  // make sure we're touching elevation bar
  Robot::chassis->tank(127, 127);
  pros::delay(500);
  // stop
  Robot::chassis->tank(0, 0);
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"),
                                         runDefensive};