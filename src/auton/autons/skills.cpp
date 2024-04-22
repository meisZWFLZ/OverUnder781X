#include "auton.h"
#include "fieldDimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "robot.h"

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
  lemlib::Timer timer {60000};
  matchload(5, INT_MAX);
  // matchload();
  Robot::Actions::retractRightWing();
  // turn to left and scoop balls away from short barrier
  Robot::chassis->turnToHeading(LEFT, /* lemlib::DriveSide::LEFT, */ 1000,
                                {.minSpeed = 32, .earlyExitRange = 5});
  Robot::chassis->waitUntilDone();

  // use long barrier to adjust odom
  tank(-96, -96, 700, 0);
  Robot::chassis->setPose({-Robot::Dimensions::drivetrainLength / 2 - 2,
                           Robot::chassis->getPose().y,
                           Robot::chassis->getPose().theta},
                          false);

  Robot::Actions::stopIntake();
  Robot::Actions::retractLeftWing();

  // scoop balls near matchload bar to other side of field
  Robot::chassis->moveToPoint(-TILE_LENGTH * 2 + 4,
                              TILE_LENGTH * 2 -
                                  Robot::Dimensions::trackWidth / 2,
                              2500, {.minSpeed = 127, .earlyExitRange = 4});

  constexpr float halfRobotWidthWithWingsExpanded =
      Robot::Dimensions::drivetrainWidth / 2 +
      Robot::Dimensions::frontWingLength;
  constexpr float yForRunningAlongAlley =
      MAX_Y - Robot::Dimensions::drivetrainWidth / 2 - 4;

  // swing around to face alley
  Robot::chassis->swingToPoint(-TILE_LENGTH, yForRunningAlongAlley,
                               lemlib::DriveSide::RIGHT, 750,
                               {.direction = AngularDirection::CW_CLOCKWISE,
                                .minSpeed = 127,
                                .earlyExitRange = 15});
  // wait until bot is facing right to expand right wing
  waitUntil([] { return robotAngDist(UP - 45) < 15 || !isMotionRunning(); });
  Robot::Actions::expandRightWing();
  Robot::chassis->waitUntilDone();

  Robot::Actions::intake();
  /** whether we have intaked a ball*/
  bool hasIntakedBall = false;

  Robot::chassis->moveToPose(-TILE_LENGTH + 6,
                             MAX_Y - Robot::Dimensions::drivetrainWidth / 2,
                             RIGHT, 2000, {.chasePower = 8, .minSpeed = 96});
  pros::delay(500);
  waitUntil([&hasIntakedBall] {
    return (!hasIntakedBall && isTriballInIntake()) ||
           robotAngDist(RIGHT) < 15 || !isMotionRunning();
  });

  if (isTriballInIntake()) {
    bool hasIntakedBall = true;
    Robot::Actions::stopIntake();
    waitUntil([] { return robotAngDist(RIGHT) < 15 || !isMotionRunning(); });
  }
  Robot::chassis->cancelMotion();
  Robot::Actions::outtake();
  Robot::chassis->moveToPoint(TILE_LENGTH, yForRunningAlongAlley, 750,
                              {.minSpeed = 127});

  const lemlib::Pose ramIntoLeftSideOfGoalTarget {
      MAX_X - Robot::Dimensions::drivetrainWidth / 2 + 2,
      TILE_LENGTH + Robot::Dimensions::drivetrainLength / 2, DOWN};
  // ram balls into left side of goal
  Robot::chassis->moveToPose(
      ramIntoLeftSideOfGoalTarget.x, ramIntoLeftSideOfGoalTarget.y,
      ramIntoLeftSideOfGoalTarget.theta, 1000, {.minSpeed = 72});

  // go full speed once facing goal
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 15; });
  Robot::chassis->cancelMotion();
  // dont spin balls up above goal
  Robot::Actions::stopIntake();
  Robot::chassis->moveToPose(ramIntoLeftSideOfGoalTarget.x, 0,
                             ramIntoLeftSideOfGoalTarget.theta, 800,
                             {.minSpeed = 127});

  // back up for second ram
  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x + 2, TILE_LENGTH * 1.8, 500,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});
  // second ram balls into goal
  Robot::chassis->moveToPose(Robot::chassis->getPose().x + 4, 0,
                             ramIntoLeftSideOfGoalTarget.theta, 700,
                             {.minSpeed = 127});
  Robot::chassis->waitUntilDone();

  // use wall and goal to adjust odom
  Robot::chassis->setPose(
      {ramIntoLeftSideOfGoalTarget.x, ramIntoLeftSideOfGoalTarget.y, DOWN},
      false);
  tank(-127, -127, 300);

  // turn to face away from goal
  Robot::chassis->swingToHeading(LEFT, lemlib::DriveSide::LEFT, 500,
                                 {.minSpeed = 127, .earlyExitRange = 20});

  // scoop balls in short barrier corner
  Robot::chassis->moveToPoint(halfRobotWidthWithWingsExpanded + 5,
                              TILE_LENGTH * 2 - halfRobotWidthWithWingsExpanded,
                              1000, {.minSpeed = 127, .earlyExitRange = 6});
  // turn towards front face of goal
  Robot::chassis->swingToHeading(
      RIGHT, lemlib::DriveSide::LEFT, 750,
      {.direction = AngularDirection::CCW_COUNTERCLOCKWISE,
       .minSpeed = 64,
       .earlyExitRange = 20});
  // wait a bit to expand left wing
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 15; });
  Robot::Actions::expandLeftWing();
  Robot::chassis->waitUntilDone();

  // first ram into front of goal (left)
  Robot::chassis->moveToPose(TILE_LENGTH * 2, TILE_RADIUS, RIGHT + 25, 1500,
                             {.minSpeed = 127});

  // back out
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractBothWings();
  Robot::chassis->moveToPose(+Robot::Dimensions::drivetrainWidth / 2 + 2,
                             TILE_LENGTH, DOWN, 500,
                             {.forwards = false, .lead = 0.4, .minSpeed = 127});
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 20; });
  Robot::chassis->cancelMotion();
  // second ram into front of goal (center)
  Robot::Actions::expandBothWings();
  Robot::chassis->moveToPose(TILE_LENGTH * 2, 0, RIGHT, 1500,
                             {.minSpeed = 127});
  Robot::chassis->waitUntilDone();

  // back out
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractBothWings();
  Robot::chassis->moveToPose(+Robot::Dimensions::drivetrainWidth / 2 + 2,
                             TILE_RADIUS, DOWN, 500,
                             {.forwards = false, .lead = 0.4, .minSpeed = 127});
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 20; });
  Robot::chassis->cancelMotion();
  // third ram into front of goal (right)
  Robot::Actions::expandBothWings();
  Robot::chassis->moveToPose(TILE_LENGTH * 2, -TILE_RADIUS, RIGHT - 25, 1500,
                             {.minSpeed = 127});
  Robot::chassis->waitUntilDone();

  tank(127, 127, 250, 0);
  Robot::chassis->moveToPose(TILE_LENGTH * 1.5, -TILE_LENGTH * 2, LEFT, 1500,
                             {.minSpeed = 72});
  waitUntil([] {
    return Robot::chassis->getPose().y < -TILE_LENGTH * 2 || !isMotionRunning();
  });
  Robot::chassis->cancelMotion();
  // let the bot chill
  stop();
  pros::delay(900);

  Robot::chassis->moveToPose(6, MIN_Y, LEFT, 2000, {.minSpeed = 64});
  // then move robot up
  Robot::Subsystems::lift->retract();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
