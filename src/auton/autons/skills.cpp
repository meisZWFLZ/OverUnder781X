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
  // matchload(5, INT_MAX);
  matchload();
  Robot::Actions::retractRightWing();
  // turn to left and scoop balls away from short barrier
  Robot::chassis->turnToHeading(LEFT, /* lemlib::DriveSide::LEFT, */ 1000,
                                {.minSpeed = 32, .earlyExitRange = 5});
  Robot::chassis->waitUntilDone();

  // use long barrier to adjust odom
  tank(-96, -96, 500, 0);
  // wait until aligned with barrier
  waitUntil([] { return robotAngDist(LEFT) < 10; }, 50, 500, true);
  // if aligned with barrier then set odom
  if (robotAngDist(LEFT) < 10)
    Robot::chassis->setPose({-Robot::Dimensions::drivetrainLength / 2 - 2,
                             Robot::chassis->getPose().y,
                             Robot::chassis->getPose().theta},
                            false);

  Robot::Actions::stopIntake();
  Robot::Actions::retractLeftWing();

  // scoop balls near matchload bar to other side of field
  Robot::chassis->moveToPoint(
      -TILE_LENGTH * 2 + 4, TILE_LENGTH * 2 - Robot::Dimensions::trackWidth / 2,
      2500, {.minSpeed = 127, .earlyExitRange = 4});

  constexpr float halfRobotWidthWithWingsExpanded =
      Robot::Dimensions::drivetrainWidth / 2 +
      Robot::Dimensions::frontWingLength;
  constexpr float yForRunningAlongAlley =
      MAX_Y - Robot::Dimensions::drivetrainWidth / 2 + 4;

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
      MAX_X - Robot::Dimensions::drivetrainWidth / 2 + 4,
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
      Robot::chassis->getPose().x + 3, Robot::chassis->getPose().y + 12, 500,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});
  Robot::chassis->waitUntilDone();
  // dont get wing stuck in goal
  Robot::Actions::retractRightWing();
  // second ram balls into goal
  Robot::chassis->moveToPose(Robot::chassis->getPose().x + 3, 0,
                             ramIntoLeftSideOfGoalTarget.theta, 1350,
                             {.minSpeed = 127});
  pros::delay(700);
  // use wall and goal to adjust odom
  // wait until aligned with goal
  waitUntil([] { return robotAngDist(DOWN) < 5 || !isMotionRunning(); }, 50,
            INT_MAX, true);
  // if aligned with goal then set odom
  if (robotAngDist(DOWN) < 10)
    Robot::chassis->setPose(
        {MAX_X - Robot::Dimensions::drivetrainWidth / 2 - 1,
         TILE_LENGTH + Robot::Dimensions::drivetrainLength / 2,
         Robot::chassis->getPose().theta},
        false);
  Robot::chassis->cancelMotion();

  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x - 2, Robot::chassis->getPose().y + 8, 750,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});

  // turn to face away from goal
  Robot::chassis->swingToHeading(LEFT, lemlib::DriveSide::LEFT, 500,
                                 {.minSpeed = 127, .earlyExitRange = 20});
  Robot::chassis->waitUntilDone();
  // use right wing for scooping balls
  Robot::Actions::expandRightWing();

  // scoop balls in short barrier corner
  Robot::chassis->moveToPoint(halfRobotWidthWithWingsExpanded + 10,
                              TILE_LENGTH * 2 -
                                  halfRobotWidthWithWingsExpanded - 1,
                              1000, {.minSpeed = 127, .earlyExitRange = 6});

  const lemlib::Pose firstFrontGoalRam {TILE_LENGTH * 2, TILE_RADIUS,
                                        RIGHT + 25};

  // turn towards front face of goal
  Robot::chassis->swingToPoint(
      firstFrontGoalRam.x, firstFrontGoalRam.y, lemlib::DriveSide::LEFT, 750,
      {.direction = AngularDirection::CCW_COUNTERCLOCKWISE,
       .minSpeed = 56,
       .earlyExitRange = 20});

  // wait a bit to expand left wing
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 15; });
  Robot::Actions::expandBothWings();
  Robot::chassis->waitUntilDone();

  // odom goofing so lets fix it
  Robot::chassis->setPose(Robot::chassis->getPose() + lemlib::Pose {-4, -5});

  // first ram into front of goal (left)
  Robot::chassis->moveToPose(firstFrontGoalRam.x, firstFrontGoalRam.y,
                             firstFrontGoalRam.theta, 2000, {.minSpeed = 96});
  pros::delay(1500);

  // wait until aligned with goal
  waitUntil([] { return !isMotionRunning() || robotAngDist(RIGHT) < 5; }, 50,
            2000, true);
  // wait for another 300ms
  waitUntil([] { return isMotionRunning(); }, 0, 300);

  // if aligned with goal set pose
  if (robotAngDist(RIGHT) < 5)
    Robot::chassis->setPose(
        {TILE_LENGTH * 2 - Robot::Dimensions::drivetrainLength / 2,
         Robot::chassis->getPose().y, Robot::chassis->getPose().theta});
  // move on to backing out
  Robot::chassis->cancelMotion();

  // back out
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractBothWings();
  const float xForBackingOutOfFrontOfGoal =
      Robot::Dimensions::drivetrainWidth / 2 + 8;
  Robot::chassis->moveToPose(xForBackingOutOfFrontOfGoal, TILE_LENGTH * 1.2,
                             DOWN, 2000,
                             {
                                 .forwards = false,
                                 .chasePower = 6,
                                 .minSpeed = 72,
                             });

  // wait until facing down
  waitUntil([] { return !isMotionRunning() || robotAngDist(DOWN) < 20; });
  Robot::chassis->cancelMotion();

  // second ram into front of goal (center)
  Robot::Actions::expandBothWings();
  Robot::chassis->moveToPose(TILE_LENGTH * 2, 0, RIGHT, 2000, {.minSpeed = 88});
  // let bot ram into goal
  pros::delay(1500);

  // wait until aligned with goal
  waitUntil([] { return !isMotionRunning() || robotAngDist(RIGHT) < 5; }, 50,
            2000, true);
  // wait for another 300ms
  waitUntil([] { return isMotionRunning(); }, 0, 300);

  // if aligned with goal set pose
  if (robotAngDist(RIGHT) < 5)
    Robot::chassis->setPose(
        {TILE_LENGTH * 2 - Robot::Dimensions::drivetrainLength / 2,
         Robot::chassis->getPose().y, Robot::chassis->getPose().theta});
  // move on to backing out
  Robot::chassis->cancelMotion();

  // back out
  Robot::Actions::retractBothWings();
  Robot::chassis->moveToPose(xForBackingOutOfFrontOfGoal, -5, UP, 2000,
                             {
                                 .forwards = false,
                                 .chasePower = 6,
                                 .minSpeed = 64,
                             });
  // wait until facing up
  waitUntil([] { return !isMotionRunning() || robotAngDist(UP) < 20; });
  Robot::chassis->cancelMotion();

  // go backwards to prepare for third ram into front of goal(right)
  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x, Robot::chassis->getPose().y - 12, 1000,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});

  const lemlib::Pose thirdRamGoalTarget {TILE_LENGTH * 2, -7.5, RIGHT - 25};

  Robot::chassis->swingToPoint(thirdRamGoalTarget.x, thirdRamGoalTarget.y,
                               lemlib::DriveSide::LEFT, 500,
                               {.minSpeed = 127, .earlyExitRange = 30});
  Robot::chassis->waitUntilDone();

  // third ram into front of goal (right)
  Robot::Actions::expandBothWings();
  Robot::chassis->moveToPose(TILE_LENGTH * 2, -7.5, RIGHT - 25, 2000,
                             {.minSpeed = 88});
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractBothWings();

  // get outta there
  Robot::chassis->moveToPoint(
      Robot::Dimensions::trackWidth / 2 + Robot::Dimensions::frontWingLength +
          1,
      Robot::chassis->getPose().y, 1500,
      {.forwards = false, .minSpeed = 80, .earlyExitRange = 3});

  // then turn
  Robot::chassis->swingToHeading(DOWN, lemlib::DriveSide::LEFT, 2500,
                                 {.minSpeed = 64, .earlyExitRange = 20});
  Robot::chassis->waitUntilDone();

  // scoop balls out of barrier corner
  Robot::Actions::expandBothWings();
  Robot::chassis->moveToPoint(Robot::Dimensions::trackWidth / 2 +
                                  Robot::Dimensions::frontWingLength + 1,
                              Robot::Dimensions::trackWidth / 2 +
                                  Robot::Dimensions::frontWingLength + 1 -
                                  TILE_LENGTH * 2,
                              1000, {.minSpeed = 48, .earlyExitRange = 3});

  // slowly scoop so as to not throw balls over k
  Robot::chassis->swingToHeading(RIGHT, lemlib::DriveSide::LEFT, 750,
                                 {.minSpeed = 64, .earlyExitRange = 25});

  const lemlib::Pose goPastShortBarrierTarget {
      TILE_LENGTH + Robot::Dimensions::drivetrainWidth / 2 + 2,
      -TILE_LENGTH * 2, DOWN};

  // fling the balls hopefully into the right side of the goal and prepare to go
  // past the short barrier
  Robot::chassis->swingToPoint(
      goPastShortBarrierTarget.x, goPastShortBarrierTarget.y,
      lemlib::DriveSide::LEFT, 750,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 25});
  Robot::chassis->waitUntilDone();

  // don't break wings
  Robot::Actions::retractBothWings();
  // go past short barrier
  Robot::chassis->moveToPose(goPastShortBarrierTarget.x,
                             goPastShortBarrierTarget.y,
                             goPastShortBarrierTarget.theta, 1000,
                             {
                                 .forwards = false,
                                 .minSpeed = 92,
                             });

  // wait until past short barrier
  waitUntil([] {
    return Robot::chassis->getPose().y < -TILE_LENGTH * 2 || !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // become parallel with the wall and face towards the right side of the goal
  Robot::chassis->moveToPose(TILE_LENGTH,
                             MIN_Y + Robot::Dimensions::drivetrainWidth / 2 + 3,
                             RIGHT, 1000, {.forwards = false, .minSpeed = 64});
  // once we are parallel with the wall, ram into the goal
  waitUntil([] { return robotAngDist(RIGHT) < 15 || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  // use left wing to scoop balls
  Robot::Actions::expandLeftWing();

  const lemlib::Pose ramIntoRightSideOfGoalTarget {
      MAX_X - Robot::Dimensions::drivetrainWidth / 2, -TILE_LENGTH * 2, UP};
  // ram balls into right side of goal
  Robot::chassis->moveToPose(
      ramIntoRightSideOfGoalTarget.x, ramIntoRightSideOfGoalTarget.y,
      ramIntoRightSideOfGoalTarget.theta, 1200, {.minSpeed = 72});

  // go full speed once facing goal
  waitUntil([&ramIntoRightSideOfGoalTarget] {
    return !isMotionRunning() ||
           robotAngDist(ramIntoRightSideOfGoalTarget.theta) < 15;
  });
  Robot::chassis->cancelMotion();
  // dont spin balls up above goal
  Robot::Actions::stopIntake();
  Robot::chassis->moveToPose(ramIntoRightSideOfGoalTarget.x, 0,
                             ramIntoRightSideOfGoalTarget.theta, 800,
                             {.minSpeed = 127});

  // back up for second ram
  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x + 3, Robot::chassis->getPose().y - 12, 500,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});
  Robot::chassis->waitUntilDone();
  // dont get wing stuck in goal
  Robot::Actions::retractLeftWing();
  // second ram balls into goal
  Robot::chassis->moveToPose(Robot::chassis->getPose().x + 3, 0,
                             ramIntoRightSideOfGoalTarget.theta, 1350,
                             {.minSpeed = 127});
  pros::delay(700);
  // use wall and goal to adjust odom
  // wait until aligned with goal
  waitUntil([] { return robotAngDist(UP) < 5 || !isMotionRunning(); }, 50,
            INT_MAX, true);
  // if aligned with goal then set odom
  if (robotAngDist(UP) < 10)
    Robot::chassis->setPose(
        {MAX_X - Robot::Dimensions::drivetrainWidth / 2,
         -TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
         Robot::chassis->getPose().theta},
        false);
  Robot::chassis->cancelMotion();

  // back up
  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x - 3, Robot::chassis->getPose().y - 12, 750,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 3});

  const lemlib::Pose goPastShortBarrierForHangTarget {
      TILE_LENGTH, MIN_Y - TILE_RADIUS, LEFT};

  // face towards elevation bar
  Robot::chassis->swingToPoint(
      goPastShortBarrierForHangTarget.x, goPastShortBarrierForHangTarget.y,
      lemlib::DriveSide::RIGHT, 500, {.minSpeed = 127, .earlyExitRange = 30});
  Robot::chassis->waitUntilDone();
  // expand hang early in order to permit it enough time to expand
  Robot::Subsystems::lift->extend();
  // go to intermediate target in order to prevent hitting short barrier
  Robot::chassis->moveToPose(
      goPastShortBarrierForHangTarget.x, goPastShortBarrierForHangTarget.y,
      goPastShortBarrierForHangTarget.theta, 2000, {.minSpeed = 72});

  // wait until past short barrier
  waitUntil([&goPastShortBarrierForHangTarget] {
    return Robot::chassis->getPose().x < goPastShortBarrierForHangTarget.x ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // go full speed into elevation bar
  Robot::chassis->moveToPoint(-1000000, Robot::chassis->getPose().y, 1500,
                              {.minSpeed = 127});
  Robot::chassis->waitUntilDone();
  // then move robot up
  Robot::Subsystems::lift->retract();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
