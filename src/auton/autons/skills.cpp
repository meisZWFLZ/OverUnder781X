#include "auton.h"
#include "fieldDimensions.h"
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

  // set slew to 5 for skills
  Robot::chassis->lateralSettings.slew = 5;

  constexpr float halfRobotWidthWithWingsExpanded =
      Robot::Dimensions::drivetrainWidth / 2 +
      Robot::Dimensions::frontWingLength;
  constexpr float xForRunningAlongLongBarrier =
      -halfRobotWidthWithWingsExpanded;

  // don't hit close short barrier
  Robot::chassis->moveToPose(
      -TILE_LENGTH, MIN_Y + TILE_LENGTH - 2 + halfRobotWidthWithWingsExpanded,
      RIGHT, 1750, {.minSpeed = 96});
  // wait until past short barrier
  waitUntil([] {
    return !isMotionRunning() || Robot::chassis->getPose().x > -TILE_LENGTH;
  });

  // then smack wings into balls
  Robot::Subsystems::wings->front->enable();
  Robot::Actions::outtake();

  // arc to face parallel to long barrier and push balls beside it
  Robot::chassis->moveToPose(xForRunningAlongLongBarrier, -TILE_RADIUS + 6, UP,
                             1500, {.minSpeed = 72});

  // wait until bot is almost mid way
  waitUntil([] {
    return Robot::chassis->getPose().y > -TILE_RADIUS || !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // push balls towards short barrier
  Robot::chassis->moveToPose(xForRunningAlongLongBarrier, 2 * TILE_LENGTH + 3,
                             UP, 2000, {.minSpeed = 96});
  waitUntil([] {
    return !isMotionRunning() ||
           Robot::chassis->getPose().y >
               TILE_LENGTH + Robot::Dimensions::drivetrainLength / 2;
  });
  Robot::chassis->cancelMotion();
  return;

  // // turn to left and scoop balls away from short barrier
  // Robot::chassis->turnToHeading(LEFT, /* lemlib::DriveSide::LEFT, */ 1000,
  //                               {.minSpeed = 127, .earlyExitRange = 20});
  Robot::chassis->waitUntilDone();

  Robot::Actions::stopIntake();

  Robot::Actions::retractLeftWing();

  Robot::chassis->moveToPose(-TILE_LENGTH * 2 - 4, TILE_LENGTH * 2, UP, 3000,
                             {.chasePower = 8, .minSpeed = 96});
  Robot::Actions::intake();
  pros::delay(500);

  // whether we have intaked a ball
  bool hasIntakedBall = false;
  waitUntil([] {
    return isTriballInIntake() || robotAngDist(UP) < 15 || !isMotionRunning();
  });
  if (isTriballInIntake()) {
    bool hasIntakedBall = true;
    Robot::Actions::stopIntake();
    waitUntil([] { return robotAngDist(UP) < 15 || !isMotionRunning(); });
  }

  Robot::chassis->cancelMotion();
  Robot::chassis->moveToPose(-TILE_LENGTH + 6, MAX_Y - TILE_RADIUS, RIGHT, 3500,
                             {.chasePower = 8, .minSpeed = 96});
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

  // Robot::chassis->moveToPoint(TILE_LENGTH * 2, MAX_Y - TILE_RADIUS, 2000,
  //                             {.minSpeed = 127});
  // Robot::chassis->waitUntilDone()
  Robot::Actions::stopIntake();
  // let the bot chill
  stop();
  pros::delay(900);

  // then move robot up
  Robot::Subsystems::lift->retract();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
