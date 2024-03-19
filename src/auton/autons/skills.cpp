#include "auton.h"
#include "fieldDimensions.h"
#include "pros/rtos.hpp"
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
  // matchload(2, INT_MAX);
  matchload();

  // set slew to 5 for skills
  Robot::chassis->lateralSettings.slew = 5;

  // intake matchload for jumping over bar
  Robot::Actions::intake();

  // go down elevation alley
  // make sure we don't hit short barrier
  Robot::chassis->moveToPoint(-TILE_LENGTH * .75, -TILE_LENGTH * 2.65, 5000,
                              {.minSpeed = 127});
  pros::delay(500);
  // wait until the robot's y position is past the short barrier
  waitUntil([] {
    return Robot::chassis->getPose().y < -TILE_LENGTH * 2.3 ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // get past second short barrier
  Robot::chassis->moveToPoint(TILE_LENGTH, -TILE_LENGTH * 2.5, 5000,
                              {.minSpeed = 127});

  // time catapult retract as we go under elevation bar
  waitUntil([] {
    return Robot::chassis->getPose().x > -TILE_LENGTH * .75 ||
           !isMotionRunning();
  });
  printf("fire\n");
  Robot::Subsystems::catapult->fire();

  // wait until the robot's x position is past the elevation bar
  waitUntil([] {
    return Robot::chassis->getPose().x > -TILE_LENGTH * .5 ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();
  Robot::chassis->setPose(Robot::chassis->getPose() + lemlib::Pose {0, -3},
                          false);

  // go to the right side of the goal
  Robot::chassis->moveToPose(TILE_LENGTH * 2.5, -TILE_LENGTH, UP, 5000);
  // wait until facing towards goal
  waitUntil([] { return robotAngDist(UP) < 15 || !isMotionRunning(); });
  // give a bit of time to ram
  Robot::chassis->cancelMotion();
  Robot::chassis->moveToPoint(TILE_LENGTH * 2.5, 0, 700, {.minSpeed = 127});
  Robot::chassis->waitUntilDone();
  printf("ram!!!\n");

  // remove triball from intake
  Robot::Actions::outtake();
  // second ram
  // back out of goal
  tank(-127, -127, 300, 3);
  tank(127, 127, 150, 3);
  // ram into goal
  Robot::chassis->moveToPoint(0, 100000000, 600, {.minSpeed = 127});
  Robot::chassis->waitUntilDone();
  // adjust y a bit: y = y + 4
  Robot::chassis->setPose(Robot::chassis->getPose().x,
                          Robot::chassis->getPose().y + 4,
                          Robot::chassis->getPose().theta);
  // back out of goal
  tank(-127, -127, 400, 0);

  Robot::Actions::intake();
  // go around center triballs
  lemlib::Pose aroundFrontTriballsTarget {TILE_LENGTH * .75,
                                          -TILE_LENGTH * 1.35};
  Robot::chassis->moveToPoint(aroundFrontTriballsTarget.x,
                              aroundFrontTriballsTarget.y, 5000,
                              {.minSpeed = 127});
  // wait until in front of goal to scoop triballs beside the goal
  Robot::chassis->waitUntil(12);
  Robot::Actions::expandBackWing();

  // wait until having gone around center triballs
  waitUntilDistToPose(aroundFrontTriballsTarget, 12, 100, true);
  Robot::chassis->cancelMotion();

  Robot::Actions::retractBackWing();

  // prevent sudden jolt
  tank(96, 96, 0, 0);
  tank(64, -64, 600, 5);

  // face towards center triballs
  Robot::chassis->moveToPose(TILE_LENGTH * 1.75, -TILE_LENGTH * 0.5, RIGHT - 20,
                             2000, {.minSpeed = 64});
  // wait until roughly facing towards center triballs
  waitUntil([] { return robotAngDist(RIGHT - 20) < 35 || !isMotionRunning(); });
  // expand the wings and outtake in preparation for ram into center
  Robot::Actions::expandBothWings();
  Robot::Actions::outtake();

  // wait until facing towards center triballs
  waitUntil([] { return robotAngDist(RIGHT - 20) < 10 || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  // ram into front of goal
  Robot::chassis->moveToPoint(MAX_X, -TILE_RADIUS - 2, 1000, {.minSpeed = 127});
  Robot::chassis->waitUntilDone();

  // adjust odom
  Robot::chassis->setPose(Robot::chassis->getPose() + lemlib::Pose {-4, 2},
                          false);

  // get out of the goal in the same way we went in the goal
  Robot::Actions::retractBothWings();
  const lemlib::Pose outOfTheGoalTarget1 {
      0 + Robot::Dimensions::drivetrainWidth / 2 + 2,
      Robot::chassis->getPose().y - 9, UP};
  Robot::chassis->moveToPose(outOfTheGoalTarget1.x, outOfTheGoalTarget1.y,
                             outOfTheGoalTarget1.theta, 3000,
                             {.forwards = false, .minSpeed = 64});
  // stop mid way to let triballs in wing go past
  Robot::chassis->waitUntil(24);
  Robot::chassis->cancelMotion();
  pros::delay(300);
  Robot::chassis->moveToPose(outOfTheGoalTarget1.x, outOfTheGoalTarget1.y,
                             outOfTheGoalTarget1.theta, 3000,
                             {.forwards = false, .minSpeed = 64});
  // wait until near pose
  waitUntil([&outOfTheGoalTarget1] {
    return !isMotionRunning() ||
           (/* outOfTheGoalTarget1.distance(Robot::chassis->getPose()) < 5 && */
            robotAngDist(outOfTheGoalTarget1.theta) < 10);
  });
  Robot::chassis->moveToPoint(Robot::Dimensions::drivetrainWidth / 2,
                              TILE_LENGTH, 1000);
  // second center ram
  // face towards center triballs
  Robot::chassis->moveToPose(TILE_LENGTH * 1.75, TILE_LENGTH * .4, RIGHT + 20,
                             2000,
                             {
                                 .lead = .7,
                                 .minSpeed = 64,
                             });
  pros::delay(500);
  // wait until facing towards center triballs
  waitUntil([] { return robotAngDist(RIGHT) < 10 || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  // expand the wings in preparation for ram into center
  Robot::Actions::expandBothWings();

  // ram into center second time
  Robot::chassis->moveToPoint(MAX_X, TILE_RADIUS, 1300, {.minSpeed = 127});
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractBothWings();

  // get out of goal
  lemlib::Pose outOfTheGoalTarget2 {Robot::Dimensions::drivetrainLength / 2 + 2,
                                    TILE_LENGTH - 3};
  Robot::chassis->moveToPoint(outOfTheGoalTarget2.x, outOfTheGoalTarget2.y,
                              1000, {.forwards = false, .minSpeed = 64});
  // wait near pose
  waitUntilDistToPose(outOfTheGoalTarget2, 9, 100, true);

  // then face down
  Robot::chassis->turnTo(0, -10000000, 1000);
  // wait until facing down
  waitUntil([] { return robotAngDist(DOWN) < 30 || !isMotionRunning(); });
  Robot::chassis->cancelMotion();

  // sweep triballs in barrier corner
  lemlib::Pose barrierCornerSweepTarget {
      TILE_LENGTH * 1.9,
      TILE_LENGTH * 2 - Robot::Dimensions::drivetrainWidth / 2 - 2};
  Robot::chassis->moveToPose(barrierCornerSweepTarget.x,
                             barrierCornerSweepTarget.y, LEFT - 15, 1500,
                             {.forwards = false, .minSpeed = 64});
  Robot::Actions::expandBackWing();
  // wait until near pose
  waitUntilDistToPose(barrierCornerSweepTarget, 8, 100, true);
  Robot::chassis->cancelMotion();

  // prepare to sweep triballs next to matchload zone
  Robot::chassis->moveToPose(TILE_RADIUS,
                             MAX_Y - Robot::Dimensions::drivetrainWidth / 2 - 1,
                             LEFT, 3000, {.lead = .7});
  // wait until a bit before retracting back wing
  pros::delay(300);
  Robot::Actions::retractBackWing();
  // wait until y position is just barely touching field perimeter
  waitUntil([] {
    return Robot::chassis->getPose().y >
               MAX_Y - Robot::Dimensions::drivetrainWidth / 2 - 1 ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // ram into left side of goal in an arc
  Robot::chassis->moveToPose(MAX_X - Robot::Dimensions::drivetrainWidth / 2 - 4,
                             TILE_LENGTH, UP, 3000,
                             {.forwards = false, .minSpeed = 64});
  // // wait until next to matchload zone to deploy back wing
  // waitUntil([] {
  //   return Robot::chassis->getPose().x > TILE_LENGTH * 2 ||
  //   !isMotionRunning();
  // });
  // Robot::Actions::expandBackWing();

  // // wait until past matchload zone to retract back wing
  // waitUntil([] {
  //   return Robot::chassis->getPose().y < TILE_LENGTH * 2 ||
  //   !isMotionRunning();
  // });
  // Robot::Actions::retractBackWing();

  // wait until near goal to ram at full power
  waitUntil([] {
    return Robot::chassis->getPose().y < TILE_LENGTH * 1.5 ||
           !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // ram at full power into left side of goal
  Robot::chassis->moveToPoint(Robot::chassis->getPose().x, TILE_LENGTH, 800,
                              {.forwards = false, .minSpeed = 127});
  Robot::chassis->waitUntilDone();

  // second ram
  // back out of goal
  tank(127, 127, 400, 3);
  // ram into goal
  Robot::chassis->moveToPoint(0, -10000000, 800,
                              {.forwards = false, .minSpeed = 127});
  Robot::chassis->waitUntilDone();
  // back out of goal
  tank(64, 127, 400, 0);

  // elevation
  Robot::chassis->moveToPose(0, TILE_LENGTH * 2.5, LEFT, 5000,
                             {.minSpeed = 127});
  Robot::chassis->waitUntil(12);
  Robot::Subsystems::lift->extend();

  // wait until 2 seconds left in auton to retract the lift
  waitUntil(
      [&timer] { return !isMotionRunning() || timer.getTimeLeft() < 2000; });

  // make sure we're on the bar by ramming into it
  Robot::chassis->moveToPoint(-10000000, 0, 5000, {.minSpeed = 127});
  // wait until 1.2 seconds left in auton to cancel motion
  waitUntil(
      [&timer] { return !isMotionRunning() || timer.getTimeLeft() < 1200; });
  Robot::chassis->cancelMotion();

  // let the bot chill
  stop();
  pros::delay(900);

  // then move robot up
  // Robot::Subsystems::lift->retract();
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
