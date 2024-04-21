#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

void printRobotPose() {
  const lemlib::Pose pose = Robot::chassis->getPose();

  printf("pose: (%4.2f,%4.2f,%4.2f)\n", pose.x, pose.y, pose.theta);
}

void score4Balls() {
  using namespace fieldDimensions;
  using namespace auton::actions;

  const int startTime = pros::millis();

  // same set as disrupt
  Robot::chassis->setPose(
      {0 - (-TILE_LENGTH * 2 + Robot::Dimensions::drivetrainWidth / 2 + 6),
       MIN_Y + TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2, UP},
      false);

  Robot::Actions::expandRightWing();

  // quickly intake first triball
  Robot::chassis->lateralSettings.slew = 7;
  Robot::Actions::intake();

  constexpr float intakeCenterTriballTargetTheta = UP - 15;
  constexpr float intakeCenterTriballTargetThetaRadians =
      lemlib::degToRad(90 - intakeCenterTriballTargetTheta);
  constexpr float distanceFromCenterTriball = 5;
  const lemlib::Pose centerTriball {TILE_LENGTH, 0};
  const lemlib::Pose intakeCenterTriballTarget =
      lemlib::Pose {centerTriball.x, centerTriball.y,
                    intakeCenterTriballTargetTheta} +
      (lemlib::Pose {float(cos(intakeCenterTriballTargetThetaRadians +
                               /* rotate by 180*/ M_PI)),
                     float(sin(intakeCenterTriballTargetThetaRadians +
                               /* rotate by 180*/ M_PI))} *
       distanceFromCenterTriball);

  Robot::chassis->moveToPose(
      intakeCenterTriballTarget.x, intakeCenterTriballTarget.y,
      intakeCenterTriballTarget.theta, 1100, {.chasePower = 6, .minSpeed = 72});
  Robot::chassis->waitUntil(6);
  Robot::Actions::retractRightWing();

  // wait until near triball to slow down
  waitUntil([] {
    return Robot::chassis->getPose().y > -TILE_LENGTH || !isMotionRunning();
  });
  Robot::chassis->cancelMotion();

  // slow down to intake triball
  Robot::chassis->moveToPose(
      intakeCenterTriballTarget.x, intakeCenterTriballTarget.y,
      intakeCenterTriballTarget.theta, 2000, {.chasePower = 6, .minSpeed = 48});
  // give a bit of time for triball sensing to be accurate
  pros::delay(300);
  // wait until robot intakes triball or robot center passes neutral line
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().y > -3;
      },
      50);
  Robot::chassis->cancelMotion();
  const float normalSlew = Robot::chassis->lateralSettings.slew;

  Robot::chassis->lateralSettings.slew = 5 /* 0 */;

  // go to matchload zone to remove the ball
  const lemlib::Pose removeMatchloadZoneTriballTarget {
      TILE_LENGTH * 2 + 8, -TILE_LENGTH * 2 + 0.5, UP - 45};
  Robot::chassis->moveToPose(removeMatchloadZoneTriballTarget.x,
                             removeMatchloadZoneTriballTarget.y,
                             removeMatchloadZoneTriballTarget.theta, 1500,
                             {.forwards = false, .minSpeed = 127});

  // slow down after a bit
  waitUntil([] {
    return Robot::chassis->getPose().y < -TILE_LENGTH * 1.65 ||
           isMotionRunning();
  });
  // don't spin the ball in the intake
  Robot::Actions::stopIntake();

  Robot::chassis->cancelMotion();
  Robot::chassis->lateralSettings.slew = normalSlew;
  Robot::chassis->moveToPose(
      removeMatchloadZoneTriballTarget.x, removeMatchloadZoneTriballTarget.y,
      removeMatchloadZoneTriballTarget.theta, 1500, {.forwards = false});
  waitUntilDistToPose(removeMatchloadZoneTriballTarget, 3, 50, true);
  Robot::chassis->cancelMotion();

  // turn to be parallel to the matchload bar
  Robot::chassis->turnToHeading(UP + 45, 500);
  // expand back wing and outtake early to compensate for latency
  pros::delay(300);
  Robot::Actions::expandBackWing();
  // outtake center ball into goal
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();

  // scoop matchload zone ball out
  Robot::chassis->swingToHeading(
      UP, lemlib::DriveSide::LEFT, 750,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
       .minSpeed = 127,
       .earlyExitRange = 30});
  Robot::chassis->waitUntilDone();

  // turn towards elevation bar
  Robot::chassis->turnToHeading(
      LEFT - 20, 750,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
       .minSpeed = 127,
       .earlyExitRange = 30});
  Robot::chassis->waitUntil(45);
  Robot::Actions::retractBackWing();
  Robot::chassis->waitUntilDone();
  // don't break back wing or unintentionally hit balls

  const lemlib::Pose intakeBallUnderElevationBarTarget {9, -TILE_LENGTH * 2.5,
                                                        LEFT};
  // intake ball under elevation bar
  Robot::chassis->moveToPose(
      intakeBallUnderElevationBarTarget.x, intakeBallUnderElevationBarTarget.y,
      intakeBallUnderElevationBarTarget.theta, 1000, {.minSpeed = 127});
  Robot::Actions::intake();

  // after passing short barrier, slow down
  waitUntil(
      [] {
        return !isMotionRunning() ||
               Robot::chassis->getPose().x < TILE_LENGTH * 1.1;
      },
      0, 250);
  Robot::chassis->moveToPose(intakeBallUnderElevationBarTarget.x,
                             intakeBallUnderElevationBarTarget.y,
                             intakeBallUnderElevationBarTarget.theta, 1000);
  printf("millis: %i\n", pros::millis() - startTime);

  // wait until triball in intake or too close to other tiles (we do not want to
  // risk a disqualification)
  waitUntil(
      [] {
        return isTriballInIntake() || !isMotionRunning() ||
               Robot::chassis->getPose().x <
                   Robot::Dimensions::drivetrainLength / 2 + 4;
      },
      50);
  Robot::chassis->cancelMotion();
  printf("millis: %i\n", pros::millis() - startTime);
  printf("a isTriballInIntake(): %i\n", isTriballInIntake());
  printf("x: %4.2f\n", Robot::chassis->getPose().x);
  printf("isNotionRunning: %i\n", isMotionRunning());
  printf("theta: %4.2f\n", Robot::chassis->getPose().theta);

  const lemlib::Pose plowBallsIntoSideOfGoalTarget {TILE_LENGTH * 2.625,
                                                    -TILE_LENGTH, DOWN};
  // immediately back away from other offensive zone
  // push balls into goal
  Robot::chassis->moveToPose(plowBallsIntoSideOfGoalTarget.x,
                             plowBallsIntoSideOfGoalTarget.y,
                             plowBallsIntoSideOfGoalTarget.theta, 1500,
                             {.forwards = false, .minSpeed = 127});
  // pros::delay(500);
  // Robot::Actions::stopIntake();

  // wait until facing up
  waitUntil([] { return robotAngDist(DOWN) < 20 || !isMotionRunning(); });

  // ram into goal backwards fast
  Robot::chassis->cancelMotion();
  Robot::chassis->moveToPose(plowBallsIntoSideOfGoalTarget.x,
                             plowBallsIntoSideOfGoalTarget.y,
                             plowBallsIntoSideOfGoalTarget.theta, 750,
                             {.forwards = false, .minSpeed = 127});
  Robot::chassis->waitUntilDone();

  // back out of goal
  tank(-127, -127, 400, 0);

  // spin around and outtake ball into goal
  Robot::chassis->turnToHeading(UP, 400,
                                {.minSpeed = 127, .earlyExitRange = 30});
  Robot::chassis->waitUntil(45);
  Robot::Actions::outtake();

  Robot::chassis->moveToPoint(Robot::chassis->getPose().x, 100000, 650);
  Robot::chassis->waitUntilDone();
  Robot::chassis->setPose(
      {Robot::chassis->getPose().x,
       -TILE_LENGTH - Robot::Dimensions::drivetrainLength / 2,
       Robot::chassis->getPose().theta});
  // go backwards
  Robot::chassis->moveToPoint(
      Robot::chassis->getPose().x, Robot::chassis->getPose().y - 12, 1000,
      {.forwards = false, .minSpeed = 127, .earlyExitRange = 2});

  // turn towards last balls
  const lemlib::Pose intake5thBallTarget {11, -TILE_LENGTH, LEFT};
  Robot::chassis->swingToPoint(intake5thBallTarget.x, intake5thBallTarget.y - 3,
                               lemlib::DriveSide::RIGHT, 750,
                               {.minSpeed = 127, .earlyExitRange = 45});
  Robot::chassis->waitUntilDone();
  tank(127, 127, 0, 0);
  waitUntil([] { return Robot::chassis->getPose().x < TILE_LENGTH * 2 + 5; }, 0,
            250);
  // intake last ball for 5 ball
  Robot::chassis->moveToPose(intake5thBallTarget.x, intake5thBallTarget.y,
                             intake5thBallTarget.theta, 1150,
                             {.lead = 0.3, .minSpeed = 96});
  Robot::Actions::intake();
  // let intake speed up so triball sensing is accurate
  pros::delay(500);
  waitUntil([] { return isTriballInIntake() || !isMotionRunning(); }, 50);
  Robot::chassis->cancelMotion();
}

void run5BallRush() {
  score4Balls();
  // turn towards goal
  const lemlib::Pose pushIntoLastBallsIntoGoalTarget {
      TILE_LENGTH * 2 - Robot::Dimensions::drivetrainLength / 2,
      -TILE_LENGTH * .375, RIGHT};
  Robot::chassis->swingToPoint(pushIntoLastBallsIntoGoalTarget.x,
                               pushIntoLastBallsIntoGoalTarget.y,
                               lemlib::DriveSide::RIGHT, 500,
                               {.direction = AngularDirection::CW_CLOCKWISE,
                                .minSpeed = 127,
                                .earlyExitRange = 45});
  Robot::chassis->moveToPose(
      pushIntoLastBallsIntoGoalTarget.x, pushIntoLastBallsIntoGoalTarget.y,
      pushIntoLastBallsIntoGoalTarget.theta, 2000, {.minSpeed = 80});
  // stop spinning triball in intake
  Robot::chassis->waitUntil(6);
  Robot::Actions::stopIntake();

  // outtake ball into goal once we are facing it
  waitUntil([] { return robotAngDist(RIGHT) < 35 || !isMotionRunning(); });
  Robot::Actions::outtake();

  Robot::chassis->cancelMotion();
  // ram straight into goal
  Robot::chassis->moveToPoint(1000000, Robot::chassis->getPose().y, 900,
                              {.minSpeed = 127});
  waitUntil([] {
    return Robot::chassis->getPose().x > TILE_LENGTH * 1.5 ||
           !isMotionRunning();
  });

  Robot::chassis->cancelMotion();
  Robot::chassis->setPose(
      {TILE_LENGTH * 2 - Robot::Dimensions::drivetrainLength / 2,
       Robot::chassis->getPose().y, Robot::chassis->getPose().theta});
  tank(-127, -64, 200, 3);
  Robot::chassis->swingToHeading(LEFT, lemlib::DriveSide::RIGHT, 500,
                                 {.minSpeed = 127, .earlyExitRange = 45});

  Robot::chassis->moveToPose(
      6, -TILE_LENGTH * 2 + Robot::Dimensions::drivetrainLength / 2 + 4, LEFT,
      1200,
      {
          .minSpeed = 127,
      });
  Robot::chassis->waitUntilDone();
  Robot::Actions::expandLeftWing();
  Robot::chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  tank(127, 127, 0);
  waitUntil([] { return Robot::Sensors::imuA.get_pitch() > 30; }, 50, INT_MAX,
            true);
  stop();
}

void runBallRushElims() {
  score4Balls();
  // turn towards goal
  const lemlib::Pose pushIntoLastBallsIntoGoalTarget {
      TILE_LENGTH * 2 - Robot::Dimensions::drivetrainLength / 2,
      -TILE_LENGTH * .4, RIGHT};
  Robot::chassis->swingToPoint(pushIntoLastBallsIntoGoalTarget.x,
                               pushIntoLastBallsIntoGoalTarget.y,
                               lemlib::DriveSide::RIGHT, 500,
                               {.direction = AngularDirection::CW_CLOCKWISE,
                                .minSpeed = 32,
                                .earlyExitRange = 5});
  Robot::chassis->waitUntilDone();
  Robot::Actions::outtake();

  pros::delay(500);
}

auton::Auton auton::autons::fiveRush = {.label = (char*)("5 ball rush + awp"),
                                        .run = run5BallRush,
                                        .labelForController =
                                            (char*)("5b rush + awp")};
auton::Auton auton::autons::sixRushElims = {(char*)("6 ball rush + elims"),
                                            runBallRushElims,
                                            (char*)("6b rush + elims")};