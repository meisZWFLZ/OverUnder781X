#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

ASSET(skills_right_side_txt);
ASSET(skills_front_1_txt);
ASSET(skills_front_2_txt);

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
  using namespace fieldDimensions;
  using namespace auton::actions;
  Robot::chassis->setPose(leftStartingPose, false);
  // @todo add skills auton

  // prepareForMatchloading();

  Robot::chassis->moveTo(MIN_X + TILE_LENGTH - 8, MIN_Y + TILE_LENGTH - 2,
                         LEFT - 22.5, 3000);

  Robot::chassis->waitUntil(6);
  Robot::Actions::shoot();
  Robot::chassis->waitUntilDone();
  Robot::Actions::lowerIntake();

  delayForMatchLoading(35000);
  Robot::Actions::raiseIntake();
  Robot::Actions::stopShooter();

  Robot::chassis->turnTo(0, MIN_Y + 12, 1000);
  Robot::chassis->waitUntilDone();
  Robot::chassis->follow(skills_right_side_txt, 15, 4000);

  Robot::chassis->waitUntil(50);
  Robot::Actions::intake();
  Robot::chassis->waitUntil(112);
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();
  Robot::Actions::stopIntake();

  Robot::chassis->tank(-127, -127);
  pros::delay(400);
  Robot::chassis->tank(127, 127);
  pros::delay(600);
  
  Robot::chassis->tank(-127, -127);
  pros::delay(300);
  Robot::chassis->tank(0, 0);

  Robot::chassis->turnTo(-10000, 0, 3000);
  Robot::chassis->waitUntilDone();
  Robot::chassis->follow(skills_front_1_txt, 15, 3000);
  Robot::chassis->waitUntil(36);
  Robot::Actions::expandWings();
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractWings();
  Robot::chassis->tank(-127, -127);
  pros::delay(400);

  Robot::chassis->turnTo(0, 10000, 3000);
  Robot::chassis->waitUntilDone();
  Robot::chassis->follow(skills_front_2_txt, 15, 3000);
  // Robot::Actions::expandWings();
  Robot::Actions::outtake();
  Robot::chassis->waitUntilDone();
  Robot::Actions::retractWings();
  Robot::Actions::stopIntake();
  Robot::chassis->tank(-127, -96);
  pros::delay(200);
  Robot::chassis->tank(0, 0);
}

auton::Auton auton::autons::skills = {(char*)("skills"), runSkills};
