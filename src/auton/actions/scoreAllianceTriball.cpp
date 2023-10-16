#include "auton.h"
#include "pros/rtos.hpp"
#include "robot.h"

constexpr int BOTTOM_LEFT_X = -72;
constexpr int BOTTOM_LEFT_Y = -72;
constexpr int UP = 0;
void auton::actions::scoreAllianceTriball() {
  Robot::chassis->moveTo(BOTTOM_LEFT_X + 12,BOTTOM_LEFT_Y + 27, UP, 5000);
  pros::delay(100);
  Robot::Actions::outtake();
  pros::delay(250);
  Robot::chassis->tank(127,127);
  pros::delay(100);
  Robot::Actions::stopIntake();
  pros::delay(900);
  Robot::chassis->tank(-32,-32);
  pros::delay(500);
  Robot::chassis->tank(0,0);

}