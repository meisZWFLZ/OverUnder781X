#include "pros/rtos.hpp"
#include "robot.h"

void Robot::Actions::shootMacro() {
  Robot::Actions::shoot();
  Robot::Actions::intake();
  Robot::chassis->tank(-127,-127);
  pros::Task::delay(100);
  Robot::chassis->tank(127,127);
  pros::Task::delay(100);
  Robot::chassis->tank(0,0);
  pros::Task::delay(350);
  Robot::Actions::stopShooter();
  Robot::Actions::stopIntake();
  // if (strcmp(pros::Task::current().get_name(), "shootMacro") == 0)
  // pros::Task::current().suspend();
}