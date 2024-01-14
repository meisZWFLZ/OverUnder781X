#include "pros/rtos.hpp"
#include "robot.h"

void Robot::Actions::prepareRobot() {
  new pros::Task([]() {
    Robot::Pistons::blocker.set_value(true);
    pros::delay(200);
    Robot::Pistons::blocker.set_value(false);
  });
}