#include "pros/rtos.hpp"
#include "robot.h"

void Robot::Actions::prepareIntake() {
    Robot::Actions::shoot();
    pros::delay(50);
    Robot::Actions::stopShooter();
}