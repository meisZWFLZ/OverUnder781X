#include "pros/rtos.hpp"
#include "robot.h"

void Robot::Actions::prepareIntake() {
    Robot::Actions::unshoot();
    pros::delay(50);
    Robot::Actions::stopShooter();
}