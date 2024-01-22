#include "auton.h"
#include "pros/motors.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

void auton::actions::prepareForMatchloading() {
  tank(-64, -64, 0, 0);
  // Robot::Motors::rightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  // Robot::Motors::rightDrive.brake();
  waitUntil([] {
    const float angDist = robotAngDist(40);
    printf("angDist: %f\n", angDist);
    return angDist < 25;
  });
  // Robot::Actions::lowerIntake();
}