#include "auton.h"
#include "pros/motors.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;
using namespace auton::utils;

void auton::actions::prepareForMatchloading() {
  tank(-64, -64, 0, 0);
  Robot::Motors::leftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  Robot::Motors::rightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  waitUntil([] {
    const float angDist = robotAngDist(10);
    printf("angDist: %f\n", angDist);
    return angDist < 5;
  }, 0, 3000);
  Robot::Motors::leftDrive.brake();
  Robot::Motors::rightDrive.brake();
}