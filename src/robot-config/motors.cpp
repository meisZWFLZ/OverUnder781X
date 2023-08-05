#include "pros/motors.h"
#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA {12, pros::E_MOTOR_GEAR_600, true};
pros::Motor leftDriveB {18, pros::E_MOTOR_GEAR_600, true};
// right
pros::Motor rightDriveA {11, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveB {13, pros::E_MOTOR_GEAR_600, /* true */};

// intake
pros::Motor intakeA {10, true};

// shooter
pros::Motor shooterA {3};
pros::Motor shooterB {4, true};
pros::Motor shooterC {5};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive {leftDriveA, leftDriveB};
pros::Motor_Group Robot::Motors::rightDrive {rightDriveA, rightDriveB};
pros::Motor_Group Robot::Motors::intake {intakeA};
pros::Motor_Group Robot::Motors::shooter {shooterA, shooterB, shooterC};