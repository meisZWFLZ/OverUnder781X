#include "pros/motors.h"
#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA {2, pros::E_MOTOR_GEAR_600, true};
pros::Motor leftDriveB {4, pros::E_MOTOR_GEAR_600, true};
// right
pros::Motor rightDriveA {3, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveB {6, pros::E_MOTOR_GEAR_600, /* true */};

// intake
pros::Motor intakeA {5};
pros::Motor intakeB {6};

// shooter
pros::Motor shooterA {7};
pros::Motor shooterB {8};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive {leftDriveA, leftDriveB};
pros::Motor_Group Robot::Motors::rightDrive {rightDriveA, rightDriveB};
pros::Motor_Group Robot::Motors::intake {intakeA, rightDriveB};
pros::Motor_Group Robot::Motors::shooter {shooterA, rightDriveB};