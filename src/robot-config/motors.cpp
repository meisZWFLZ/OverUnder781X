#include "pros/motors.h"
#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA {16, pros::E_MOTOR_GEAR_600, true};
pros::Motor leftDriveB {10, pros::E_MOTOR_GEAR_600, true};
// right
pros::Motor rightDriveA {18, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveB {13, pros::E_MOTOR_GEAR_600, /* true */};

// intake
pros::Motor intakeA {12, true};

// shooter
pros::Motor shooterA {14, true};
pros::Motor shooterB {6, true};
pros::Motor shooterC {9};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive {leftDriveA, /* leftDriveB */};
pros::Motor_Group Robot::Motors::rightDrive {rightDriveA, /* rightDriveB */};
pros::Motor_Group Robot::Motors::intake {intakeA};
pros::Motor_Group Robot::Motors::shooter {shooterA, shooterB, shooterC};