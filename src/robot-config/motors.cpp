#include "pros/motors.h"
#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA {5, pros::E_MOTOR_GEAR_600, true};
pros::Motor leftDriveB {9, pros::E_MOTOR_GEAR_600, true};
// right
pros::Motor rightDriveA {10, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveB {18, pros::E_MOTOR_GEAR_600, /* true */};

// intake
pros::Motor intakeA {16, true};

// shooter
pros::Motor elevatorA {16, false};
pros::Motor elevatorB {17, true};

// shooter
pros::Motor catapultA {21, true};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive {leftDriveA, leftDriveB};
pros::Motor_Group Robot::Motors::rightDrive {rightDriveA, rightDriveB};
pros::Motor_Group Robot::Motors::intake {intakeA};
pros::Motor_Group Robot::Motors::catapult {catapultA};
pros::Motor_Group Robot::Motors::elevator {elevatorA, elevatorB};