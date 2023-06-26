#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA{1};
pros::Motor leftDriveB{2};
// right
pros::Motor rightDriveA{3};
pros::Motor rightDriveB{4};

// intake
pros::Motor intakeA{5};
pros::Motor intakeB{6};

// shooter
pros::Motor shooterA{7};
pros::Motor shooterB{8};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive{leftDriveA, leftDriveB};
pros::Motor_Group Robot::Motors::rightDrive{rightDriveA, rightDriveB};
pros::Motor_Group Robot::Motors::intake{intakeA, rightDriveB};
pros::Motor_Group Robot::Motors::shooter{shooterA, rightDriveB};