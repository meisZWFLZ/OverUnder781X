#include "robot.h"

// drivetrain
// left
pros::Motor leftDriveA {20, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor leftDriveB {19, pros::E_MOTOR_GEAR_600, true};
pros::Motor leftDriveC {18, pros::E_MOTOR_GEAR_600, true};

// right
pros::Motor rightDriveA {13, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveB {12, pros::E_MOTOR_GEAR_600, /* true */};
pros::Motor rightDriveC {11, pros::E_MOTOR_GEAR_600, true};

// intake
// 5.5w
pros::Motor intakeA {21, pros::E_MOTOR_GEAR_GREEN, true};

// elevator
// pros::Motor elevatorA {11, pros::E_MOTOR_GEAR_RED, true};
// pros::Motor elevatorB {18, pros::E_MOTOR_GEAR_RED, false};

// shooter
pros::Motor catapultA {9, pros::E_MOTOR_GEAR_RED, false};
// 5.5w
pros::Motor catapultB {4, pros::E_MOTOR_GEAR_GREEN, false};

// robot groups
pros::Motor_Group Robot::Motors::leftDrive {leftDriveA, leftDriveB, leftDriveC};
pros::Motor_Group Robot::Motors::rightDrive {rightDriveA, rightDriveB,
                                             rightDriveC};
pros::Motor_Group Robot::Motors::intake {intakeA};
pros::Motor_Group Robot::Motors::catapult {catapultA, catapultB};
pros::Motor_Group Robot::Motors::elevator {std::vector<pros::Motor> {}};