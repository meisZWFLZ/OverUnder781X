#include "pros/motors.h"
#include "lemlib/chassis/differential.hpp"
#include "pros/abstract_motor.hpp"
#include "robot.h"
#include <memory>

// drivetrain
// left
pros::Motor leftDriveA {-5, pros::v5::MotorGears::blue};
pros::Motor leftDriveB {-9, pros::v5::MotorGears::blue};
// right
pros::Motor rightDriveA {10, pros::v5::MotorGears::blue, /* true */};
pros::Motor rightDriveB {18, pros::v5::MotorGears::blue, /* true */};

// intake
pros::Motor intakeA {-16};

// shooter
pros::Motor shooterA {-21};
pros::Motor shooterB {-11};
pros::Motor shooterC {12};

// robot groups
std::shared_ptr<pros::MotorGroup> Robot::Motors::leftDrive = nullptr; 
std::shared_ptr<pros::MotorGroup> Robot::Motors::rightDrive = nullptr;
pros::MotorGroup Robot::Motors::intake {intakeA};
pros::MotorGroup Robot::Motors::shooter {shooterA.get_port(), shooterB.get_port(),shooterC.get_port()};

void Robot::initializeMotorConfig() {
  Robot::Motors::leftDrive = lemlib::makeMotorGroup({leftDriveA.get_port(), leftDriveB.get_port()}, pros::v5::MotorGears::blue);
  Robot::Motors::rightDrive = lemlib::makeMotorGroup({rightDriveA.get_port(), rightDriveB.get_port()}, pros::v5::MotorGears::blue);
}