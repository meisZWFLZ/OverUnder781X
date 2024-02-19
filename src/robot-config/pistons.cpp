#include "pros/adi.hpp"
#include "robot.h"

pros::ADIDigitalOut Robot::Pistons::leftWing {'C'};
pros::ADIDigitalOut Robot::Pistons::rightWing {'B'};
pros::ADIDigitalOut Robot::Pistons::retractLift {'G'};
pros::ADIDigitalOut Robot::Pistons::extendLift {'A'};
pros::ADIDigitalOut Robot::Pistons::backWing {'D'};