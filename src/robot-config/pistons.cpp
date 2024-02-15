#include "pros/adi.hpp"
#include "robot.h"

pros::ADIDigitalOut Robot::Pistons::leftWing {'D'};
pros::ADIDigitalOut Robot::Pistons::rightWing {'C'};
pros::ADIDigitalOut Robot::Pistons::retractLift {'G'};
pros::ADIDigitalOut Robot::Pistons::extendLift {'A'};