#include "pros/adi.hpp"
#include "robot.h"

pros::ADIDigitalOut Robot::Pistons::blocker {'A'};
pros::ADIDigitalOut Robot::Pistons::wings {'G'};
pros::ADIDigitalOut Robot::Pistons::liftLock {'B'};