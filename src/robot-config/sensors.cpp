#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {6, true};
pros::Rotation Robot::Sensors::hori {15, true};
pros::Imu Robot::Sensors::imu {5};
pros::ADILineSensor Robot::Sensors::cataTriball {'H'};
pros::Rotation Robot::Sensors::cata {19};