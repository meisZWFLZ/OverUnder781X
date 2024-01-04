#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {9, false};
pros::Rotation Robot::Sensors::hori {6, false};
pros::Imu Robot::Sensors::imu {15};
pros::ADILineSensor Robot::Sensors::cataTriball {'H'};
pros::Rotation Robot::Sensors::cata {12};