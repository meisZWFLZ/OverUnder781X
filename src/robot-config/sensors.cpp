#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {14, false};
pros::Rotation Robot::Sensors::hori {7, false};
pros::Imu Robot::Sensors::imu {3};
pros::ADILineSensor Robot::Sensors::cataTriball {'H'};
pros::Rotation Robot::Sensors::cata {15};