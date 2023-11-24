#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {6};
pros::Rotation Robot::Sensors::hori {-15};
pros::Imu Robot::Sensors::imu {4};