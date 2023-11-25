#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {2};
pros::Rotation Robot::Sensors::hori {-21};
pros::Imu Robot::Sensors::imu {11};