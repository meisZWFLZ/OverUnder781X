#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::leftDrive {16, true};
pros::Rotation Robot::Sensors::rightDrive {17, true};

pros::Rotation Robot::Sensors::vert {19};
pros::Rotation Robot::Sensors::hori {6, true};
pros::Imu Robot::Sensors::imu {20};