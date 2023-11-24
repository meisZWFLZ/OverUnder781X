#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

// pros::Rotation Robot::Sensors::leftDrive {15, true};
// pros::Rotation Robot::Sensors::rightDrive {17, true};

pros::Rotation Robot::Sensors::vert {6};
pros::Rotation Robot::Sensors::hori {15, true};
pros::Imu Robot::Sensors::imu {11};