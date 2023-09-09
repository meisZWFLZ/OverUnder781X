#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::leftDrive{16};
pros::Rotation Robot::Sensors::rightDrive{17};

pros::Rotation Robot::Sensors::vert{19};
pros::Rotation Robot::Sensors::hori{6};
pros::Imu Robot::Sensors::imu{20};