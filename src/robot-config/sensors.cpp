#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {15, false};
pros::Rotation Robot::Sensors::hori {6, false};
pros::Imu Robot::Sensors::imuA {7}; 
pros::Imu Robot::Sensors::imuB {5};
pros::Imu Robot::Sensors::imuC {16};
// not used
pros::ADILineSensor Robot::Sensors::cataElevationBar {'E'};
pros::Rotation Robot::Sensors::cata {9, true};
pros::Rotation Robot::Sensors::autonSelector {11};