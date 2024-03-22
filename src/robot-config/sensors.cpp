#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::vert {14, false};
pros::Rotation Robot::Sensors::hori {7, false};
pros::Imu Robot::Sensors::imuA {3};
pros::Imu Robot::Sensors::imuB {4};
pros::Imu Robot::Sensors::imuC {16};
pros::ADILineSensor Robot::Sensors::cataElevationBar {'B'};
pros::Rotation Robot::Sensors::cata {5, true};