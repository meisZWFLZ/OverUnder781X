#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

pros::Rotation Robot::Sensors::rightVert{12};
pros::Rotation Robot::Sensors::hori{11};
pros::Imu Robot::Sensors::imu{21};