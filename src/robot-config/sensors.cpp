#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "robot.h"

// pros::Rotation Robot::Sensors::leftVert = nullptr;
pros::Rotation Robot::Sensors::rightVert{12};
    /* Robot::Sensors::leftVert; */
pros::Rotation Robot::Sensors::hori{11};
pros::Imu Robot::Sensors::imu{21};