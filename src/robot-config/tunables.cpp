#include "lemlib/chassis/chassis.hpp"
#include "robot.h"

// forward/backward PID
lemlib::ControllerSettings Robot::Tunables::lateralController {
    20, // kP
    25, // kD
    1, // smallErrorRange
    500, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    50 // slew rate
};

// turning PID
lemlib::ControllerSettings Robot::Tunables::angularController {
    9, // kP
    50, // kD`
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// chasePower
const float Robot::Tunables::chasePower = 1000;