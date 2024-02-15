#include "lemlib/chassis/chassis.hpp"
#include "robot.h"

// forward/backward PID
lemlib::ControllerSettings Robot::Tunables::lateralController {
    10, // kP
    0, // kI
    30, // kD
    0, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// turning PID
lemlib::ControllerSettings Robot::Tunables::angularController {
    7.5, // kP
    0, // kI
    43, // kD
    0, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3.5, // largeErrorRange
    350, // largeErrorTimeout
    0 // slew rate
};

// chasePower
const float Robot::Tunables::chasePower = 100;
