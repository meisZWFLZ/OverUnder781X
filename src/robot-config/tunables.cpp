#include "lemlib/chassis/chassis.hpp"
#include "lemlib/units.hpp"
#include "robot.h"

// forward/backward PID
lemlib::ControllerSettings<Length> Robot::Tunables::lateralController {
    20, // kP
    25, // kD
    1_in, // smallErrorRange
    500_ms, // smallErrorTimeout
    3_in, // largeErrorRange
    500_ms, // largeErrorTimeout
    50 // slew rate
};

// turning PID
lemlib::ControllerSettings<Angle> Robot::Tunables::angularController {
    9, // kP
    50, // kD`
    1_deg, // smallErrorRange
    100_ms, // smallErrorTimeout
    3_deg, // largeErrorRange
    500_ms, // largeErrorTimeout
    0 // slew rate
};

// chasePower
const float Robot::Tunables::chasePower = 1000;