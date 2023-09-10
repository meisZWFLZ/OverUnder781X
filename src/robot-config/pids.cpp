#include "robot.h"

// forward/backward PID
lemlib::ChassisController_t Robot::PIDs::lateralController {
    20, // kP
    40, // kD
    1, // smallErrorRange
    500, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    50 // slew rate
};

// turning PID
lemlib::ChassisController_t Robot::PIDs::angularController {
    0, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};