#include "lemlib/chassis/chassis.hpp"
#include "robot.h"

// forward/backward PID
lemlib::ControllerSettings Robot::Tunables::lateralController {
    14.25, // kP
    0, // kI
    64, // kD
    3, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// turning PID
lemlib::ControllerSettings Robot::Tunables::angularController {
    3.6875, // kP
    0, // kI
    32, // kD
    15, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3.5, // largeErrorRange
    350, // largeErrorTimeout
    0 // slew rate
};

// chasePower
const float Robot::Tunables::chasePower = 100;

// IMU gains
const float Robot::Tunables::imuAGain =
    (360 * 10) / 3586.60 * (360 * 10) / 3601.61 * (360 * 10) /
    ((3604.66 + 3606.07 + 3605.91 + 3605.32 + 3604.84) / 5);
const float Robot::Tunables::imuBGain =
    (360 * 10) / 3589.76 * (360 * 10) / 3601.27 * (360 * 10) /
    ((3606.91 + 3604.99 + 3603.55 + 3602.21 + 3602.45) / 5);
const float Robot::Tunables::imuCGain =
    (360 * 10) / 3579.97 * (360 * 10) / 3582.93 * (360 * 10) /
    ((3605.22 + 3604.18 + 3603.45 + 3603.83 + 3603.02) / 5);

const float Robot::Tunables::driverWingJoystickThreshold = .85 * 127;