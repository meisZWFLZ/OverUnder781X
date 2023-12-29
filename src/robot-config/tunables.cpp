#include "lemlib/chassis/chassis.hpp"
#include "robot.h"

// forward/backward PID
lemlib::ControllerSettings Robot::Tunables::lateralController {
    20, // kP
    0, // kI
    25, // kD
    0, // windup range
    1, // smallErrorRange
    500, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    50 // slew rate
};

// turning PID
lemlib::ControllerSettings Robot::Tunables::angularController {
    9, // kP
    0, // kI
    50, // kD
    0, // windup range
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// chasePower
const float Robot::Tunables::chasePower = 1000;

const PIDControllerSettings LiftArmStateMachine::pidSettings = {
    .kP = 10,
    .kI = 0.0,
    .kD = 2,
    .windupRange = 0.0,
    .signFlipReset = false};

const float LiftArmStateMachine::acceptableErrorRange = 8;
const float LiftArmStateMachine::BANG_BANG_POWER = 127;

float LiftArmStateMachine::minAngle = 0;
float LiftArmStateMachine::maxAngle = 295;