#include "robot.h"
#include "lemlib/chassis/trackingWheel.hpp"

const float Robot::Dimensions::trackWidth = 11.25 - 1.0/32;
const float Robot::Dimensions::driveWheelDiameter = 3.25;
const float Robot::Dimensions::driveWheelRpm = 360;
const float Robot::Dimensions::driveEncGearRatio = 2;

const float Robot::Dimensions::vertEncDiameter = lemlib::Omniwheel::NEW_275;
const float Robot::Dimensions::vertEncDistance =
    -1.0 / 8 /* Robot::Dimensions::trackWidth / 2 */;
const float Robot::Dimensions::vertEncGearRatio = 1;

const float Robot::Dimensions::horiEncDiameter = lemlib::Omniwheel::NEW_275;
const float Robot::Dimensions::horiEncDistance = -5.274;
const float Robot::Dimensions::horiEncGearRatio = 1;

const int Robot::Dimensions::autonSelectorGearTeeth = 12;