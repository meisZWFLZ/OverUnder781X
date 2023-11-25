#include "lemlib/units.hpp"
#include "robot.h"

const Length Robot::Dimensions::trackWidth = 10.25_in;
const Length Robot::Dimensions::driveWheelDiameter = 3.25_in;
const AngularVelocity Robot::Dimensions::driveWheelRpm = 400_rpm;
const float Robot::Dimensions::driveEncGearRatio = 2;

const Length Robot::Dimensions::vertEncDiameter = 2.75_in;
const Length Robot::Dimensions::vertEncDistance = 0_in;
const float Robot::Dimensions::vertEncGearRatio = 1;

const Length Robot::Dimensions::horiEncDiameter = 2.75_in;
const Length Robot::Dimensions::horiEncDistance = -7.0_in / 8.0;
const float Robot::Dimensions::horiEncGearRatio = 1;