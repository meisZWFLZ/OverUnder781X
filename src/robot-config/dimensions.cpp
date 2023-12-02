#include "robot.h"

const float Robot::Dimensions::trackWidth = 11 - (1.0 / 8);
const float Robot::Dimensions::driveWheelDiameter = 3.25;
const float Robot::Dimensions::driveWheelRpm = 400;
const float Robot::Dimensions::driveEncGearRatio = 2;

const float Robot::Dimensions::vertEncDiameter = 2.75;
const float Robot::Dimensions::vertEncDistance =
    2.5 /* Robot::Dimensions::trackWidth / 2 */;
const float Robot::Dimensions::vertEncGearRatio = 1;

const float Robot::Dimensions::horiEncDiameter = 2.75;
const float Robot::Dimensions::horiEncDistance = -3.874;
const float Robot::Dimensions::horiEncGearRatio = 1;