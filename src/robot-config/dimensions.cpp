#include "robot.h"

const float Robot::Dimensions::trackWidth = 10.25;
const float Robot::Dimensions::driveWheelDiameter = 3.25;
const float Robot::Dimensions::driveWheelRpm = 400;
const float Robot::Dimensions::driveEncGearRatio = 2;

const float Robot::Dimensions::vertEncDiameter =
    2.75;
const float Robot::Dimensions::vertEncDistance =
    0/* Robot::Dimensions::trackWidth / 2 */;
const float Robot::Dimensions::vertEncGearRatio = 1;

const float Robot::Dimensions::horiEncDiameter = 2.75;
const float Robot::Dimensions::horiEncDistance = 7.0/8.0;
const float Robot::Dimensions::horiEncGearRatio = 1;