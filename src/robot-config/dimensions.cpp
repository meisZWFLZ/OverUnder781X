#include "robot.h"

const float Robot::Dimensions::trackWidth = 10.25;
const float Robot::Dimensions::driveWheelDiameter = 3.25;
const float Robot::Dimensions::driveWheelRpm = 400;

const float Robot::Dimensions::vertEncDiameter =
    Robot::Dimensions::driveWheelDiameter;
const float Robot::Dimensions::vertEncDistance =
    Robot::Dimensions::trackWidth / 2;
const float Robot::Dimensions::vertEncGearRatio = 0.5;

const float Robot::Dimensions::horiEncDiameter = 3.25;
const float Robot::Dimensions::horiEncDistance = 6.5;
const float Robot::Dimensions::horiEncGearRatio = 1;