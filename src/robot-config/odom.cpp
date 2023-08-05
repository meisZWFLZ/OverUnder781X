#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "robot.h"

lemlib::Drivetrain_t drivetrain {
    &Robot::Motors::leftDrive, &Robot::Motors::rightDrive,
    Robot::Dimensions::trackWidth, Robot::Dimensions::driveWheelDiameter,
    Robot::Dimensions::driveWheelRpm};

lemlib::TrackingWheel* leftVert = new lemlib::TrackingWheel(
    &Robot::Motors::leftDrive, Robot::Dimensions::vertEncDiameter,
    -Robot::Dimensions::vertEncDistance, Robot::Dimensions::vertEncGearRatio);

lemlib::TrackingWheel* rightVert = new lemlib::TrackingWheel(
    &Robot::Motors::rightDrive, Robot::Dimensions::vertEncDiameter,
    Robot::Dimensions::vertEncDistance, Robot::Dimensions::vertEncGearRatio);

lemlib::TrackingWheel* hori = new lemlib::TrackingWheel(
    &Robot::Sensors::hori, Robot::Dimensions::horiEncDiameter,
    Robot::Dimensions::horiEncDistance, Robot::Dimensions::horiEncGearRatio);

lemlib::OdomSensors_t sensors {leftVert, rightVert, hori, nullptr,
                               nullptr/* &Robot::Sensors::imu */};

lemlib::Chassis Robot::chassis {drivetrain, Robot::PIDs::lateralController,
                                Robot::PIDs::angularController, sensors};