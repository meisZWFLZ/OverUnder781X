#include "lemlib/api.hpp"
#include "lemlib/chassis/differential.hpp"
#include "robot.h"

lemlib::OdomSensors* Robot::odomSensors = nullptr;
lemlib::Differential* Robot::chassis = nullptr;

void Robot::initializeOdometryConfig() {
  lemlib::Drivetrain drivetrain {
      Robot::Motors::leftDrive,         Robot::Motors::rightDrive,
      Robot::Dimensions::trackWidth,    Robot::Dimensions::driveWheelDiameter,
      Robot::Dimensions::driveWheelRpm, Robot::Tunables::chasePower};

  lemlib::TrackingWheel* leftVert =
      Robot::Sensors::vert.get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(
                Robot::Sensors::vert.get_port(),
                Robot::Dimensions::vertEncDiameter,
                -Robot::Dimensions::vertEncDistance,
                Robot::Dimensions::vertEncGearRatio /* 300 */ /* 1 */)
          : nullptr;

  lemlib::TrackingWheel* rightVert = nullptr;
  // Robot::Sensors::vert.get_angle() != PROS_ERR
  //     ? nullptr
  //     : new lemlib::TrackingWheel(&Robot::Sensors::rightDrive,
  //                                 Robot::Dimensions::driveWheelDiameter,
  //                                 -Robot::Dimensions::trackWidth / 2,
  //                                 Robot::Dimensions::driveEncGearRatio);
  lemlib::TrackingWheel* hori =
      (&Robot::Sensors::hori)->get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(Robot::Sensors::hori.get_port(),
                                      Robot::Dimensions::horiEncDiameter,
                                      Robot::Dimensions::horiEncDistance,
                                      Robot::Dimensions::horiEncGearRatio)
          : nullptr;

  Robot::odomSensors =
      new lemlib::OdomSensors {leftVert, rightVert /* nullptr */, hori, nullptr,
                               nullptr /* &Robot::Sensors::imu */};

  Robot::chassis =
      new lemlib::Differential {drivetrain, Robot::Tunables::lateralController,
                           Robot::Tunables::angularController, *odomSensors};
}