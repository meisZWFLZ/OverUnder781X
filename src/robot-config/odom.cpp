#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "robot.h"

lemlib::OdomSensors* Robot::odomSensors = nullptr;
lemlib::Chassis* Robot::chassis = nullptr;

void Robot::initializeOdometryConfig() {
  lemlib::Drivetrain drivetrain {
      &Robot::Motors::leftDrive,        &Robot::Motors::rightDrive,
      Robot::Dimensions::trackWidth,    Robot::Dimensions::driveWheelDiameter,
      Robot::Dimensions::driveWheelRpm, Robot::Tunables::chasePower};

  lemlib::TrackingWheel* leftVert =
      Robot::Sensors::vert.get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(
                &Robot::Sensors::vert, Robot::Dimensions::vertEncDiameter,
                Robot::Dimensions::vertEncDistance,
                Robot::Dimensions::vertEncGearRatio /* 300 */ /* 1 */)
          : nullptr;

  lemlib::TrackingWheel* rightVert = nullptr;
  lemlib::TrackingWheel* hori =
      (&Robot::Sensors::hori)->get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(&Robot::Sensors::hori,
                                      Robot::Dimensions::horiEncDiameter,
                                      Robot::Dimensions::horiEncDistance,
                                      Robot::Dimensions::horiEncGearRatio)
          : nullptr;

  Robot::odomSensors =
      new lemlib::OdomSensors {leftVert, rightVert /* nullptr */, hori,
                                 nullptr, /* nullptr */ &Robot::Sensors::imu};

  Robot::chassis =
      new lemlib::Chassis {drivetrain, Robot::Tunables::lateralController,
                           Robot::Tunables::angularController, *odomSensors};
}