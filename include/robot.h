#pragma once
#include "lemlib/chassis/differential.hpp"
#include "main.h"
#include "lemlib/units.hpp"
#include "okapi/impl/device/controller.hpp"

class Robot {
  public:
    class Motors {
      public:
        static std::shared_ptr<pros::MotorGroup> leftDrive;
        static std::shared_ptr<pros::MotorGroup> rightDrive;

        static pros::MotorGroup intake;

        static pros::MotorGroup shooter;
    };
    class Pistons {
      public: 
        /** starts retracted */
        static pros::ADIDigitalOut intakeElevator;
        static pros::ADIDigitalOut wings;
    };
    class Sensors {
      public:
        // odometry sensors (dedicated odom wheels)
        static pros::Rotation vert;
        static pros::Rotation hori;
        static pros::Imu imu;
    };

    class Dimensions {
      public:
        static const Length trackWidth;
        static const Length driveWheelDiameter;
        static const AngularVelocity driveWheelRpm;
        static const float driveEncGearRatio;

        static const Length vertEncDiameter;
        static const Length vertEncDistance;
        static const float vertEncGearRatio;

        static const Length horiEncDiameter;
        static const Length horiEncDistance;
        static const float horiEncGearRatio;

        static constexpr Length drivetrainWidth = 13_in;
        static constexpr Length drivetrainLength = 13.75_in;
    };
    class Actions {
    public:
      static void outtake();
      static void intake();
      static void stopIntake();

      static void shoot();
      static void shootMacro();
      static void unshoot();
      static void stopShooter();

      static void raiseIntake();
      static void lowerIntake();

      static void expandWings();
      static void retractWings();
    };
    class Tunables {
      public:
        static lemlib::ControllerSettings<Length> lateralController;
        static lemlib::ControllerSettings<Angle> angularController;
        static const float chasePower;
    };
    static void initializeOdometryConfig(); 
    static void initializeMotorConfig(); 
    static lemlib::OdomSensors *odomSensors;
    static okapi::Controller control;
    static lemlib::Differential *chassis;
};