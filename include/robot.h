#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "okapi/impl/device/controller.hpp"
#include "lemlib/chassis/chassis.hpp"
class Robot {
  public:
    class Motors {
      public:
        static pros::Motor_Group leftDrive;
        static pros::Motor_Group rightDrive;

        static pros::Motor_Group intake;

        static pros::Motor_Group topShooter;
        static pros::Motor_Group bottomShooter;
    };
    class Pistons {
      public: 
        /** starts retracted */
        static pros::ADIDigitalOut intakeElevator;
        static pros::ADIDigitalOut wings;
    };
    class Sensors {
      public:
        // pid sensors (drivetrain wheels)
        // static pros::Rotation leftDrive;
        // static pros::Rotation rightDrive;
        // odometry sensors (dedicated odom wheels)
        static pros::Rotation vert;
        static pros::Rotation hori;
        static pros::Imu imu;
    };

    class Dimensions {
      public:
        static const float trackWidth;
        static const float driveWheelDiameter;
        static const float driveWheelRpm;
        static const float driveEncGearRatio;

        static const float vertEncDiameter;
        static const float vertEncDistance;
        static const float vertEncGearRatio;

        static const float horiEncDiameter;
        static const float horiEncDistance;
        static const float horiEncGearRatio;

        static constexpr float drivetrainWidth = 13.5;
        static constexpr float drivetrainLength = 15;
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
      static void prepareIntake();

      static void expandWings();
      static void retractWings();
    };
    class Tunables {
      public:
        static lemlib::ControllerSettings lateralController;
        static lemlib::ControllerSettings angularController;
        static const float chasePower;
    };
    static void initializeOdometryConfig(); 
    static lemlib::OdomSensors *odomSensors;
    static okapi::Controller control;
    static lemlib::Chassis *chassis;
};