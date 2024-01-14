#pragma once
#include "catapult.h"
#include "lift.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "okapi/impl/device/controller.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rotation.hpp"
#include "driverFeedback.h"
#include "controllerScreen.h"

class Robot {
  public:
    class Motors {
      public:
        static pros::Motor_Group leftDrive;
        static pros::Motor_Group rightDrive;

        static pros::Motor_Group intake;

        static pros::Motor_Group elevator;
        static pros::Motor_Group catapult;
    };

    class Pistons {
      public:
        /** starts retracted */
        static pros::ADIDigitalOut blocker;
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
        static pros::ADILineSensor cataTriball;
        static pros::Rotation cata;
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

        static constexpr float drivetrainWidth = 14.5;
        static constexpr float drivetrainLength = 14;
    };

    class Actions {
      public:
        static void outtake();
        static void intake();
        static void stopIntake();

        static void shoot();
        static void matchload();
        static void stopShooter();

        static void expandWings();
        static void retractWings();

        static void expandBlocker();
        static void retractBlocker();

        static void prepareRobot();
    };

    class Tunables {
      public:
        static lemlib::ControllerSettings lateralController;
        static lemlib::ControllerSettings angularController;
        static const float chasePower;
    };

    class Subsystems {
      public:
        static CatapultStateMachine* catapult;
        static LiftArmStateMachine* lift;
        static DriverFeedback* feedback;
        static ControllerScreen* controller;

        static pros::Task* task;

        static void initialize();
        static void update();
    };

    static void initializeOdometryConfig();
    static lemlib::OdomSensors* odomSensors;
    static pros::Controller control;
    static lemlib::Chassis* chassis;
};