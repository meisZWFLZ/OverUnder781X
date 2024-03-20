#pragma once
#include "catapult.h"
#include "lift.h"
#include "lemlib/chassis/chassis.hpp"
#include "driverFeedback.h"
#include "controllerScreen.h"
#include "wings.h"

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
        static std::unique_ptr<FourWingSubsystem::PortConfig> wingConfig;
        static pros::ADIDigitalOut extendLift;
        static pros::ADIDigitalOut retractLift;
    };

    class Sensors {
      public:
        // pid sensors (drivetrain wheels)
        // static pros::Rotation leftDrive;
        // static pros::Rotation rightDrive;
        // odometry sensors (dedicated odom wheels)
        static pros::Rotation vert;
        static pros::Rotation hori;
        static pros::Imu imuA;
        static pros::Imu imuB;
        static pros::Imu imuC;
        static pros::ADILineSensor cataElevationBar;
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
        static constexpr float drivetrainLength = 25 / 2.0;
    };

    class Actions {
      public:
        static void outtake();
        static void intake();
        static void stopIntake();

        static void shoot();
        static void matchload();
        static void stopShooter();

        static void expandBothWings();
        static void expandLeftWing();
        static void expandRightWing();

        static void retractBothWings();
        static void retractLeftWing();
        static void retractRightWing();

        static void toggleBothWings();
        static void toggleLeftWing();
        static void toggleRightWing();

        static void retractBackWing();
        static void expandBackWing();
        static void toggleBackWing();

        static void prepareRobot();

        static void switchToMatchloadingIMU();
        static void switchToNormalIMU();
    };

    class Tunables {
      public:
        static lemlib::ControllerSettings lateralController;
        static lemlib::ControllerSettings angularController;
        static const float chasePower;
        static const float imuAGain;
        static const float imuBGain;
        static const float imuCGain;
        static const float driverWingJoystickThreshold;
    };

    class Subsystems {
      public:
        static CatapultStateMachine* catapult;
        static LiftArmStateMachine* lift;
        static DriverFeedback* feedback;
        static ControllerScreen* controller;
        static FourWingSubsystem* wings;

        static pros::Task* task;

        static void initialize();
        static void update();
    };

    static void initializeOdometry();
    static lemlib::OdomSensors* odomSensors;
    static pros::Controller control;
    static lemlib::Chassis* chassis;
};