#pragma once
#include "main.h"

class Robot {
  public:
    class Motors {
      public:
        static pros::Motor_Group leftDrive;
        static pros::Motor_Group rightDrive;

        static pros::Motor_Group intake;

        static pros::Motor_Group shooter;
    };
    class Pistons {
      public: 
        /** starts retracted */
        static pros::ADIDigitalOut intakeElevator;
    };
    class Sensors {
      public:
        static pros::Rotation leftVert;
        static pros::Rotation rightVert;
        static pros::Rotation hori;
        static pros::Imu imu;
    };

    class Dimensions {
      public:
        static const float trackWidth;
        static const float driveWheelDiameter;
        static const float driveWheelRpm;

        static const float vertEncDiameter;
        static const float vertEncDistance;
        static const float vertEncGearRatio;

        static const float horiEncDiameter;
        static const float horiEncDistance;
        static const float horiEncGearRatio;
    };

    class PIDs {
      public:
        static lemlib::ChassisController_t lateralController;
        static lemlib::ChassisController_t angularController;
    };

    static okapi::Controller control;
    static lemlib::Chassis chassis;
};