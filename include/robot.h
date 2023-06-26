// Contributes motors
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
  static okapi::Controller control;
};