#pragma once

#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include <vector>

struct PIDControllerSettings {
    float kP;
    float kI;
    float kD;
    float windupRange = 0;
    bool signFlipReset = true;
};

class LiftArmStateMachine {
  public:
    enum STATE { UNPOWERED, RETRACTING, EXTENDING };

    LiftArmStateMachine(pros::ADIDigitalOut* retract,
                        pros::ADIDigitalOut* extend);

    STATE getState() const;
    void retract();
    void release();
    void extend();

    void update();
  private:
    pros::ADIDigitalOut* retractPiston;
    pros::ADIDigitalOut* extendPiston;
    STATE state = UNPOWERED;
};