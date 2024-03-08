#pragma once

#include "pros/adi.hpp"

class LiftArmStateMachine {
  public:
    // our possible states
    enum STATE { IDLE, RETRACTING, EXTENDING };

    // initializes state machine object and gives the machine the solenoids
    LiftArmStateMachine(pros::ADIDigitalOut* retract,
                        pros::ADIDigitalOut* extend);

    // retrieve the current state of the state machine
    STATE getState() const;

    // state changes
    // set the state to retracting
    void retract();
    // set the state to idle
    void release();
    // set the state to extending
    void extend();

    // run every 10 ms
    void update();

    // these members cannot be accessed from outside the class
  private:
    // the solenoids
    pros::ADIDigitalOut* retractPiston;
    pros::ADIDigitalOut* extendPiston;
    // the current state
    STATE state = IDLE;
};