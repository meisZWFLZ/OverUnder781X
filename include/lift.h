#pragma once

#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

struct PIDControllerSettings {
    float kP;
    float kI;
    float kD;
    float windupRange = 0;
    bool signFlipReset = true;
};

class LiftArmStateMachine {
  public:
    enum STATE { EMERGENCY_STOPPED, STOPPED, MOVING };

    enum CONTROLLER_MODE {
      PID, // uses PID controller
      BANG_BANG, // uses bang bang controller
      INTERNAL_PID // uses internal PID controller
    };

    enum STOP_MODE {
      NEVER, // will just use controllers to hold position
      BRAKE, // uses pros brake mode
      HOLD, // uses pros hold mode
      COAST, // uses pros coast mode
    };

    LiftArmStateMachine(pros::Motor_Group* liftMotors,
                        pros::ADIPotentiometer* angleSensor);

    STATE getState() const;
    STOP_MODE getStoppingMode() const;
    CONTROLLER_MODE getControllerMode() const;

    void setStoppingMode(STOP_MODE);
    void setControllerMode(CONTROLLER_MODE);

    float getTarget() const;
    void setTarget(float newTarget);
    void changeTarget(float targetChange);

    void emergencyStop();
    void cancelEmergencyStop();

    void update();

    static constexpr float minAngle = 0;
    static constexpr float maxAngle = 330;
  private:
    void stopMotors();
    pros::motor_brake_mode_e_t getProsStoppingMode() const;

    void moveWithBangBang();
    void moveWithPID();
    void moveWithInternalPID();

    float calcError() const;
    bool isErrorInRange() const;

    static const PIDControllerSettings pidSettings;

    static const float acceptableErrorRange;
    static const float BANG_BANG_POWER;


    lemlib::PID pidController;

    pros::Motor_Group* motors;
    pros::ADIPotentiometer* angleSensor;

    float target = 0;
    STATE state = STOPPED;
    STOP_MODE stopMode = NEVER;
    CONTROLLER_MODE controllerMode = PID;
};