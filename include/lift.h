#pragma once

#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include "pros/motors.hpp"
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

    LiftArmStateMachine(pros::Motor_Group* liftMotors);

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

    void tareAngle();

    static float minAngle;
    static float maxAngle;
  private:
    void stopMotors();
    pros::motor_brake_mode_e_t getProsStoppingMode() const;

    void moveWithBangBang();
    void moveWithPID();
    void moveWithInternalPID();

    std::vector<double> getAngles() const;
    std::vector<double> calcError() const;
    double calcMaxError() const;
    bool isErrorInRange() const;

    /**
     * @brief if a motor is at current limit, adjust the min/max angle to be
     * that motor's current angle
     */
    void adjustMaxAngle();
    
    lemlib::Timer maxCurrentTimer;

    static const PIDControllerSettings pidSettings;

    static const float acceptableErrorRange;
    static const float BANG_BANG_POWER;

    std::vector<lemlib::PID> pidController;

    pros::Motor_Group* motors;

    float target = 0;
    STATE state = STOPPED;
    STOP_MODE stopMode = BRAKE;
    CONTROLLER_MODE controllerMode = PID;
};