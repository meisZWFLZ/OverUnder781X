#include "lift.h"
#include "lemlib/util.hpp"

pros::motor_brake_mode_e_t LiftArmStateMachine::getProsStoppingMode() const {
  switch (this->stopMode) {
    case NEVER: return pros::E_MOTOR_BRAKE_COAST;
    case BRAKE: return pros::E_MOTOR_BRAKE_BRAKE;
    case HOLD: return pros::E_MOTOR_BRAKE_HOLD;
    case COAST: return pros::E_MOTOR_BRAKE_COAST;
  }
}

LiftArmStateMachine::LiftArmStateMachine(pros::Motor_Group* liftMotors,
                                         pros::ADIPotentiometer* angleSensor)
  : motors(liftMotors), angleSensor(angleSensor),
    pidController(pidSettings.kP, pidSettings.kI, pidSettings.kD,
                  pidSettings.windupRange, pidSettings.signFlipReset) {};

float LiftArmStateMachine::calcError() const {
  return this->target - this->angleSensor->get_angle();
}

bool LiftArmStateMachine::isErrorInRange() const {
  return std::abs(this->calcError()) < acceptableErrorRange;
}

void LiftArmStateMachine::stopMotors() {
  this->motors->set_brake_modes(this->getProsStoppingMode());
  this->motors->brake();
}

void LiftArmStateMachine::update() {
  switch (this->state) {
    case STOPPED:
      if (!this->isErrorInRange()) this->state = MOVING;
      else this->stopMotors();
      break;
    case MOVING:
      if (this->isErrorInRange()) {
        this->pidController.reset();
        this->state = STOPPED;
      } else switch (this->controllerMode) {
          case PID: this->moveWithPID(); break;
          case BANG_BANG: this->moveWithBangBang(); break;
          case INTERNAL_PID: this->moveWithInternalPID(); break;
        }
      break;
    case EMERGENCY_STOPPED:
      this->motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
      this->motors->brake();
      break;
  }
}

void LiftArmStateMachine::moveWithPID() {
  this->motors->move(this->pidController.update(this->calcError()));
}

void LiftArmStateMachine::moveWithBangBang() {
  this->motors->move(lemlib::sgn(this->calcError()) *
                     LiftArmStateMachine::BANG_BANG_POWER);
}

void LiftArmStateMachine::moveWithInternalPID() {
  this->motors->move_absolute(this->target, INT32_MAX);
}

void LiftArmStateMachine::emergencyStop() { this->state = EMERGENCY_STOPPED; }

void LiftArmStateMachine::cancelEmergencyStop() { this->state = STOPPED; }

void LiftArmStateMachine::setStoppingMode(STOP_MODE newMode) {
  this->stopMode = newMode;
}

void LiftArmStateMachine::setControllerMode(CONTROLLER_MODE newMode) {
  this->controllerMode = newMode;
}

float LiftArmStateMachine::getTarget() const { return this->target; }

void LiftArmStateMachine::setTarget(float newTarget) {
  this->target = newTarget;
}

void LiftArmStateMachine::changeTarget(float targetChange) {
  this->target += targetChange;
}

LiftArmStateMachine::STATE LiftArmStateMachine::getState() const {
  return this->state;
}

LiftArmStateMachine::STOP_MODE LiftArmStateMachine::getStoppingMode() const {
  return this->stopMode;
}

LiftArmStateMachine::CONTROLLER_MODE
LiftArmStateMachine::getControllerMode() const {
  return this->controllerMode;
}