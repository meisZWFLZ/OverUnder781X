#include "lift.h"
#include "lemlib/util.hpp"
#include <numeric>

double avg(const std::vector<double> nums) {
  return std::reduce(nums.begin(), nums.end(), 0) / double(nums.size());
}

pros::motor_brake_mode_e_t LiftArmStateMachine::getProsStoppingMode() const {
  switch (this->stopMode) {
    case NEVER: return pros::E_MOTOR_BRAKE_COAST;
    case BRAKE: return pros::E_MOTOR_BRAKE_BRAKE;
    case HOLD: return pros::E_MOTOR_BRAKE_HOLD;
    case COAST: return pros::E_MOTOR_BRAKE_COAST;
  }
  return pros::E_MOTOR_BRAKE_INVALID;
}

LiftArmStateMachine::LiftArmStateMachine(pros::Motor_Group* liftMotors)
  : motors(liftMotors), target(0), state(STOPPED), stopMode(BRAKE),
    controllerMode(PID), maxCurrentTimer(500) {
  this->pidController = std::vector<lemlib::PID>(
      this->motors->size(),
      lemlib::PID(pidSettings.kP, pidSettings.kI, pidSettings.kD,
                  pidSettings.windupRange, pidSettings.signFlipReset));

  this->motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  maxCurrentTimer.pause();
};

std::vector<double> LiftArmStateMachine::getAngles() const {
  return this->motors->get_positions();
}

void LiftArmStateMachine::tareAngle() { motors->tare_position(); }

std::vector<double> LiftArmStateMachine::calcError() const {
  std::vector<double> errs {};
  for (const auto ang : this->getAngles()) errs.push_back(this->target - ang);
  double avgError = avg(errs);
  // printf("err: %4.2f,%4.2f\n", errs[0], errs[1]);
  return std::vector<double>(this->motors->size(), avgError);
}

double LiftArmStateMachine::calcMaxError() const {
  const auto errs = this->calcError();
  return *std::max_element(errs.begin(), errs.end(), [](double a, double b) {
    return std::abs(a) < std::abs(b);
  });
}

bool LiftArmStateMachine::isErrorInRange() const {
  return abs(LiftArmStateMachine::calcMaxError()) < acceptableErrorRange;
}

void LiftArmStateMachine::stopMotors() {
  this->motors->set_brake_modes(this->getProsStoppingMode());
  this->motors->brake();
}

void LiftArmStateMachine::update() {
  switch (this->state) {
    case STOPPED:
      if (!this->isErrorInRange()) {
        printf("switch to moving\n");
        this->state = MOVING;
      } else this->stopMotors();
      break;
    case MOVING:
      if (this->isErrorInRange() && this->stopMode != NEVER) {
        printf("switch to stopped\n");
        for (auto pid : this->pidController) pid.reset();
        this->state = STOPPED;
        break;
      } else switch (this->controllerMode) {
          case PID: this->moveWithPID(); break;
          case BANG_BANG: this->moveWithBangBang(); break;
          case INTERNAL_PID: this->moveWithInternalPID(); break;
        }
      adjustMaxAngle();
      break;
    case EMERGENCY_STOPPED:
      this->motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
      this->motors->brake();
      break;
  }
}

void LiftArmStateMachine::adjustMaxAngle() {
  const auto isOverCurrent = this->motors->are_over_current();
  bool isAMotorAtTop = false;
  for (int i = 0; i < this->motors->size(); i++) {
    const auto m = this->motors->at(i);
    if (isOverCurrent[i] && m.get_position() > (maxAngle - minAngle) / 2) {
      isAMotorAtTop = true;
      if (this->maxCurrentTimer.paused == false) {
        if (this->maxCurrentTimer.isDone()) {
          this->maxAngle = std::min(float(m.get_position()), this->maxAngle);
        } else continue;
      }
    }
  }
  if (isAMotorAtTop) {
    this->maxCurrentTimer.resume();
  } else {
    this->maxCurrentTimer.reset();
    this->maxCurrentTimer.pause();
  }
}

void LiftArmStateMachine::moveWithPID() {
  const auto errs = this->calcError();
  for (int i = 0; i < this->motors->size(); i++) {
    this->motors->at(i).move(this->pidController[i].update(errs[i]));
  }
}

void LiftArmStateMachine::moveWithBangBang() {
  const auto errs = this->calcError();
  for (int i = 0; i < this->motors->size(); i++)
    this->motors->at(i).move(lemlib::sgn(errs[i]) *
                             LiftArmStateMachine::BANG_BANG_POWER);
}

void LiftArmStateMachine::moveWithInternalPID() {
  for (int i = 0; i < this->motors->size(); i++)
    this->motors->at(i).move_absolute(this->target, INT32_MAX);
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

void LiftArmStateMachine::setTarget(const float newTarget) {
  this->target = std::clamp(newTarget, minAngle, maxAngle);
}

void LiftArmStateMachine::changeTarget(const float targetChange) {
  this->setTarget(this->target + targetChange);
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

void LiftArmStateMachine::changeKP(const float change) {
  for (auto& pid : this->pidController) pid.kP += change;
}

void LiftArmStateMachine::changeKD(const float change) {
  for (auto& pid : this->pidController) pid.kD += change;
}

float LiftArmStateMachine::getKP() const { return this->pidController[0].kP; }

float LiftArmStateMachine::getKD() const { return this->pidController[0].kD; }