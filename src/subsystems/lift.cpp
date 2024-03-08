#include "lift.h"
#include "lemlib/util.hpp"
#include <numeric>

LiftArmStateMachine::LiftArmStateMachine(pros::ADIDigitalOut* retract,
                                         pros::ADIDigitalOut* extend)
  : retractPiston(retract), extendPiston(extend) {
  this->retractPiston->set_value(false);
  this->extendPiston->set_value(false);
};

void LiftArmStateMachine::update() {
  switch (this->state) {
    // when in the idle
    case IDLE:
      this->retractPiston->set_value(false);
      this->extendPiston->set_value(false);
      break;
    case RETRACTING:
      this->retractPiston->set_value(true);
      this->extendPiston->set_value(false);
      break;
    case EXTENDING:
      this->retractPiston->set_value(false);
      this->extendPiston->set_value(true);
      break;
  }
}

void LiftArmStateMachine::extend() { this->state = EXTENDING; }
void LiftArmStateMachine::retract() { this->state = RETRACTING; }
void LiftArmStateMachine::release() { this->state = IDLE; }

LiftArmStateMachine::STATE LiftArmStateMachine::getState() const {
  return this->state;
}