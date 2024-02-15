#include "lift.h"
#include "lemlib/util.hpp"
#include <numeric>

double avg(const std::vector<double> nums) {
  return std::reduce(nums.begin(), nums.end(), 0) / double(nums.size());
}

LiftArmStateMachine::LiftArmStateMachine(pros::ADIDigitalOut* retract,
                                         pros::ADIDigitalOut* extend)
  : retractPiston(retract), extendPiston(extend) {
  this->retractPiston->set_value(false);
  this->extendPiston->set_value(false);
};

void LiftArmStateMachine::update() {
  switch (this->state) {
    case UNPOWERED:
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
void LiftArmStateMachine::release() { this->state = UNPOWERED; }

LiftArmStateMachine::STATE LiftArmStateMachine::getState() const {
  return this->state;
}