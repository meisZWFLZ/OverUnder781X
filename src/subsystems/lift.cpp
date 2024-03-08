#include "lift.h"

// construct the lift arm state machine
LiftArmStateMachine::LiftArmStateMachine(pros::ADIDigitalOut* retract,
                                         pros::ADIDigitalOut* extend)
  : retractPiston(retract), extendPiston(extend) {
  this->retractPiston->set_value(false);
  this->extendPiston->set_value(false);
};

// run every 10 ms
void LiftArmStateMachine::update() {
  switch (this->state) {
    // when in the idle state:
    case IDLE:
      // set both to false (ie: close the valves)
      this->retractPiston->set_value(false);
      this->extendPiston->set_value(false);
      break;

    // when in the retracting state:
    case RETRACTING:
      // only set retractPiston to true (ie: only open the retract valve)
      this->retractPiston->set_value(true);
      this->extendPiston->set_value(false);
      break;

    // when in the extended state:
    case EXTENDING:
      // only set extendPiston to true (ie: only open the expand valve)
      this->retractPiston->set_value(false);
      this->extendPiston->set_value(true);
      break;
  }
}

// extend the pistons
void LiftArmStateMachine::extend() { this->state = EXTENDING; }

// retract the pistons
void LiftArmStateMachine::retract() { this->state = RETRACTING; }

// stop sending air to the pistons
void LiftArmStateMachine::release() { this->state = IDLE; }

// get the current state of the lift arm state machine
LiftArmStateMachine::STATE LiftArmStateMachine::getState() const {
  return this->state;
}