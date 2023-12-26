#include "catapult.h"

CatapultStateMachine::CatapultStateMachine(pros::Motor_Group* cataMotors,
                                           pros::ADILineSensor* triballSensor)
  : motors(cataMotors), sensor(triballSensor) {}

void CatapultStateMachine::fire() { this->state = FIRING; }

void CatapultStateMachine::matchload(int millis, int triballs) {
  this->timer.set(millis);
  this->triballsLeftToBeFired = triballs;

  this->matchloading = true;
  this->fire();
}

void CatapultStateMachine::stopMatchloading() {
  this->matchloading = false;
  this->timer.set(0);
  this->triballsLeftToBeFired = 0;
}

bool CatapultStateMachine::isTriballLoaded() const {
  return this->sensor->get_value() > 2000;
}

void CatapultStateMachine::update() {
  switch (this->state) {
    case IDLE: break;
    case LOADING: break;
    case FIRING: break;
    case RETRACTING: break;
  }
}

int CatapultStateMachine::getTriballsLeft() const {
  return this->triballsLeftToBeFired;
}

int CatapultStateMachine::getTriballsFired() const {
  return this->triballsFired;
}

bool CatapultStateMachine::getIsMatchloading() const {
  return this->matchloading;
}

CatapultStateMachine::STATE CatapultStateMachine::getState() const {
  return this->state;
}
