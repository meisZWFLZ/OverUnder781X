#include "catapult.h"
#include <cmath>
#include <cstdio>

CatapultStateMachine::CatapultStateMachine(pros::Motor_Group* cataMotors,
                                           pros::ADILineSensor* triballSensor,
                                           pros::Rotation* cataRotation)
  : motors(cataMotors), triballSensor(triballSensor), rotation(cataRotation) {}

bool CatapultStateMachine::fire() {
  printf("fire\n");
  if (this->state == IDLE || this->state == LOADING) {
    this->state = FIRING;
    return true;
  }
  return false;
}

bool CatapultStateMachine::matchload(int millis, int triballs) {
  printf("matchload\n");
  if (this->state != IDLE) return false;
  this->timer.set(millis);
  this->triballsLeftToBeFired = triballs;

  this->matchloading = true;

  this->state = LOADING;
}

void CatapultStateMachine::stop() {
  this->matchloading = false;
  this->timer.set(0);
  this->triballsLeftToBeFired = 0;
  if (LOADING) this->state = IDLE;
}

bool CatapultStateMachine::isTriballLoaded() const {
  printf("triball: %i\n", this->triballSensor->get_value());
  return this->triballSensor->get_value() < 1250;
}

bool CatapultStateMachine::isCataLoadable() const {
  printf("cata pos: %i\n", this->rotation->get_position());
  return std::remainder(this->rotation->get_position(), 36000) < 0;
}

void CatapultStateMachine::update() {
  if (this->matchloading &&
      (this->timer.isDone() || this->triballsLeftToBeFired == 0)) {
    this->matchloading = false;
  }

  switch (this->state) {
    case IDLE: break;
    case LOADING:
      if (this->isTriballLoaded()) {
        printf("switch to firing\n");
        this->state = FIRING;
      }
      break;
    case FIRING:
      this->retractCataMotor();
      if (!this->isCataLoadable()) {
        printf("switch to retracting\n");
        this->state = RETRACTING;
      }
      break;
    case RETRACTING:
      this->retractCataMotor();
      printf("cata pos: %i\n", this->rotation->get_position());
      if (this->isCataLoadable()) {
        this->stopCataMotor();
        if (this->matchloading) {
          printf("switch to loading\n");
          this->state = LOADING;
        } else {
          printf("switch to idle\n");
          this->state = IDLE;
        }
      }
      break;
  }
}

void CatapultStateMachine::retractCataMotor() {
  this->motors->move_voltage(12000);
}

void CatapultStateMachine::stopCataMotor() { this->motors->move_voltage(0); }

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
