#include "catapult.h"
#include <cmath>
#include <cstdio>

CatapultStateMachine::CatapultStateMachine(pros::Motor_Group* cataMotors,
                                           pros::ADILineSensor* triballSensor,
                                           pros::Rotation* cataRotation)
  : motors(cataMotors), triballSensor(triballSensor), rotation(cataRotation) {}

bool CatapultStateMachine::fire() {
  if (this->state == EMERGENCY_STOPPED) return false;
  printf("fire\n");
  if (this->state == READY) {
    this->state = FIRING;
    return true;
  }
  // this->constantFiring = true;
  return false;
}

bool CatapultStateMachine::matchload(int millis, int triballs) {
  if (this->state == EMERGENCY_STOPPED) return false;
  printf("matchload\n");
  if (this->state != READY) return false;
  this->timer.set(millis);
  this->triballsLeftToBeFired = triballs;

  this->matchloading = true;
  // this->constantFiring = false;
  return true;
}

void CatapultStateMachine::stop() {
  this->matchloading = false;
  this->timer.set(0);
  this->triballsLeftToBeFired = 0;

  // this->constantFiring = false;
}

void CatapultStateMachine::emergencyStop() {
  this->stop();
  this->state = EMERGENCY_STOPPED;
}

void CatapultStateMachine::cancelEmergencyStop() { this->state = READY; }

bool CatapultStateMachine::isTriballLoaded() const {
  return this->triballSensor->get_value() < 1400;
}

bool CatapultStateMachine::isCataLoadable() const {
  // printf("cata pos: %i\n", this->rotation->get_position());
  return std::remainder(this->rotation->get_position(), 36000) < 0;
}

bool CatapultStateMachine::isCataNotLoadable() const {
  // printf("cata pos: %i\n", this->rotation->get_position());
  return std::remainder(this->rotation->get_position(), 36000) > 300;
}

void CatapultStateMachine::update() {
  if (this->matchloading &&
      (this->timer.isDone() || this->triballsLeftToBeFired == 0)) {
    printf("stop matchloading\n");
    this->matchloading = false;
  }
  const STATE startState = this->state;
  switch (this->state) {
    case READY:
      if (this->isCataNotLoadable()) this->state = RETRACTING;
      // if(this->matchloading)printf("matchloading\n");
      if (this->matchloading && this->isTriballLoaded()) {
        printf("switch to firing\n");
        this->state = FIRING;
      }
      break;
    case FIRING:
      this->retractCataMotor();
      if (this->isCataNotLoadable()) {
        printf("switch to retracting\n");
        this->state = RETRACTING;
        this->indicateTriballFired();
      }
      break;
    case RETRACTING:
      this->retractCataMotor();
      if (this->isCataLoadable()) {
        {
        printf("switch to ready\n");
          this->stopCataMotor();
          this->state = READY;
        }
        break;
        case EMERGENCY_STOPPED: this->stopCataMotor(); break;
      }
  }
  if(startState != this->state) this->update();
}

void CatapultStateMachine::indicateTriballFired() {
  if (this->triballsLeftToBeFired > 0) this->triballsLeftToBeFired--;
  this->triballsFired++;
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

void CatapultStateMachine::waitUntilDoneMatchloading() const {
  while (this->matchloading) pros::delay(10);
}