#include "catapult.h"
#include "lemlib/timer.hpp"
#include "pros/error.h"
#include "robot.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>

CatapultStateMachine::CatapultStateMachine(
    pros::Motor_Group* cataMotors, pros::ADILineSensor* elevationBarSensor,
    pros::Rotation* cataRotation)
  : motors(cataMotors), elevationBarSensor(elevationBarSensor),
    rotation(cataRotation) {}

bool CatapultStateMachine::fire() {
  if (this->state == EMERGENCY_STOPPED) return false;
  printf("fire\n");
  // if (this->state == READY) {
  this->state = FIRING;
  return true;
  // }
  // // this->constantFiring = true;
  // return false;
}

bool CatapultStateMachine::matchload(int millis, int triballs) {
  this->timer.set(millis);
  this->timer.resume();
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

bool CatapultStateMachine::isElevationBarSensed() const {
  const bool disconnected = this->elevationBarSensor->get_value() < 100;
  return !disconnected && this->elevationBarSensor->get_value() < 1000;
}

bool CatapultStateMachine::isCataLoadable() const {
  // printf("cata pos: %i\n", this->rotation->get_position());
  return std::remainder(this->rotation->get_position(), 36000) > -300;
}

bool CatapultStateMachine::isCataNotLoadable() const {
  // printf("cata pos: %i\n", this->rotation->get_position());
  return std::remainder(this->rotation->get_position(), 36000) < -500;
}

bool hasFired = false;
int timeWasted = 0;
bool startReadying = false;
int start = 0;
bool prevCataDisconnected = false;

void CatapultStateMachine::update() {
  static STATE prevState = READY;
  static float prevCataDegrees = float(this->rotation->get_position()) / 100;

  float currCataDegrees = float(this->rotation->get_position()) / 100;

  if (this->matchloading &&
      (this->timer.isDone() || this->triballsLeftToBeFired == 0)) {
    this->matchloading = false;
  }
  const STATE startState = this->state;
  switch (this->state) {
    case READY:
      if (hasFired && !startReadying) start = pros::millis();
      startReadying = true;
      // if (this->isCataNotLoadable()) this->state = RETRACTING;
      if (this->matchloading || this->isElevationBarSensed()) {
        // printf("switch to firing\n");
        this->state = FIRING;
        if (hasFired) timeWasted += pros::millis() - start;
        // printf("triballs fired: %i\n", this->triballsFired);
        // printf("time wasted: %i\n", timeWasted);
      }
      break;
    case FIRING:
      // if we just changed to firing
      if (prevState != FIRING) {
        // get the average temperature of the motors
        std::vector<float> temps = {};
        for (int i = 0; i < this->motors->size(); i++) {
          const float temp = this->motors->get_temperatures()[i];
          // ignore disconnected motors
          if (temp != PROS_ERR) temps.push_back(temp);
        }
        float avgTemp = 0;
        for (float temp : temps) avgTemp += temp;
        avgTemp /= temps.size();

        // make a new test entry
        this->retractionTests.push_back(
            {.config = this->config,
             .data = {},
             .startTime = pros::millis(),
             .batteryPercent = float(pros::battery::get_capacity()),
             .motorTemp = avgTemp});
      }

      hasFired = true;
      startReadying = false;
      this->retractCataMotor();
      if (this->isCataNotLoadable()) {
        // printf("switch to retracting\n");
        this->state = RETRACTING;
        this->indicateTriballFired();
      }
      break;
    case RETRACTING:
      this->retractCataMotor();
      if (this->isCataLoadable()) {
        {
          this->stopCataMotor();
          this->state = READY;
        }
        break;
        case EMERGENCY_STOPPED: this->stopCataMotor(); break;
      }
  }

  if (this->state != EMERGENCY_STOPPED || this->state != READY) {
    float avgWatts = INFINITY;
    float avgVolts = INFINITY;
    int numMotors = 0;
    for (int i = 0; i < this->motors->size(); i++) {
      const float watts = this->motors->at(i).get_power();
      const float volts = this->motors->at(i).get_voltage();
      if (watts != PROS_ERR) {
        if (avgWatts == INFINITY) {
          avgWatts = 0;
          avgVolts = 0;
        }
        avgWatts += watts;
        avgVolts += volts;
        numMotors++;
      }
    }
    avgWatts /= numMotors;
    avgVolts /= numMotors;
    this->retractionTests.back().data.push_back(
        {.velocity = (currCataDegrees - prevCataDegrees) / 0.01F,
         .wattage = avgWatts,
         .voltage = avgVolts});
  }

  prevCataDegrees = currCataDegrees;

  // problem:
  // If motor disconnects, and then reconnects, the motor no longer has 127
  // power.
  // But, we are still sending 127 power to the motor, right?
  // Well no, we are telling the brain to send 127 power to the motor, but the
  // brain doesn't like to listen.
  // The brain attempts to minimize messages sent to the motor by only sending
  // messages when the motor's state has changed.
  // So, if we are constantly telling the brain to send 127 power, but instead
  // of repeatedly sending a 127 power message to the motor, it only sends one
  // message.
  // So this means that the motor when it gets reconnected has no knowledge that
  // it's supposed to be at 127 power.

  // potential solutions:
  // 1. So what if we send a different power to the motor every time (possibly
  //    randomly) to force the brain to send a new message?
  //    Well this does work, but the motor doesn't go at full power and
  //    stutters, even if we go at 100% power for the first nine runs, and go at
  //    99% power for the tenth run. :(
  // 2. Detect disconnect using PROS_ERR and then deliver a different amount to
  //    the motor once connected.
  //    Well there's two problems with this solution:
  //    a)  Sometimes you can get a very short disconnection that does not
  //        register by using the pros methods but still breaks the motor.
  //    b)  The motor will take some seemingly random amount of time to be
  //        capable of receiving power.
  //        So we have to somehow detect when the motor is capable of receiving
  //        power, then send it, then continue normal operation of the catapult.
  // 3. Detect whether motor is moving using current, if not, send a different
  //    power to the motor.
  //    And this seems to work fantastically!:

  // Whether we are detecting the cata is not moving
  static bool notMoving = false;
  // is the cata supposed to be moving?
  bool inMovingState = this->state == RETRACTING || this->state == FIRING;
  // start time of cata supposed to be moving and at 0 current continuously
  static int startZeroCurrent = 0;

  // only let notMoving be true, if inMovingState is true
  notMoving &= inMovingState;
  if (inMovingState) {
    // get the current draw of the motor
    const double curr = this->motors->get_current_draws()[0];

    // if current is PROS_ERR, then the motor is likely disconnected, and thus
    // is likely not moving
    notMoving |= curr == PROS_ERR;

    // if the motor has zero current draw, then
    if (this->motors->get_current_draws()[0] == 0) {
      // if start time has not been set, then set it
      if (startZeroCurrent == 0) startZeroCurrent = pros::millis();
    }
    // if the cata is not supposed to be moving, reset the start time
    // or if the cata the current is not 0, reset the start time
    else
      startZeroCurrent = 0;

    // if zero curent is currently being detected and has been detected for the
    // last 50ms, report that the cata is not moving
    if (startZeroCurrent != 0 && pros::millis() - startZeroCurrent > 50) {
      // printf("cata disconnected, time:%i\n", pros::millis() -
      // startZeroCurrent);
      notMoving = true;
    } else notMoving = false;

    // if the cata is not moving, then we shall randomize voltage in order to
    // prevent the brain from optimizing messages sent to the motor
    if (notMoving) { this->motors->move_voltage(12000 - 120 * (rand() % 3)); }
  }
  prevState = this->state;

  // if state changed, rerun update
  if (startState != this->state) this->update();
}

void CatapultStateMachine::indicateTriballFired() {
  if (this->triballsLeftToBeFired > 0) this->triballsLeftToBeFired--;
  this->triballsFired++;
}

void CatapultStateMachine::retractCataMotor() {
  const auto currentTest = this->retractionTests.back();
  const uint32_t timeSinceStart = pros::millis() - currentTest.startTime;
  const int intervalNum =
      floor(float(timeSinceStart) / currentTest.config.interval);
  this->motors->move_voltage(intervalNum % 2 == 0
                                 ? currentTest.config.milliVoltsA
                                 : currentTest.config.milliVoltsB);
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

void CatapultStateMachine::waitUntilDoneMatchloading() {
  while (this->matchloading && !this->timer.isDone() && getTriballsLeft() != 0)
    pros::delay(10);
}

void CatapultStateMachine::waitUntilDoneFiring() {
  while (this->state != READY) pros::delay(10);
}

void CatapultStateMachine::fireWithConfig(CataConfig config, bool isBlocking) {
  this->waitUntilDoneFiring();
  this->config = config;
  this->fire();
  if (isBlocking) this->waitUntilDoneFiring();
}

void CatapultStateMachine::testManyConfigs(std::vector<CataConfig> configs) {
  for (CataConfig config : configs) { this->fireWithConfig(config, true); }
}