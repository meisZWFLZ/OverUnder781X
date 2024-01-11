#include "driverFeedback.h"
#include "lift.h"
#include "pros/rtos.hpp"
#include "robot.h"

constexpr int MIN_MILLIS_BETWEEN_UPDATES = 10;

CatapultStateMachine* Robot::Subsystems::catapult = nullptr;
LiftArmStateMachine* Robot::Subsystems::lift = nullptr;
pros::Task* Robot::Subsystems::task = nullptr;

void Robot::Subsystems::initialize() {
  Robot::Subsystems::catapult = new CatapultStateMachine(
      &Robot::Motors::catapult, &Robot::Sensors::cataTriball,
      &Robot::Sensors::cata);
  Robot::Subsystems::lift = new LiftArmStateMachine(&Robot::Motors::elevator);
  Robot::Subsystems::feedback = new DriverFeedback();

  Robot::Subsystems::task = new pros::Task([]() {
    while (true) {
      const int start = pros::millis();
      Robot::Subsystems::update();
      pros::delay(MIN_MILLIS_BETWEEN_UPDATES - (pros::millis() - start));
    }
  });
}

void Robot::Subsystems::update() {
  Robot::Subsystems::feedback->update();
  Robot::Subsystems::catapult->update();
  Robot::Subsystems::lift->update();
}