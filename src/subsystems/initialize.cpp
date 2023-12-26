#include "pros/rtos.hpp"
#include "robot.h"

CatapultStateMachine* Robot::Subsystems::catapult = nullptr;
pros::Task* Robot::Subsystems::task = nullptr;

void Robot::Subsystems::initialize() {
  Robot::Subsystems::catapult = new CatapultStateMachine(
      &Robot::Motors::catapult, &Robot::Sensors::cataTriball);

  Robot::Subsystems::task = new pros::Task([]() {
    while (true) {
      Robot::Subsystems::update();
      pros::delay(10);
    }
  });
}

void Robot::Subsystems::update() { Robot::Subsystems::catapult->update(); }