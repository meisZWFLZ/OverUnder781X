#include "robot.h"

constexpr int MIN_MILLIS_BETWEEN_UPDATES = 10;

CatapultStateMachine* Robot::Subsystems::catapult = nullptr;
LiftArmStateMachine* Robot::Subsystems::lift = nullptr;
DriverFeedback* Robot::Subsystems::feedback = nullptr;
ControllerScreen* Robot::Subsystems::controller = nullptr;
FourWingSubsystem* Robot::Subsystems::wings = nullptr;

pros::Task* Robot::Subsystems::task = nullptr;

void Robot::Subsystems::initialize() {
  Robot::Subsystems::catapult = new CatapultStateMachine(
      &Robot::Motors::catapult, &Robot::Sensors::cataElevationBar,
      &Robot::Sensors::cata);
  // we no longer have a motorized lift, but just to fool driver code
  Robot::Subsystems::lift = new LiftArmStateMachine(
      &Robot::Pistons::retractLift, &Robot::Pistons::extendLift);
  // Robot::Subsystems::feedback = new DriverFeedback();
  Robot::Subsystems::controller = new ControllerScreen(&Robot::control);
  Robot::Subsystems::wings = FourWingSubsystem::makeFromPortConfig(
                                 Robot::Pistons::wingConfig,
                                 Robot::Tunables::driverWingJoystickThreshold);
  
  printf("wings front size: %d\n", Robot::Subsystems::wings->front->size());
  printf("wings back size: %d\n", Robot::Subsystems::wings->back->size());

  Robot::Subsystems::task = new pros::Task([]() {
    while (true) {
      const int start = pros::millis();
      Robot::Subsystems::update();
      // const int a = pros::millis();
      // printf("update took %i ms\n", a - start);
      pros::delay(10);
      // pros::delay(MIN_MILLIS_BETWEEN_UPDATES - (a - start));
    }
  });
}

void Robot::Subsystems::update() {
  // printf("updating feedback: %i\n", Robot::Subsystems::feedback == nullptr);
  // Robot::Subsystems::feedback->update();
  // printf("updating catapult\n");
  Robot::Subsystems::catapult->update();
  // printf("updating lift\n");
  // we no longer have a motorized lift
  Robot::Subsystems::lift->update();
  // printf("updating controller\n");
  // Robot::Subsystems::controller->update();
}