#include "main.h"
#include "catapult.h"
#include "pros/misc.h"
#include "robot.h"
#include "auton.h"
#include <string>
#include "neil_pid.h"
#include "fieldDimensions.h"
#include "selector.h"

bool autonHasRun = false;

void screen() {
  // loop forever
  while (true) {
    lemlib::Pose pose =
        Robot::chassis->getPose(); // get the current position of the robot
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "x: %f in", pose.x); // print the x position
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "y: %f in", pose.y); // print the y position
    pros::lcd::clear_line(3);
    pros::lcd::print(3, "heading: %f deg",
                     pose.theta); // print the heading
    pros::delay(200);
  }
}

pros::Task* screenTask;

void addAutons() {
  auton::AutonSelector::addAuton(&auton::autons::defensive);
  auton::AutonSelector::addAuton(&auton::autons::offensive);
  auton::AutonSelector::addAuton(&auton::autons::skills);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Robot::initializeOdometryConfig();
  Robot::Subsystems::initialize();
  // pros::lcd::initialize();
  // pros::lcd::set_text(1, "Calibrating chassis...");

  // Robot::chassis->calibrate(); // calibrate the chassis
  // pros::lcd::set_text(1, "Chassis Calibrated!");
  // Robot::chassis->setPose(0, 0, 0);
  // Robot::Actions::raiseIntake();

  // addAutons();
  // auton::AutonSelector::init();
  // auton::AutonSelector::enable();
  // new pros::Task {screen}; // create a task to print the position to the
  // screen
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  auton::AutonSelector::disable();
  autonHasRun = true;
  // if (pros::competition::is_connected()) Robot::Actions::prepareIntake();
  printf("auton start");
  pros::competition::is_connected();

  auton::AutonSelector::runAuton();
}

// class OnRisingEdgeListener {
//   private:
//     void (*callback)();
//     bool prevButton = false;
//     const pros::controller_digital_e_t button;
//     const pros::controller_id_e_t controllerId;

//     void update() {
//       if (button && !prevButton) callback();
//       prevButton = button;
//     }

//     static std::vector<OnRisingEdgeListener*> listeners;
//   public:
//     OnRisingEdgeListener(
//         const pros::controller_digital_e_t button, void (*callback)(),
//         const pros::controller_id_e_t controllerId =
//         pros::E_CONTROLLER_MASTER)
//       : callback(callback), button(button), controllerId(controllerId) {
//       OnRisingEdgeListener::listeners.push_back(this);
//     }

//     static void updateAll() {
//       for (OnRisingEdgeListener* listener : listeners) listener->update();
//     }
// };

// std::vector<OnRisingEdgeListener*> OnRisingEdgeListener::listeners {};

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  auton::AutonSelector::disable();
  Robot::Actions::retractWings();
  Robot::Subsystems::catapult->matchload();

  // if (pros::competition::is_connected() && !autonHasRun)
  //   Robot::Actions::prepareIntake();

  // bool skills = false;
  // if (!std::strcmp(auton::AutonSelector::getCurrentAuton(),
  // (char*)("skills")))
  //   skills = true;

  // if (skills) {
  //   Robot::chassis->setPose(fieldDimensions::leftStartingPose, false);

  //   auton::actions::prepareForMatchloading();
  // }

  /**
   * false = retracted
   *
   * true = expanded
   */
  bool wingsState = false;
  bool wasUpPressed = false;

  bool prevX = false;

  bool prevEStopCombo = false;
  while (true) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 								     Drive Code
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // drivetrain
    // Robot::chassis->tank(
    //     Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
    //     Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

    // intake / outtake
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      Robot::Motors::intake.move(127);
    else if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
      Robot::Motors::intake.move(-127);
    else Robot::Motors::intake.move(0);

    // matchload toggle
    const bool buttonX =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (buttonX && !prevX) {
      if (Robot::Subsystems::catapult->getIsMatchloading())
        Robot::Subsystems::catapult->stop();
      else Robot::Subsystems::catapult->matchload();
    }
    prevX = buttonX;

    // catapult manual fire
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      Robot::Subsystems::catapult->fire();

    // catapult emergency stop
    const bool eStopCombo =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (eStopCombo && !prevEStopCombo) {
      if (Robot::Subsystems::catapult->getState() ==
          CatapultStateMachine::STATE::EMERGENCY_STOPPED)
        Robot::Subsystems::catapult->cancelEmergencyStop();
      else Robot::Subsystems::catapult->emergencyStop();
    }
    prevEStopCombo = eStopCombo;

    // wing toggle
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      if (!wasUpPressed) Robot::Pistons::wings.set_value(wingsState ^= true);
      wasUpPressed = true;
    } else wasUpPressed = false;

    pros::delay(20);
  }
}
