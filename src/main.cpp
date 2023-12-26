#include "main.h"
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
  Robot::initializeOdometryConfig();

  pros::lcd::initialize();
  pros::lcd::set_text(1, "Calibrating chassis...");

  Robot::chassis->calibrate(); // calibrate the chassis
  pros::lcd::set_text(1, "Chassis Calibrated!");
  Robot::chassis->setPose(0, 0, 0);
  Robot::Actions::raiseIntake();

  addAutons();
  auton::AutonSelector::init();
  auton::AutonSelector::enable();
  new pros::Task {screen}; // create a task to print the position to the screen
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

void intakeAndShoot() {
  using namespace fieldDimensions;

  auton::actions::intakeTriball(
      {-TILE_LENGTH - 2, 0 - Robot::Dimensions::drivetrainLength / 2 - 0.85,
       UP});
  auton::actions::shootTriballIntoOffensiveZone();
}

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

  // if (pros::competition::is_connected() && !autonHasRun)
  //   Robot::Actions::prepareIntake();

  bool skills = false;
  if (!std::strcmp(auton::AutonSelector::getCurrentAuton(), (char*)("skills")))
    skills = true;

  if (skills) {
    Robot::chassis->setPose(fieldDimensions::leftStartingPose, false);

    auton::actions::prepareForMatchloading();
  }
  /**
   * false = down
   *
   * true = up
   */
  bool intakeElevatorState = true;
  bool wasXPressed = false;
  /**
   * false = retracted
   *
   * true = expanded
   */
  bool wingsState = false;
  bool wasUpPressed = false;

  bool prevR1 = Robot::control.getDigital(ControllerDigital::R1);
  bool prevR2 = Robot::control.getDigital(ControllerDigital::R2);
  int shooterState = 0;
  while (true) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 								     Drive Code
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // drivetrain
    Robot::chassis->tank(
        Robot::control.getAnalog(ControllerAnalog::leftY) * 127,
        Robot::control.getAnalog(ControllerAnalog::rightY) * 127, 15);

    // intake / outtake
    if (Robot::control.getDigital(ControllerDigital::L1))
      Robot::Motors::intake.move(127);
    else if (Robot::control.getDigital(ControllerDigital::L2))
      Robot::Motors::intake.move(-127);
    else Robot::Motors::intake.move(0);

    // // shoot / un-shoot? / shootMacro
    const bool r1 = Robot::control.getDigital(ControllerDigital::R1);
    const bool r2 = Robot::control.getDigital(ControllerDigital::R2);

    if (skills) {
      if (r1 && prevR1 == false) shooterState = shooterState == 0 ? 1 : 0;
      else if (r2 && prevR2 == false) shooterState = shooterState == 0 ? -1 : 0;
      switch (shooterState) {
        case 1: Robot::Actions::shoot(); break;
        // case 0: Robot::Actions::stopShooter(); break;
        // case -1: Robot::Actions::unshoot(); break;
      }
      prevR1 = r1;
      prevR2 = r2;
    } else {
      if (r1) Robot::Actions::shoot();
      // else if (r2) Robot::Actions::unshoot();
      // else Robot::Actions::stopShooter();
    }

    // elevate intake
    if (Robot::control.getDigital(ControllerDigital::X)) {
      if (!wasXPressed) {
        intakeElevatorState = !intakeElevatorState;
        if (intakeElevatorState) Robot::Actions::raiseIntake();
        else Robot::Actions::lowerIntake();
      }
      wasXPressed = true;
    } else wasXPressed = false;

    if (Robot::control.getDigital(ControllerDigital::up)) {
      if (!wasUpPressed) Robot::Pistons::wings.set_value(wingsState ^= true);
      wasUpPressed = true;
    } else wasUpPressed = false;

    pros::delay(20);
  }
}
