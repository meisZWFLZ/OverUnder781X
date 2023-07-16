#include "main.h"
#include "robot.h"
#include <string>

void screen() {
  // loop forever
  while (true) {
    lemlib::Pose pose =
        Robot::chassis.getPose(); // get the current position of the robot
    pros::lcd::print(0, "x: %f", pose.x); // print the x position
    pros::lcd::print(1, "y: %f", pose.y); // print the y position
    pros::lcd::print(2, "heading: %f",
                     pose.theta); // print the heading
    pros::delay(10);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Calibrating chassis...");
  Robot::chassis.calibrate(); // calibrate the chassis
  pros::lcd::set_text(1, "Chassis Calibrated!");
  pros::Task screenTask(
      screen); // create a task to print the position to the screen
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
void autonomous() {}

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
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 								  Test Friction Code
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  Robot::Motors::leftDrive.tare_position();
  Robot::Motors::leftDrive.tare_position();
  Robot::Motors::leftDrive.move(127);
  Robot::Motors::leftDrive.move(127);

  while (true) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 								     Drive Code
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // drivetrain
    Robot::Motors::leftDrive.move_voltage(
        pow(Robot::control.getAnalog(ControllerAnalog::leftY), 3) * 12000);
    Robot::Motors::rightDrive.move_voltage(
        pow(Robot::control.getAnalog(ControllerAnalog::rightY), 3) * 12000);

    // intake / outtake
    if (Robot::control.getDigital(ControllerDigital::L1))
      Robot::Motors::intake.move(11);
    else if (Robot::control.getDigital(ControllerDigital::L2))
      Robot::Motors::intake.move(-11);
    else Robot::Motors::intake.move(0);

    // shoot / un-shoot?
    if (Robot::control.getDigital(ControllerDigital::R1)) {
      Robot::Motors::shooter.move(11);
      Robot::control.rumble("-");
    } else if (Robot::control.getDigital(ControllerDigital::R2))
      Robot::Motors::shooter.move(-11);
    else Robot::Motors::shooter.move(0);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 							     Driver Feedback
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // drive temp display
    const auto leftTempArr = Robot::Motors::leftDrive.get_temperatures();
    const auto rightTempArr = Robot::Motors::rightDrive.get_temperatures();

    const std::string leftTempStr =
        std::to_string((int)((leftTempArr[0] + leftTempArr[1]) * 0.9) + 32);
    const std::string rightTempStr =
        std::to_string((int)((rightTempArr[0] + rightTempArr[1]) * 0.9) + 32);

    Robot::control.clearLine(0);
    Robot::control.setText(
        0, 0,
        leftTempStr + std::string(4 - leftTempStr.length(), ' ') + "DRIVE " +
            std::string(4 - rightTempStr.length(), ' ') + rightTempStr);

    pros::delay(20);
  }
}
