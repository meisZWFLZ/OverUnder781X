#include "main.h"
#include "robot.h"
#include "auton.h"
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include "neil_pid.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string.h>

float average(std::vector<float> const& v) {
  if (v.empty()) { return 0; }

  auto const count = static_cast<float>(v.size());
  return std::reduce(v.begin(), v.end()) / count;
}

const float get_wattage(pros::Motor_Group& group) {
  std::vector<float> watts = {};
  for (int i = 0; i < group.size(); i++) watts.push_back(group[i].get_power());
  return average(watts);
}

void screen() {
  // loop forever
  pros::Controller controller(pros::controller_id_e_t::E_CONTROLLER_MASTER);
  while (true) {
    lemlib::Pose pose =
        Robot::chassis->getPose(); // get the current position of the robot
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "x: %f in", pose.x); // print the x position
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "y: %f in", pose.y); // print the y position
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "heading: %f deg",
                     pose.theta); // print the heading
    if (!controller.get_digital(
            pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP)) {
    } else {
      // display wattage
      // controller.clear_line(0);
      printf("in: %fw\n", get_wattage(Robot::Motors::intake));
      // char intake[15];
      // sprintf(intake, "in: %fw",
      //         get_wattage(Robot::Motors::intake)); // print the x position
      // controller.set_text(0, 0, intake);
      // // controller.clear_line(1);
      char shooter[15];
      sprintf(shooter, "fly: %fw",
              get_wattage(Robot::Motors::shooter)); // print the x position
      controller.set_text(0, 0, shooter);
      // char drive[15];
      // sprintf(drive, "dri: %fw",
      //         (get_wattage(Robot::Motors::leftDrive) +
      //          get_wattage(Robot::Motors::rightDrive)) /
      //             2); // print the x position
      // controller.set_text(2, 0, drive); // print the heading
    }
    pros::delay(200);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  Robot::initializeOdometryConfig();
  printf("get_ang:%i, horiPresent:%i\n", Robot::Sensors::hori.get_angle(),
         Robot::Sensors::hori.get_angle() != PROS_ERR);
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Calibrating chassis...");
  printf("<sensors = nullptr>\n");
  if (Robot::odomSensors->horizontal1 == nullptr) printf("  hori1,\n");
  if (Robot::odomSensors->horizontal2 == nullptr) printf("  hori2,\n");
  if (Robot::odomSensors->imu == nullptr) printf("  imu,\n");
  if (Robot::odomSensors->vertical1 == nullptr) printf("  vert1,\n");
  if (Robot::odomSensors->vertical2 == nullptr) printf("  vert2,\n");
  printf("<sensors = nullptr/>\n");
  printf("<error>\n");
  switch (errno) {
    case PROS_ERR: printf("  PROS_ERR\n"); break;
    case ENODEV: printf("  ENODEV\n"); break;
    case ENXIO: printf("  ENXIO\n"); break;
  }
  errno = 0;
  printf("<error/>\n");
  Robot::chassis->calibrate(); // calibrate the chassis
  pros::lcd::set_text(1, "Chassis Calibrated!");
  Robot::chassis->setPose(0, 0, 0);
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
void autonomous() {
  printf("auton start");
  // score alliance triball
  // lemlib::Logger::initialize();
  // Robot::chassis->setPose(41.5, -65.125, 0);
  // Robot::chassis->moveTo(60, -45, 0, 5000, false, true, 0, 0.6, 127, true);

  // shoot le ball 
  // Robot::chassis->setPose(-24, -24, 0);
  // auton::actions::shootTriballIntoOffensiveZone();


  Robot::chassis->setPose((-72+Robot::Dimensions::drivetrainWidth/2 + 24),
  -72+Robot::Dimensions::drivetrainLength/2, 0);
  auton::actions::touchElevationBar();
  // auton::actions::scoreAllianceTriball();

  // move forward one tile
  // Robot::chassis.moveTo(0, 24, 5000, 200);

  // // move right one tile
  // Robot::chassis.moveTo(-12, 0, 5000);

  // // move right and forward one tile
  // Robot::chassis->moveTo(0, 24, 0,5000);

  // turn 90 deg
  // Robot::chassis.turnTo(1000, 0, 5000);
  // // turn around
  // Robot::chassis.turnTo(0, -24, 5000);

  // // move in square
  // Robot::chassis.moveTo(0, 24, 500);
  // pros::delay(1000);
  // Robot::chassis.moveTo(24, 24, 500);
  // pros::delay(1000);
  // Robot::chassis.moveTo(24, 0, 500);
  // pros::delay(1000);
  // Robot::chassis.moveTo(0, 0, 500);
  // pros::delay(1000);
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
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 								  Test Friction Code
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // std::string myStr = "";
  // std::cout << "What's your name? ";
  // getline(std::cin, myStr);
  // std::cout << "Hello " << myStr << std::endl;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 								  Test Terminal input
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // FILE* serialIn = fopen("sin", "w");

  // std::string input = "";
  // while (1) {
  //   std::cout << "ECHO > " << std::endl;
  //   std::getline(std::cin, input);
  //   std::cout << std::endl;
  //   std::cout << input << std::endl;
  // }
  // pros::c::fdctl(serialIn->_file, SERCTL_ACTIVATE, NULL);
  // while (true) {
  //   std::string input;
  //   std::cout << "ECHO > " << std::endl;
  //   std::getline(std::cin, input);
  //   std::cout << std::endl;
  //   std::cout << input << std::endl;
  // }
  // Robot::Motors::leftDrive.tare_position();
  // Robot::Motors::leftDrive.tare_position();
  // Robot::Motors::leftDrive.move(127);
  // Robot::Motors::leftDrive.move(127);
  /**
   * false = retracted
   *
   * true = expanded
   */
  bool intakeElevatorState = false;
  bool wasXPressed = false;
  bool wingsState = false;
  bool wasUpPressed = false;
  // int shooterState = 0;
  // bool wasShootPressed = false;
  while (true) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 								     Drive Code
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // drivetrain
    // Robot::Motors::leftDrive.move_voltage(
    //     pow(Robot::control.getAnalog(ControllerAnalog::leftY), 3) * 12000);
    // Robot::Motors::rightDrive.move_voltage(
    //     pow(Robot::control.getAnalog(ControllerAnalog::rightY), 3) * 12000);
    // Robot::chassis.curvature(Robot::control.getAnalog(ControllerAnalog::leftY)*127, Robot::control.getAnalog(ControllerAnalog::rightX)*127,20);
    Robot::chassis->tank(Robot::control.getAnalog(ControllerAnalog::leftY)*127, Robot::control.getAnalog(ControllerAnalog::rightY)*127,15);


    // intake / outtake
    if (Robot::control.getDigital(ControllerDigital::L1))
      Robot::Motors::intake.move(127);
    else if (Robot::control.getDigital(ControllerDigital::L2))
      Robot::Motors::intake.move(-127);
    else Robot::Motors::intake.move(0);

    // // shoot / un-shoot?
    if (Robot::control.getDigital(ControllerDigital::R1)) {
      Robot::Motors::shooter.move(127);
      Robot::control.rumble("-");
    } else if (Robot::control.getDigital(ControllerDigital::R2))
      Robot::Motors::shooter.move(-127);
    else Robot::Motors::shooter.move(0);
    // if (Robot::control.getDigital(ControllerDigital::R1)) {
    //   if(!wasShootPressed) {
    //     Robot::Motors::shooter.move(shooterState = shooterState != 0 ? 0 : 127);
    //     Robot::control.rumble("-");
    //   }
    //   wasShootPressed = true;
    // } else if (Robot::control.getDigital(ControllerDigital::R2)) {
    //   if(!wasShootPressed) {
    //     Robot::Motors::shooter.move(shooterState = shooterState != 0 ? 0 : -127);
    //     Robot::control.rumble("-");
    //   }
    //   wasShootPressed = true;
    // } 
    // else wasShootPressed = false;

    // elevate intake
    if (Robot::control.getDigital(ControllerDigital::X)) {
      if (!wasXPressed)
        Robot::Pistons::intakeElevator.set_value(intakeElevatorState ^= true);
      wasXPressed = true;
    } else wasXPressed = false;

    if (Robot::control.getDigital(ControllerDigital::X)) {
      if (!wasUpPressed)
        Robot::Pistons::wings.set_value(wingsState ^= true);
      wasUpPressed = true;
    } else wasUpPressed = false;

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
