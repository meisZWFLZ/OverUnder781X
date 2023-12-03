#include "main.h"
#include "pros/misc.hpp"
#include "robot.h"
#include "auton.h"
#include <numeric>
#include <string>
#include <vector>
#include "neil_pid.h"
#include <vector>
#include "fieldDimensions.h"
#include "selector.h"

bool autonHasRun = false;

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
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "x: %f in", pose.x); // print the x position
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "y: %f in", pose.y); // print the y position
    pros::lcd::clear_line(3);
    pros::lcd::print(3, "heading: %f deg",
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
      // char shooter[15];
      // sprintf(shooter, "fly: %fw",
      //         get_wattage(Robot::Motors::shooter)); // print the x position
      // controller.set_text(0, 0, shooter);
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
  // printf("get_ang:%i, horiPresent:%i\n", Robot::Sensors::hori.get_angle(),
  //        Robot::Sensors::hori.get_angle() != PROS_ERR);
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Calibrating chassis...");
  // printf("<sensors = nullptr>\n");
  // if (Robot::odomSensors->horizontal1 == nullptr) printf("  hori1,\n");
  // if (Robot::odomSensors->horizontal2 == nullptr) printf("  hori2,\n");
  // if (Robot::odomSensors->imu == nullptr) printf("  imu,\n");
  // if (Robot::odomSensors->vertical1 == nullptr) printf("  vert1,\n");
  // if (Robot::odomSensors->vertical2 == nullptr) printf("  vert2,\n");
  // printf("<sensors = nullptr/>\n");
  // printf("<error>\n");
  // switch (errno) {
  //   case PROS_ERR: printf("  PROS_ERR\n"); break;
  //   case ENODEV: printf("  ENODEV\n"); break;
  //   case ENXIO: printf("  ENXIO\n"); break;
  // }
  // errno = 0;
  // printf("<error/>\n");
  Robot::chassis->calibrate(); // calibrate the chassis
  pros::lcd::set_text(1, "Chassis Calibrated!");
  Robot::chassis->setPose(0, 0, 0);
  Robot::Actions::raiseIntake();

  addAutons();
  auton::AutonSelector::init();
  auton::AutonSelector::enable();
  screenTask = new pros::Task {
      screen}; // create a task to print the position to the screen
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

void score4BallAuto() {
  using namespace fieldDimensions;
  // pick up and shoot first triball
  Robot::chassis->moveTo(0 + TILE_RADIUS, MIN_Y + (TILE_LENGTH * 2), LEFT - 15,
                         3000);
  Robot::Actions::intake();
  Robot::Actions::shoot();
  Robot::chassis->tank(96, 96);
  pros::delay(500);
  Robot::Actions::stopIntake();
  Robot::Actions::stopShooter();

  // pick up second triball
  Robot::chassis->moveTo(0 + TILE_RADIUS, MIN_Y + (TILE_LENGTH * 3), LEFT,
                         3000);
  Robot::Actions::intake();
  Robot::chassis->tank(96, 96);
  pros::delay(500);
  Robot::Actions::stopIntake();
  Robot::chassis->turnTo(0, 10000, 3000);

  // Robot::chassis->moveTo(0 + Robot::Dimensions::drivetrainWidth/2 - 1, MIN_Y
  // + (TILE_LENGTH * 2.5), UP,
  //                        3000);

  // Plow triballs into goal
  Robot::Actions::expandWings();
  Robot::chassis->turnTo(10000, 0, 3000);
  Robot::Actions::outtake();
  pros::delay(150);
  Robot::chassis->tank(127, 127);
  pros::delay(800);
  Robot::Actions::retractWings();
  Robot::Actions::stopIntake();
  Robot::chassis->tank(-64, -60);
  pros::delay(175);
}

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
  if (pros::competition::is_connected()) Robot::Actions::prepareIntake();
  printf("auton start");
  pros::competition::is_connected();

  auton::AutonSelector::runAuton();
  // score alliance triball
  // lemlib::Logger::initialize();
  // Robot::chassis->setPose(41.5, -65.125, 0);
  // Robot::chassis->moveTo(60, -45, 0, 5000, false, true, 0, 0.6, 127, true);

  // shoot le ball
  // Robot::chassis->setPose(-24, -24, 0);
  // auton::actions::shootTriballIntoOffensiveZone();

  // Robot::chassis->setPose((-72+Robot::Dimensions::drivetrainWidth/2 + 24),
  // -72+Robot::Dimensions::drivetrainLength/2, 0);
  // auton::actions::touchElevationBar();
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
  auton::AutonSelector::disable();
  Robot::Actions::retractWings();

  if (pros::competition::is_connected() && !autonHasRun)
    Robot::Actions::prepareIntake();

  // using namespace fieldDimensions;
  // Robot::chassis->setPose(
  //     (MIN_X + TILE_LENGTH + Robot::Dimensions::drivetrainWidth / 2),
  //     (MIN_Y + Robot::Dimensions::drivetrainLength / 2), UP);

  // if (!std::strcmp(auton::AutonSelector::getCurrentAuton(),
  // (char*)("skills")))
  //   auton::actions::prepareForMatchloading();

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

  // disable drive
  // bool macroRunning = false;
  // pros::Task* macroTask = nullptr;
  // bool prevR1 = Robot::control.getDigital(ControllerDigital::R1);
  // bool prevR2 = Robot::control.getDigital(ControllerDigital::R2);
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
    // Robot::chassis.curvature(Robot::control.getAnalog(ControllerAnalog::leftY)*127,
    // Robot::control.getAnalog(ControllerAnalog::rightX)*127,20);
    // if (macroTask == nullptr) {
    //   macroRunning = false;
    //   printf("macro null\n");
    // }
    // if (macroTask != nullptr) printf("state:%i \n", macroTask->get_state());
    // if ((macroRunning &&
    //      (std::abs(Robot::control.getAnalog(ControllerAnalog::leftY)) > 0.1
    //      ||
    //       std::abs(Robot::control.getAnalog(ControllerAnalog::rightY)) >
    //           0.1)) ||
    //     (macroTask != nullptr &&
    //      macroTask->get_state() == pros::E_TASK_STATE_SUSPENDED)) {
    //   macroRunning = false;
    //   macroTask->suspend();
    //   macroTask->remove();
    //   // delete &macroTask;
    //   macroTask = nullptr;
    // }
    // if (!macroRunning)
    Robot::chassis->tank(
        Robot::control.getAnalog(ControllerAnalog::leftY) * 127,
        Robot::control.getAnalog(ControllerAnalog::rightY) * 127, 15);

    // intake / outtake
    // if (!macroRunning) {
    if (Robot::control.getDigital(ControllerDigital::L1))
      Robot::Motors::intake.move(127);
    else if (Robot::control.getDigital(ControllerDigital::L2))
      Robot::Motors::intake.move(-127);
    else Robot::Motors::intake.move(0);
    // }

    // // shoot / un-shoot? / shootMacro
    const bool r1 = Robot::control.getDigital(ControllerDigital::R1);
    const bool r2 = Robot::control.getDigital(ControllerDigital::R2);
    // printf("run shooter macro");
    // if (!(prevR1 && prevR2) && r1 && r2) {
    //   printf("run shooter macro\n");
    //   // pros::Task::delay(1000);
    //   if (!macroRunning && macroTask == nullptr) {
    //     printf("start shooter macro\n");
    //     macroRunning = true;
    //     macroTask =
    //         new pros::Task(Robot::Actions::shootMacro, TASK_PRIORITY_DEFAULT,
    //                        TASK_STACK_DEPTH_DEFAULT, "shooterMacro");
    //   } else if (macroRunning && macroTask != nullptr) {
    //     printf("stop shooter macro\n");
    //     macroRunning = false;
    //     macroTask->suspend();
    //     macroTask->remove();
    //     // delete &macroTask;
    //     macroTask = nullptr;
    //   }
    // }
    // if (!macroRunning && !(r1 && r2)) {
    if (r1) Robot::Actions::shoot();
    else if (r2) Robot::Actions::unshoot();
    else Robot::Actions::stopShooter();
    // }
    // prevR1 = r1;
    // prevR2 = r2;
    // if (Robot::control.getDigital(ControllerDigital::R1)) {
    //   if(!wasShootPressed) {
    //     Robot::Motors::shooter.move(shooterState = shooterState != 0 ? 0 :
    //     127); Robot::control.rumble("-");
    //   }
    //   wasShootPressed = true;
    // } else if (Robot::control.getDigital(ControllerDigital::R2)) {
    //   if(!wasShootPressed) {
    //     Robot::Motors::shooter.move(shooterState = shooterState != 0 ? 0 :
    //     -127); Robot::control.rumble("-");
    //   }
    //   wasShootPressed = true;
    // }
    // else wasShootPressed = false;

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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 							     Driver Feedback
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // // drive temp display
    // const auto leftTempArr = Robot::Motors::leftDrive.get_temperatures();
    // const auto rightTempArr = Robot::Motors::rightDrive.get_temperatures();

    // const std::string leftTempStr =
    //     std::to_string((int)((leftTempArr[0] + leftTempArr[1]) * 0.9) + 32);
    // const std::string rightTempStr =
    //     std::to_string((int)((rightTempArr[0] + rightTempArr[1]) * 0.9) +
    //     32);

    // Robot::control.clearLine(0);
    // Robot::control.setText(
    //     0, 0,
    //     leftTempStr + std::string(4 - leftTempStr.length(), ' ') + "DRIVE " +
    //         std::string(4 - rightTempStr.length(), ' ') + rightTempStr);

    pros::delay(20);
  }
}
