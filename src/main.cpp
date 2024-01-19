#include "main.h"
#include "catapult.h"
#include "lift.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "auton.h"
#include <string>
#include "neil_pid.h"
#include "fieldDimensions.h"
#include "selector.h"

bool autonHasRun = false;

void screen() {
  // loop forever
  Robot::control.clear_line(0);
  Robot::control.clear_line(1);
  Robot::control.clear_line(2);

  float lastHeading = 0;
  while (true) {
    // pros::lcd::print(0, "kP: %f", Robot::Subsystems::lift->getKP());
    // pros::lcd::print(1, "kD: %f", Robot::Subsystems::lift->getKD());
    // pros::lcd::print(2, "stopped: %i",
    //                  Robot::Subsystems::lift->getState() ==
    //                      LiftArmStateMachine::STATE::STOPPED);
    // auto angs = Robot::Motors::elevator.get_positions();
    // pros::lcd::print(3, "lift: %4.2f,%4.2f", angs[0], angs[1]);
    // pros::lcd::print(4, "target: %4.2f",
    // Robot::Subsystems::lift->getTarget());
    pros::lcd::print(4, "target: %4.2f", Robot::Subsystems::lift->getTarget());
    pros::lcd::print(5, "current: %i, %i",
                     Robot::Motors::elevator.at(0).get_current_draw(),
                     Robot::Motors::elevator.at(1).get_current_draw());
    auto currLim = Robot::Motors::elevator.are_over_current();
    pros::lcd::print(6, "curr lim: %i,%i", currLim[0], currLim[1]);
    lemlib::Pose pose =
        Robot::chassis->getPose(); // get the current position of the

    pros::lcd::clear_line(1);
    pros::lcd::print(1, "x: %f in", pose.x); // print the x position
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "y: %f in", pose.y); // print the y position
    pros::lcd::clear_line(3);
    pros::lcd::print(3, "heading: %f deg",
                     pose.theta); // print the heading

    printf("theta: %f\n", pose.theta);

    Robot::chassis->setPose(
        Robot::chassis->getPose().x, Robot::chassis->getPose().y,
        lastHeading +
            (Robot::chassis->getPose().theta - lastHeading) * 360 / 355);
    lastHeading = Robot::chassis->getPose().theta;
    pros::delay(20);
  }
}

pros::Task* screenTask;

void addAutons() {
  auton::AutonSelector::addAuton(&auton::autons::sixRush);
  auton::AutonSelector::addAuton(&auton::autons::sixBall);
  auton::AutonSelector::addAuton(&auton::autons::defensive);
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
  Robot::chassis->calibrate(); // calibrate the chassis

  pros::lcd::set_text(1, "Calibrating chassis...");
  Robot::Subsystems::initialize();
  pros::lcd::set_text(1, "Chassis Calibrated!");
  // Robot::chassis->setPose(0, 0, 0);
  // Robot::Actions::raiseIntake();

  addAutons();
  auton::AutonSelector::init();
  auton::AutonSelector::enable();
  // create a task to print the position to the screen
  new pros::Task {screen};
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

void printPose() {
  lemlib::Pose pose = Robot::chassis->getPose();
  printf("x: %f in\ny: %f in\nheading: %f deg\n", pose.x, pose.y, pose.theta);
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
  if (pros::competition::is_connected()) Robot::Actions::prepareRobot();
  // printf("auton start");

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
  Robot::Subsystems::lift->tareAngle();

  // int start = pros::millis();
  // while (Robot::Subsystems::catapult->getTriballsFired() < 46) {
  //   Robot::Subsystems::catapult->fire();
  //   pros::delay(1000);
  // }
  // printf("time: %i\n", pros::millis() - start);

  if (pros::competition::is_connected() && !autonHasRun)
    Robot::Actions::prepareRobot();

  // bool skills = false;
  // if (!std::strcmp(auton::AutonSelector::getCurrentAuton(),
  // (char*)("skills")))
  //   skills = true;

  // if (skills) {
  //   Robot::chassis->setPose(fieldDimensions::leftStartingPose, false);

  //   auton::actions::prepareForMatchloading();
  // }

  const float liftIncrement =
      (LiftArmStateMachine::maxAngle - LiftArmStateMachine::minAngle) /
      ((float)75);

  /**
   * false = retracted
   *
   * true = expanded
   */
  bool wingsState = false;
  bool prevR1 = false;

  // blocker
  bool blockerState = false;
  bool prevR2 = false;

  bool prevCataEStopCombo = false;
  bool prevLiftEStopCombo = false;

  bool prevUp = false;
  bool prevDown = false;

  // timestamp of last R1 press
  int lastUpPress = 0;
  // timestamp of last R2 press
  int lastDownPress = 0;

  const int maxTimeBetweenDoublePress = 150;

  bool prevRight = 0;
  while (true) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 								     Drive Code
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // takes each side's drive power in the range [-127, 127] and a curve gain
    Robot::chassis->tank(
        // left drive power
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        // right drive power
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
        // drive curve gain to enable greater control of the robot.
        15);

    // intake / outtake
    // if pressing L1, then spin the intake inwards
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      Robot::Motors::intake.move(127);
    // if pressing L2, then spin the intake outwards
    else if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
      Robot::Motors::intake.move(-127);
    // otherwise, dont power the intake
    else Robot::Motors::intake.move(0);

    // matchload toggle
    const bool buttonRight =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (buttonRight && !prevRight) {
      if (Robot::Subsystems::catapult->getIsMatchloading())
        Robot::Subsystems::catapult->stop();
      else Robot::Subsystems::catapult->matchload();
    }
    prevRight = buttonRight;

    // blocker toggle
    // retrieve the value of the R2 button
    const bool buttonR2 =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    // if on the rising edge of the button
    if (buttonR2 && !prevR2) {
      // flip the state of the blocker
      blockerState = !blockerState;
      // apply the state of the blocker to the actual piston
      if (blockerState) Robot::Actions::expandBlocker();
      else Robot::Actions::retractBlocker();
    }
    // update previous value of R2
    prevR2 = buttonR2;

    // catapult manual fire
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      Robot::Subsystems::catapult->fire();

    // get whether both emergency stop buttons are currently being pressed
    const bool cataEStopCombo =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    // if both buttons have just become pressed (rising edge), then toggle the
    // emergency stop of the catapult
    if (cataEStopCombo && !prevCataEStopCombo) {
      // if the catapult is currently emergency stopped, then disable the
      // emergency stop
      if (Robot::Subsystems::catapult->getState() ==
          CatapultStateMachine::STATE::EMERGENCY_STOPPED)
        Robot::Subsystems::catapult->cancelEmergencyStop();
      // otherwise emergency stop the catapult
      else Robot::Subsystems::catapult->emergencyStop();
    }
    // update the previous value of the emergency stop buttons
    prevCataEStopCombo = cataEStopCombo;

    // Lift buttons
    // retrieve the values of the up and down buttons
    const bool up = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    const bool down =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    // lift granular control
    // change the goal position of the lift by the liftIncrement
    Robot::Subsystems::lift->changeTarget(liftIncrement * (up - down));

    // lift max/min angle
    // on rising edge of up, if up was pressed recently,
    // then set target to max angle
    if (up && !prevUp &&
        pros::millis() - lastUpPress < maxTimeBetweenDoublePress)
      Robot::Subsystems::lift->setTarget(LiftArmStateMachine::maxAngle);
    // on the rising edge of down, if down was pressed recently,
    // then set target to max angle
    if (down && !prevDown &&
        pros::millis() - lastDownPress < maxTimeBetweenDoublePress)
      Robot::Subsystems::lift->setTarget(LiftArmStateMachine::minAngle);

    // on the falling edge of up & down, update the last press time
    if (!up && prevUp) lastUpPress = pros::millis();
    if (!down && prevDown) lastDownPress = pros::millis();

    // update previous values of up and down
    prevUp = up;
    prevDown = down;

    // lift emergency stop
    const bool liftEStopCombo =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (liftEStopCombo && !prevLiftEStopCombo) {
      if (Robot::Subsystems::lift->getState() ==
          LiftArmStateMachine::STATE::EMERGENCY_STOPPED)
        Robot::Subsystems::lift->cancelEmergencyStop();
      else Robot::Subsystems::lift->emergencyStop();
    }
    prevLiftEStopCombo = liftEStopCombo;

    // wings toggle
    // retrieve the value of the R2 button
    const bool r1 = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    // if on the rising edge of the button
    if (r1 && !prevR1) {
      // flip the state of the wings
      wingsState = !wingsState;
      // apply the state of the wings to the actual pistons
      if (wingsState) Robot::Actions::expandWings();
      else Robot::Actions::retractWings();
    }
    // update the previous value of R1
    prevR1 = r1;

    pros::delay(10);
  }
}
