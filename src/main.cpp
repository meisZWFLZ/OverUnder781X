#include "main.h"
#include "catapult.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "lift.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "auton.h"
#include <cmath>
#include <iostream>
#include <sstream>
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
    // pros::lcd::print(5, "current: %i, %i",
    //                  Robot::Motors::elevator.at(0).get_current_draw(),
    //                  Robot::Motors::elevator.at(1).get_current_draw());
    // auto currLim = Robot::Motors::elevator.are_over_current();
    // pros::lcd::print(6, "curr lim: %i,%i", currLim[0], currLim[1]);
    lemlib::Pose pose =
        Robot::chassis->getPose(); // get the current position of the

    pros::lcd::clear_line(1);
    pros::lcd::print(1, "x: %f in", pose.x); // print the x position
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "y: %f in", pose.y); // print the y position
    pros::lcd::clear_line(3);
    pros::lcd::print(3, "heading: %f deg",
                     pose.theta); // print the heading
    pros::lcd::print(4, "cata: %i deg",
                     Robot::Sensors::cata.get_angle()); // print the heading
    // const float deltaTheta = Robot::chassis->getPose().theta - lastHeading;
    // if (deltaTheta > 10) {
    //   Robot::chassis->setPose(Robot::chassis->getPose().x,
    //                           Robot::chassis->getPose().y, lastHeading);
    //   Robot::control.print(1, 1, "imu threw");
    // } else
    //   Robot::chassis->setPose(Robot::chassis->getPose().x,
    //                           Robot::chassis->getPose().y,
    //                           lastHeading + deltaTheta * 360 / 355);
    // if (pros::competition::is_autonomous() &&
    //     pros::competition::is_disabled() && pros::millis() % 80 < 20)
    //   printf("th:%i\t%4.2f\n", pros::millis(),
    //   Robot::chassis->getPose().theta);

    lastHeading = Robot::chassis->getPose().theta;
    pros::delay(20);
  }
}

pros::Task* screenTask;

void addAutons() {
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::disrupt);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::sixBall);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::defensive);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::fiveRush);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::sixRushElims);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::doNothing);
  Robot::Subsystems::autonSelector->addAuton(&auton::autons::skills);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  Robot::initializeOdometry();

  Robot::chassis->calibrate(true); // calibrate the chassis

  pros::lcd::set_text(1, "Calibrating chassis...");
  Robot::Subsystems::initialize();
  pros::lcd::set_text(1, "Chassis Calibrated!");

  addAutons();
  Robot::Subsystems::autonSelector->init();
  Robot::Subsystems::autonSelector->enable();

  // create a task to print the position to the screen
  new pros::Task {screen};
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  Robot::Motors::leftDrive.brake();
  Robot::Motors::rightDrive.brake();
}

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
  Robot::Subsystems::autonSelector->disable();
  autonHasRun = true;
  if (pros::competition::is_connected()) Robot::Actions::prepareRobot();
  // printf("auton start");

  Robot::Subsystems::autonSelector->runAuton();
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

std::vector<std::string> split(const std::string& _input,
                               const std::string& delimiter) {
  std::vector<std::string> tokens;
  std::string source = _input;
  size_t pos = 0;
  while ((pos = source.find(delimiter)) != std::string::npos) {
    tokens.push_back(source.substr(0, pos));
    source.erase(0, pos + delimiter.length());
  }
  tokens.push_back(source);
  return tokens;
}

void makeLowerCase(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  ;
}

enum TUNING_MODE { LATERAL, ANGULAR };

float getValue(const TUNING_MODE mode) {
  if (mode == LATERAL) return Robot::chassis->getPose().y;
  return Robot::chassis->getPose().theta;
}

void print(const TUNING_MODE mode, bool key = true) {
  if (mode == LATERAL) printf("%s%fin\n", key ? "y: " : "", getValue(mode));
  else printf("%s%fdeg\n", key ? "theta: " : "", getValue(mode));
  printf("battery: %i\n", int(pros::battery::get_capacity()));

  // print average drive motor temperature
  auto temps = Robot::Motors::leftDrive.get_temperatures();
  auto rightTemps = Robot::Motors::rightDrive.get_temperatures();
  // concat right motor temps to temps
  temps.insert(temps.end(), rightTemps.begin(), rightTemps.end());
  // sum temps
  float sum = 0;
  for (const auto& temp : temps) sum += temp;
  float average = sum / temps.size();
  printf("motor temp: %f\n", average);
}

void tuningCLI() {
  static TUNING_MODE mode = ANGULAR;
  lemlib::PID* pid = &(mode == LATERAL ? Robot::chassis->lateralPID
                                       : Robot::chassis->angularPID);
  lemlib::ControllerSettings* settings =
      &(mode == LATERAL ? Robot::chassis->lateralSettings
                        : Robot::chassis->angularSettings);

  const float defaultAngularDist = 90;
  const float defaultLateralDist = 24;
  float angularDist = defaultAngularDist;
  float lateralDist = defaultLateralDist;
  while (1) {
    try {
      std::cout << "pid tuner> ";
      std::string input;
      getline(std::cin, input);
      makeLowerCase(input);
      auto params = split(input, " ");
      std::string command = params.at(0);

      if (command == "s" || command == "set") {
        if (params.size() < 3) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string gainType = params.at(1);
        std::string gainValueStr = params.at(2);
        float gainValue = std::stof(gainValueStr);

        if (gainType.find("p") != std::string::npos) {
          pid->kP = gainValue;
        } else if (gainType.find("d") != std::string::npos) {
          pid->kD = gainValue;
        } else if (gainType.find("i") != std::string::npos) {
          pid->kI = gainValue;
        } else if (gainType.find("s") != std::string::npos) {
          settings->slew = gainValue;
        } else {
          std::cout << "invalid gain type" << std::endl;
        }
      } else if (command == "g" || command == "get") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string gainType = params.at(1);
        if (gainType == "mode") {
          std::cout << "mode: " << (mode == LATERAL ? "lateral" : "angular")
                    << std::endl;
        } else if (gainType == "dist") {
          std::cout << "dist: " << (mode == LATERAL ? lateralDist : angularDist)
                    << std::endl;
        } else if (gainType.find("p") != std::string::npos) {
          std::cout << "kP: " << pid->kP << std::endl;
        } else if (gainType.find("d") != std::string::npos) {
          std::cout << "kD: " << pid->kD << std::endl;
        } else if (gainType.find("i") != std::string::npos) {
          std::cout << "kI: " << pid->kI << std::endl;
        } else if (gainType.find("s") != std::string::npos) {
          std::cout << "slew: " << settings->slew << std::endl;
        } else {
          std::cout << "invalid gain type" << std::endl;
        }
      } else if (command == "run" || command == "x" || command == "rr") {
        Robot::chassis->cancelMotion();
        Robot::chassis->setPose(0, 0, 0);
        float timeout = 2000;
        bool wait = true;
        if (params.size() > 1) {
          auto noWaitIt = find(params.begin(), params.end(), "-n");
          if (noWaitIt != params.end()) {
            timeout = 1000000;
            wait = false;
          }

          auto timeoutIt = find(params.begin(), params.end(), "-t");
          if (timeoutIt != params.end() && ++timeoutIt != params.end()) {
            timeout = std::stof(*timeoutIt);
          }
        }
        const bool reversed = command == "rr";
        const int multiplier = reversed ? -1 : 1;
        const int startTime = pros::millis();
        switch (mode) {
          case LATERAL:
            Robot::chassis->moveToPoint(0, multiplier * lateralDist, timeout,
                                        {.forwards = !reversed});
            break;
          case ANGULAR:
            Robot::chassis->turnToHeading(multiplier * angularDist, timeout);
            break;
        }

        if (wait) {
          float prev = getValue(mode);
          float prevOscillation = 0;
          float prevVel = 0;
          int count = 0;

          printf("oscillations\nnum\ttime\tcurr\taccel\n");
          while (Robot::chassis->isInMotion() &&
                 !Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            pros::delay(10);
            const float curr = getValue(mode);
            const float vel = (curr - prev) / 0.01;
            const float smoothVel = vel * 0.75 + prevVel * 0.25;
            prev = curr;

            // if sign of velocity changes, and this oscillation is not small
            if (smoothVel * prevVel < 0 && std::fabs(prevOscillation - curr) >
                                               (mode == LATERAL ? 0.25 : 1)) {
              printf("%i\t%4.2f\t%4.2f\t%4.2f\n", ++count,
                     float(pros::millis() - startTime) / 1000, curr,
                     smoothVel - prevVel);
              prevOscillation = curr;
            }
            prevVel = vel;
          }
          printf("\n");
          print(mode);
        }
      } else if (command == "print" || command == "p") {
        print(mode);
      } else if (command == "stop" || command == "s") {
        Robot::chassis->cancelMotion();
      } else if (command == "exit") {
        break;

      } else if (command == "mode") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string newMode = params.at(1);

        if (newMode.find("l") != std::string::npos) {
          std::cout << "switching to lateral mode" << std::endl;
          pid = &Robot::chassis->lateralPID;
          settings = &Robot::chassis->lateralSettings;
          mode = LATERAL;
        } else if (newMode.find("a") != std::string::npos) {
          std::cout << "switching to angular mode" << std::endl;
          mode = ANGULAR;
          pid = &Robot::chassis->angularPID;
          settings = &Robot::chassis->angularSettings;
        } else {
          std::cout << "invalid mode" << std::endl;
        }

      } else if (command == "dist") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        float dist;
        if (std::string(params.at(1)).find("r") != std::string::npos)
          dist = mode == LATERAL ? defaultLateralDist : defaultAngularDist;
        dist = std::stof(params.at(1));

        printf("setting %s dist to: %f\n",
               mode == LATERAL ? "lateral" : "angular", dist);

        if (mode == LATERAL) {
          lateralDist = dist;
        } else {
          angularDist = dist;
        }

      } else {
        std::cout << "invalid command" << std::endl;
      }
    } catch (std::exception e) {
      std::cout << "error: " << e.what() << std::endl;
    }

    pros::delay(10);
  }
}

const bool tuneModeEnabled = true;

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() {
  Robot::chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  // Robot::Subsystems::autonSelector->disable();
  Robot::Actions::retractBothWings();
  Robot::Actions::retractBackWing();

  // int start = pros::millis();/
  // while (Robot::Subsystems::catapult->getTriballsFired() < 46) {
  //   Robot::Subsystems::catapult->fire();
  //   pros::delay(1000);
  // }
  // printf("time: %i\n", pros::millis() - start);

  bool skills = false;
  if (pros::competition::is_connected() && !autonHasRun) {
    if (!std::strcmp(Robot::Subsystems::autonSelector->getCurrentAuton(),
                     (char*)("skills")))
      skills = true;
    else Robot::Actions::prepareRobot();
  }

  if (skills) {
    auton::actions::matchload();
    Robot::Motors::leftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    Robot::Motors::rightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  }

  bool prevR1 = false;

  // blocker
  bool blockerState = false;
  bool prevR2 = false;

  bool prevCataEStopCombo = false;
  bool prevLiftEStopCombo = false;
  bool prevLiftLockCombo = false;

  bool prevLeft = false;
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

    const bool l1 = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool l2 = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    // intake / outtake
    // if pressing L1 && L2, then spin the intake inwards slowly
    if (l1 && l2) Robot::Motors::intake.move(127 * .25);
    // if pressing only L1, then spin the intake inwards
    else if (l1) Robot::Motors::intake.move(127);
    // if pressing only L2, then spin the intake outwards
    else if (l2) Robot::Motors::intake.move(-127);
    // otherwise, dont power the intake
    else Robot::Motors::intake.move(0);

    // matchload toggle
    const bool buttonLeft =
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    if (buttonLeft && !prevLeft) {
      if (Robot::Subsystems::catapult->getIsMatchloading())
        Robot::Subsystems::catapult->stop();
      else Robot::Subsystems::catapult->matchload();
    }
    prevLeft = buttonLeft;

    // catapult manual fire
    if (Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
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

    // if up is pressed
    if (!up && prevUp) {
      switch (Robot::Subsystems::lift->getState()) {
        // if the current state is retracting, then stop retracting
        case LiftArmStateMachine::STATE::RETRACTING:
          Robot::Subsystems::lift->release();
          break;
        // if the current state is idle, then extend the lift
        case LiftArmStateMachine::STATE::IDLE:
          Robot::Subsystems::lift->extend();
          break;
        // if the current state is extending the do nothing
        case LiftArmStateMachine::STATE::EXTENDING: break;
      }
    }
    // if down is pressed
    if (!down && prevDown) {
      switch (Robot::Subsystems::lift->getState()) {
        // if the current state is retracting, then do nothing
        case LiftArmStateMachine::STATE::RETRACTING: break;
        // if the current state is idle, then retract the lift
        case LiftArmStateMachine::STATE::IDLE:
          Robot::Subsystems::lift->retract();
          break;
        // if the current state is extending, then stop extending
        case LiftArmStateMachine::STATE::EXTENDING:
          Robot::Subsystems::lift->release();
          break;
      }
    }

    // update previous values of up and down
    prevUp = up;
    prevDown = down;

    // wings
    Robot::Subsystems::wings->driverUpdate();

    if (tuneModeEnabled &&
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_B))
      tuningCLI();

    pros::delay(10);
  }
}
