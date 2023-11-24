// /*----------------------------------------------------------------------------*/
// /*                                                                            */
// /*    Module:       main.cpp                                                  */
// /*    Author:       VEX                                                       */
// /*    Created:      Thu Sep 26 2019                                           */
// /*    Description:  Competition Template                                      */
// /*                                                                            */
// /*----------------------------------------------------------------------------*/

// // ---- START VEXCODE CONFIGURED DEVICES ----
// // Robot Configuration:
// // [Name]               [Type]        [Port(s)]
// // Controller1          controller
// // LDrive               motor_group   11, 13
// // RDrive               motor_group   12, 18
// // vRot                 rotation      19
// // ---- END VEXCODE CONFIGURED DEVICES ----

// // #include "vex.h"
// #include "neil_pid.h"
// #include "pros/motors.h"
// #include <cmath> //std::abs

// // using namespace vex;

// // A global instance of competition
// // competition Competition;

// // define your global instances of motors and other devices here

// /*---------------------------------------------------------------------------*/
// /*                          Pre-Autonomous Functions                         */
// /*                                                                           */
// /*  You may want to perform some actions before the competition starts.      */
// /*  Do them in the following function.  You must return from this function   */
// /*  or the autonomous and usercontrol tasks will not be started.  This       */
// /*  function is only called once after the V5 has been powered on and        */
// /*  not every time that the robot is disabled.                               */
// /*---------------------------------------------------------------------------*/

// // void pre_auton(void) {
// //   // Initializing Robot Configuration. DO NOT REMOVE!
// //   vexcodeInit();

// //   // All activities that occur before the competition starts
// //   // Example: clearing encoders, setting servo positions, ...
// // }

// // Settings

// const int maxTurnIntegral = 300; // These cap the integrals
// const int maxIntegral = 300;
// const int integralBound =
//     3; // If error is outside the bounds, then apply the integral. This is a
//        // buffer with +-integralBound degrees

// // Autonomous Settings
// // int desiredValue = 0;
// // int desiredTurnValue = 0;
// // float waypoints[][2] = {
// //     {36, 0} // Waypoint 1 (x, y)
// //             // Waypoint 2 (x, y)
// //             // Waypoint 3 (x, y)
// // };
// // int numberOfTimesRan = 0;

// // // int currentWaypoint = 0;
// // int turnError; // SensorValue - DesiredValue : Position

// bool resetDriveSensors = false;

// // Variables modified for use
// bool enableDrivePID = true;

// // Pasted from a C++ resource
// double signnum_c(double x) {
//   if (x > 0.0) return 1.0;
//   if (x < 0.0) return -1.0;
//   return x;
// }

// float getLeftEncDegrees() {
//   return Robot::Sensors::leftDrive.get_position() *
//          Robot::Dimensions::driveEncGearRatio / 100;
// };

// float getRightEncDegrees() {
//   return Robot::Sensors::rightDrive.get_position() *
//          Robot::Dimensions::driveEncGearRatio / 100;
// };

// void PID::drive(float goalDriveValue, float goalTurnValue) {
//   float error; // SensorValue - DesiredValue : Position
//   float prevError = 0; // Position 20 miliseconds ago
//   float derivative; // error - prevError : Speed
//   float totalError = 0; // totalError = totalError + error

//   float turnPrevError = 0; // Position 20 miliseconds ago
//   float turnTotalError = 0; // totalError = totalError + error
//   Robot::Sensors::leftDrive.set_position(0);
//   Robot::Sensors::rightDrive.set_position(0);
//   // Robot::Sensors::leftDrive.set_data_rate(5);
//   // Robot::Sensors::rightDrive.set_data_rate(5);

//   // printf(" current waypoint%d\n", currentWaypoint);
//   printf("%s\n", "pid true");
//   while (enableDrivePID) {
//     // printf("desired val%f\n", goalDriveValue);
//     // printf("desired turn val%d\n", desiredTurnValue);
//     // if (resetDriveSensors) {
//     //   LDrive.setPosition(0, degrees);
//     //   RDrive.setPosition(0, degrees);
//     //   resetDriveSensors = false;
//     // }
//     // currentWaypoint == sizeof(waypoints)/ sizeof(waypoints[0])

//     // if (currentWaypoint < sizeof(waypoints) / sizeof(waypoints[0])) {
//     //   // Get target waypoint coordinates
//     //   desiredValue = 720 - 100;
//     //   // desiredValue = waypoints[currentWaypoint][0];
//     //   desiredTurnValue = waypoints[currentWaypoint][1];
//     // }

//     // Get the position of both motors
//     float leftDrivePosition = getLeftEncDegrees();
//     float rightDrivePosition = getRightEncDegrees();
//     printf("left:%f\n", leftDrivePosition);
//     printf("right:%f\n", rightDrivePosition);
//     ///////////////////////////////////////////
//     // Lateral movement PID
//     /////////////////////////////////////////////////////////////////////
//     // Get average of the two motors
//     float averagePosition =
//         (leftDrivePosition + rightDrivePosition) / 2; // why -2!?
//     // if (averagePosition == 360) {
//     //   resetDriveSensors = true;
//     //   LDrive.stop();
//     //   RDrive.stop();
//     // }
//     printf("avg:%f\n", averagePosition);
//     // Potential
//     error = averagePosition - goalDriveValue;
//     printf("err:%f\n", error);
//     // Derivative
//     derivative = error - prevError;

//     // Integral
//     if (abs(error) < integralBound) { // it just quits after too much error!?
//       totalError += error;
//     } else {
//       totalError = 0;
//     }
//     // totalError += error;

//     // This would cap the integral - yeah, but why!?
//     totalError = abs(totalError) > maxIntegral
//                      ? signnum_c(totalError) * maxIntegral
//                      : totalError;

//     double lateralMotorPower =
//         error * PID::kP + derivative * PID::kD + totalError * PID::kI;
//     /////////////////////////////////////////////////////////////////////

//     ///////////////////////////////////////////
//     // Turning movement PID
//     /////////////////////////////////////////////////////////////////////
//     // Get average of the two motors
//     float turnDifference = leftDrivePosition - rightDrivePosition;

//     // Potential
//     float turnError = turnDifference - goalTurnValue;

//     // Derivative
//     float turnDerivative = turnError - turnPrevError;

//     // Integral
//     if (abs(error) < integralBound) {
//       turnTotalError += turnError;
//     } else {
//       turnTotalError = 0;
//     }
//     // turnTotalError += turnError;

//     // This would cap the integral
//     turnTotalError = abs(turnTotalError) > maxIntegral
//                          ? signnum_c(turnTotalError) * maxIntegral
//                          : turnTotalError;

//     double turnMotorPower = turnError * PID::turnkP +
//                             turnDerivative * PID::turnkD +
//                             turnTotalError * PID::turnkI;
//     /////////////////////////////////////////////////////////////////////
//     printf("pow:%f\n", lateralMotorPower);
//     printf("terr:%f\n", turnError);
//     printf("tdif:%f\n", turnDifference);
//     printf("tpow%f\n", turnMotorPower);
//     Robot::Motors::leftDrive.move_voltage((lateralMotorPower + turnMotorPower) *
//                                           1000);
//     Robot::Motors::rightDrive.move_voltage(
//         (lateralMotorPower - turnMotorPower) * 1000);
//     //  LDrive.spin(forward, lateralMotorPower +
//     // turnMotorPower,
//     //             voltageUnits::volt);
//     // RDrive.spin(forward, lateralMotorPower - turnMotorPower,
//     //             voltageUnits::volt);

//     prevError = error;
//     turnPrevError = turnError;
//     // numberOfTimesRan++;
//     if (abs(error) < 10 && abs(turnError) < 0.05) {
//       break;
//       // resetDriveSensors = true;
//       // currentWaypoint++;
//       // desiredValue = waypoints[currentWaypoint][0];
//       // ;
//       // desiredTurnValue = waypoints[currentWaypoint][1];
//     }
//     pros::delay(50);
//   }

//   printf("%s\n", "pid false");
// }

// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                              Autonomous Task                              */
// /*                                                                           */
// /*  This task is used to control your robot during the autonomous phase of   */
// /*  a VEX Competition.                                                       */
// /*                                                                           */
// /*  You must modify the code to add your own robot specific commands here.   */
// /*---------------------------------------------------------------------------*/

// // void autonomous(void) {
// //   // printf("%s/n","auton called");

// //   /// First set of instructoin
// //   // vex::task billWiTheScienceFi(drivePID);
// //   drivePID(10);
// //   turnPID(30);
// //   drivePID(20);
// //   turnPID(290);

// //   printf("%s\n", "auton true");
// //   // vex::task::sleep(1000);

// //   // secon set of insctrution
// //   // resetDriveSensors = true;
// //   // desiredValue = 360;
// //   //  desiredTurnValue = 45;

// //   // vex::task::sleep(1000);

// //   // end pid
// //   // enableDrivePID = false;
// //   // resetDriveSensors = true;

// //   // LDrive.stop();
// //   // RDrive.stop();
// // }

// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                              User Control Task                            */
// /*                                                                           */
// /*  This task is used to control your robot during the user control phase of */
// /*  a VEX Competition.                                                       */
// /*                                                                           */
// /*  You must modify the code to add your own robot specific commands here.   */
// /*---------------------------------------------------------------------------*/

// // void usercontrol(void) {
// //   //  enableDrivePID = false;
// //   /*
// //     ///////////////////////////
// //     //Settings
// //     ///////////////////////////////////////////////////////////////////////////

// //     //Drivetrain
// //     double turnImportance = 0.5;

// //     while (1) {

// //       ///////////////////////////
// //       //Driver Control
// //       ///////////////////////////////////////////////////////////////////////////
// //       double turnVal = Controller1.Axis1.position(percent);
// //       double forwardVal = Controller1.Axis3.position(percent);

// //       double turnVolts = turnVal * 0.12;
// //       double forwardVolts = forwardVal * 0.12 * (1 -
// //       (std::abs(turnVolts)/12.0)
// //     * turnImportance);

// //       //0 - 12 = -12
// //       //0 + 12 = 12(due to cap)

// //       LeftMotor.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
// //       RightMotor.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
// //       ///////////////////////////////////////////////////////////////////////////

// //       ///////////////////////////
// //       //Arm Control
// //       ///////////////////////////////////////////////////////////////////////////
// //       bool topRightButton = Controller1.ButtonR1.pressing();
// //       bool bottomRightButton = Controller1.ButtonR2.pressing();

// //       if (topRightButton){
// //         ArmMotor.spin(forward, 12.0, voltageUnits::volt);
// //       }
// //       else if (bottomRightButton){
// //         ArmMotor.spin(forward, -12.0, voltageUnits::volt);
// //       }
// //       else{
// //         ArmMotor.spin(forward, 0, voltageUnits::volt);
// //       }
// //       ///////////////////////////////////////////////////////////////////////////

// //       ///////////////////////////
// //       //Claw Control
// //       ///////////////////////////////////////////////////////////////////////////
// //       bool topLeftButton = Controller1.ButtonL1.pressing();
// //       bool bottomLeftButton = Controller1.ButtonL2.pressing();

// //       if (topLeftButton){
// //         ClawMotor.spin(forward, 12.0, voltageUnits::volt);
// //       }
// //       else if (bottomLeftButton){
// //         ClawMotor.spin(forward, -12.0, voltageUnits::volt);
// //       }
// //       else{
// //         ClawMotor.spin(forward, 0, voltageUnits::volt);
// //       }
// //       ///////////////////////////////////////////////////////////////////////////

// //       wait(20, msec); // Sleep the task for a short amount of time to
// //                       // prevent wasted resources.
// //     }
// //       */
// // }

// //
// // Main will set up the competition functions and callbacks.
// //
// // int main() {
// //   // Set up callbacks for autonomous and driver control periods.
// //   Competition.autonomous(autonomous);
// //   // printf("%s/n","calling auton");
// //   Competition.drivercontrol(usercontrol);

// //   // Run the pre-autonomous function.
// //   pre_auton();

// //   // Prevent main from exiting with an infinite loop.
// //   while (true) {
// //     double leftVal = Controller1.Axis3.position(percent);
// //     double rightVal = Controller1.Axis2.position(percent);

// //     double leftVolts = leftVal * 0.12;
// //     double rightVolts = rightVal * 0.12;
// //     // printf("left drive speed %f\n",leftVal );
// //     // printf("right drive speed %f\n",rightVal );
// //     LDrive.spin(forward, leftVolts, voltageUnits::volt);
// //     RDrive.spin(forward, rightVolts, voltageUnits::volt);
// //     wait(100, msec);
// //   }
// // }