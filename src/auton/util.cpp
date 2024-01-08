#include "auton.h"
#include "robot.h"

namespace auton::utils {
float prevLeft = 0;
float prevRight = 0;

void tankUpdate(float left, float right, float slew) {
  const float leftLimited = lemlib::slew(left, prevLeft, slew);
  const float rightLimited = lemlib::slew(right, prevRight, slew);
  Robot::chassis->tank(leftLimited, rightLimited);
  prevLeft = leftLimited;
  prevRight = rightLimited;
}

void tank(float left, float right, int ms, float slew) {
  const int startTime = pros::millis();
  do {
    tankUpdate(left, right, slew);
    pros::delay(10);
  } while (pros::millis() - startTime < ms);
}

void stop() { Robot::chassis->tank(0, 0); }

void waitUntilDistToPose(lemlib::Pose pose, float error, int time,
                         bool checkMotionRunning) {
  printf("start wait\n");
  const int start = pros::millis();
  int inRangeStartTime = 0;
  pros::delay(100);
  do {
    if (inRangeStartTime == 0) {
      if (Robot::chassis->getPose().distance(pose) < error)
        inRangeStartTime = pros::millis();
    } else if (pros::millis() - inRangeStartTime > time) {
      break;
    }
    pros::delay(10);
  } while (!checkMotionRunning || Robot::chassis->isInMotion());
  printf("time in wait: %i\n", pros::millis() - start);
}

int leftOrRight(int ifLeft, int ifRight) {
  return Robot::chassis->getPose().x < 0 ? ifLeft : ifRight;
}

float robotAngDist(float target) {
  return std::fabs(
      lemlib::angleError(Robot::chassis->getPose().theta, target, false));
}

} // namespace auton::utils