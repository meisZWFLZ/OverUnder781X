#include "auton.h"
#include "robot.h"
#include <functional>

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

void stop() { tank(0, 0, 0, 0); }

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
      std::remainder(Robot::chassis->getPose().theta - target, 360));
}

bool isTriballInIntake() {
  return Robot::Motors::intake.at(0).is_over_current();
}

bool isMotionRunning() { return Robot::chassis->isInMotion(); }

void waitUntil(std::function<bool(void)> condition, int timeConditionIsTrue,
               int timeout, bool resetTrueStartTime) {
  const int start = pros::millis();
  int conditionTrueStartTime = 0;
  while ((pros::millis() - start < timeout)) {
    const bool condVal = condition();
    if (!condVal && resetTrueStartTime) conditionTrueStartTime = 0;
    if (conditionTrueStartTime == 0 && condVal)
      conditionTrueStartTime = pros::millis();
    if (conditionTrueStartTime != 0 &&
        (pros::millis() - conditionTrueStartTime) > timeConditionIsTrue)
      break;

    pros::delay(10);
  }
}
} // namespace auton::utils