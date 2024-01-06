#include "auton.h"
#include "robot.h"

namespace auton::utils {
void tankUpdate(float left, float right, float slew) {
  const int prevLeft = Robot::Motors::leftDrive.get_voltages()[0];
  const int prevRight = Robot::Motors::rightDrive.get_voltages()[0];
  const int leftLimited = lemlib::slew(left, prevLeft, slew);
  const int rightLimited = lemlib::slew(right, prevRight, slew);
  Robot::chassis->tank(leftLimited, rightLimited);
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
      printf("breaking\n");
      break;
    }
    pros::delay(10);
  } while (!checkMotionRunning || Robot::chassis->isInMotion());
  printf("time in wait: %i\n", pros::millis() - start);
}

int leftOrRight(int ifLeft, int ifRight) {
  return Robot::chassis->getPose().x < 0 ? ifLeft : ifRight;
}

} // namespace auton::utils