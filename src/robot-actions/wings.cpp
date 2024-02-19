#include "robot.h"

bool leftWingState = false;
bool rightWingState = false;
bool backWingState = false;

void setLeftWing(bool state) {
  leftWingState = state;
  Robot::Pistons::leftWing.set_value(leftWingState);
}

void setRightWing(bool state) {
  rightWingState = state;
  Robot::Pistons::rightWing.set_value(rightWingState);
}

void setBackWing(bool state) {
  backWingState = state;
  Robot::Pistons::backWing.set_value(backWingState);
}

void Robot::Actions::expandBothWings() {
  expandLeftWing();
  expandRightWing();
}

void Robot::Actions::expandLeftWing() { setLeftWing(true); }

void Robot::Actions::expandRightWing() { setRightWing(true); }

void Robot::Actions::retractBothWings() {
  retractLeftWing();
  retractRightWing();
}

void Robot::Actions::retractLeftWing() { setLeftWing(false); }

void Robot::Actions::retractRightWing() { setRightWing(false); }

void Robot::Actions::toggleBothWings() {
  toggleLeftWing();
  setRightWing(leftWingState);
}

void Robot::Actions::toggleLeftWing() { setLeftWing(!leftWingState); }

void Robot::Actions::toggleRightWing() { setRightWing(!rightWingState); }

void Robot::Actions::retractBackWing() { setBackWing(false); }

void Robot::Actions::expandBackWing() { setBackWing(true); }

void Robot::Actions::toggleBackWing() { setBackWing(!backWingState); }