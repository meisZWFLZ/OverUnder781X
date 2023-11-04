#include "selector.h"
#include "pros/llemu.hpp"
#include <cmath>

using namespace auton;

int AutonSelector::index = 0;
bool AutonSelector::state = false;
std::vector<Auton*> AutonSelector::autons {};

void AutonSelector::attachListeners() {
  pros::lcd::register_btn0_cb(AutonSelector::decrementListener);
  pros::lcd::register_btn2_cb(AutonSelector::incrementListener);
}

void AutonSelector::initScreen() { pros::lcd::initialize(); }

void AutonSelector::init() {
  AutonSelector::initScreen();
  AutonSelector::attachListeners();
  AutonSelector::updateDisplay();
}

void AutonSelector::incrementListener() {
  if (!AutonSelector::isEnabled()) return;
  index++;
  if (index >= autons.size()) index = 0;
  AutonSelector::updateDisplay();
}

void AutonSelector::decrementListener() {
  if (!AutonSelector::isEnabled()) return;
  index--;
  if (index < 0) index = autons.size() - 1;
  AutonSelector::updateDisplay();
}

void AutonSelector::clearDisplay() {
  pros::lcd::clear_line(0);
}
void AutonSelector::updateDisplay() {
  pros::lcd::set_text(0, autons[index]->label);
}

void AutonSelector::addAuton(Auton* auton) { autons.push_back(auton); }

void AutonSelector::enable() {
  AutonSelector::state = true;
  AutonSelector::updateDisplay();
}
void AutonSelector::disable() {
  AutonSelector::state = false;
  AutonSelector::clearDisplay();
}

bool AutonSelector::isEnabled() {
  return AutonSelector::state;
}

void AutonSelector::runAuton() { 
  autons[index]->run(); 
}
char* AutonSelector::getCurrentAuton() {
  return autons[index]->label;
}