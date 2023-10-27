#include "selector.h"
#include "pros/llemu.hpp"
#include <cmath>

using namespace auton;

int AutonSelector::index = 0;
std::vector<Auton*> AutonSelector::autons {};

void AutonSelector::attachListeners() {
  pros::lcd::register_btn0_cb(AutonSelector::decrement);
  pros::lcd::register_btn2_cb(AutonSelector::increment);
}

void AutonSelector::initScreen() { pros::lcd::initialize(); }

void AutonSelector::init() {
  AutonSelector::initScreen();
  AutonSelector::attachListeners();
}

void AutonSelector::increment() {
  index++;
  if (index >= autons.size()) index = 0;
  AutonSelector::updateDisplay();
}

void AutonSelector::decrement() {
  index--;
  if (index < 0) index = autons.size() - 1;
  AutonSelector::updateDisplay();
}

void AutonSelector::updateDisplay() {
  pros::lcd::set_text(0, autons[index]->label);
}

void AutonSelector::addAuton(Auton* auton) { autons.push_back(auton); }

void AutonSelector::runAuton() { autons[index]->run(); }
