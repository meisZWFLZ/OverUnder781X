#include "controllerScreen.h"
#include <string>
#include "pros/rtos.h"
#include <string.h>
#include <streambuf>
#include <algorithm>

const ControllerScreenConfig ControllerScreen::DEFAULT_CONFIG = {
    .lines = 3,
    .columns = 14,
    .minTimeBetweenPrints = 50,
    .minTimeBetweenVibrates = 100};

void ControllerScreen::setText(unsigned int line, unsigned int col,
                               const char* cStr) {
  if (line > config.lines) return;
  std::string curStr(this->queuedLines[line]);
  curStr.insert(col, cStr);

  queuedLines[line] = curStr.substr(0, config.columns).c_str();
}

void ControllerScreen::print(unsigned int line, const char* cStr) {
  this->setText(line, 0, cStr);
}

const char* ControllerScreen::getLine(unsigned int line) const {
  return (char*)currLines[line];
}

void ControllerScreen::vibrate(const char* pattern) {
  this->vibrateRequested = (char*)pattern;
}

void ControllerScreen::update() {
  if (strlen(this->vibrateRequested) &&
      (pros::millis() - lastVibrateTime) > config.minTimeBetweenVibrates)
    vibrateController();
  else if ((pros::millis() - lastPrintTime) > config.minTimeBetweenPrints) {
    int line = lastLinePrinted + 1;
    for (int i = 0; i < config.lines; i++, line++, line %= config.lines) {
      if (strcmp(currLines[line], queuedLines[line]) != 0) {
        printToController(line);
        break;
      }
    }
  }
}

ControllerScreen::ControllerScreen(pros::Controller* con,
                                   const ControllerScreenConfig& config)
  : con(con), config(config), lastLinePrinted(0), lastPrintTime(0) {
  for (int i = 0; i < config.lines; i++) {
    queuedLines.push_back("");
    currLines.push_back("");
  }

  this->con->clear();
  lastPrintTime = pros::millis();
}

void ControllerScreen::printToController(int line) {
  con->print(line, 0, queuedLines[line]);
  currLines[line] = queuedLines[line];
  lastPrintTime = pros::millis();
  lastLinePrinted = line;
}

void ControllerScreen::vibrateController() {
  con->rumble(this->vibrateRequested);
  this->vibrateRequested = "";
  lastVibrateTime = pros::millis();
}