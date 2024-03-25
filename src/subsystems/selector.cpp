#include "selector.h"
#include "pros/llemu.hpp"
#include <cmath>

using namespace auton;

const std::string ERR = "\x1b[31m[ERR]\x1b[0m";

float AutonSelector::getDegreesPerAuton() const {
  const int teethPerAuton = floor(float(this->gearTeeth) / this->autons.size());
  float degreesPerAuton = 360.0 / teethPerAuton;

  // if there more autons than teeth (should be impossible), set the degrees per
  // auton to 360 / autons.size()
  if (teethPerAuton < 1) degreesPerAuton = 360.0 / this->autons.size();

  return degreesPerAuton;
}

unsigned int AutonSelector::getIndex() {
  const float degreesPerAuton = getDegreesPerAuton();

  const float sensedDegrees = float(this->rotation.get_angle()) / 100;
  int index = floor(sensedDegrees / degreesPerAuton);

  // if index is out of bounds, return the last auton
  if (index >= autons.size()) index = autons.size() - 1;

  return index;
}

void AutonSelector::setIndex(unsigned int newIndex) {
  // Ensure new index is within the bounds of autons.size()
  if (newIndex >= autons.size() || newIndex < 0) {
    printf((ERR + "AutonSelector::setIndex(): newIndex out of bounds: %i\n")
               .c_str(),
           newIndex);
  }

  const unsigned int currIndex = this->getIndex();
  // If the index is the same, there is no need to update
  if (newIndex == currIndex) return;

  const float degreesPerAuton = getDegreesPerAuton();

  const float minTargetDegrees = newIndex * degreesPerAuton;
  const float currentDegrees = float(this->rotation.get_angle()) / 100;

  // The minimum degrees to be at the current index
  const float currIndexMinDegrees = currIndex * degreesPerAuton;
  // The degrees past currIndexMinDegrees
  const float additionalDegrees = currIndexMinDegrees - currentDegrees;

  // If additionalDegrees is negative, something went wrong
  if (additionalDegrees < 0) {
    // print some debug info
    printf((ERR + "AutonSelector::setIndex(): additionalDegrees is negative\n"
                  "\tcurrentDegrees:\t%4.2f\n"
                  "\tadditionalDegrees:\t%4.2f\n"
                  "\tcurrIndexMinDegrees:\t%4.2f\n")
               .c_str(),
           currentDegrees, additionalDegrees, currIndexMinDegrees);
    return;
  }

  // Adjusts minTargetDegrees such that it is a multiple of degreesPerAuton away
  // from currentDegrees.
  // Ensures that the current position is at the same position within the range
  // of the new auton as it was in the old auton
  const float targetDegrees = minTargetDegrees + additionalDegrees;

  // Set the position of the rotation sensor
  // im not sure if set_position takes centidegrees
  this->rotation.set_position(targetDegrees * 100);
  if (this->rotation.get_position() != targetDegrees * 100) {
    // print some debug info
    printf((ERR +
            "AutonSelector::setIndex(): failed to correctly set position\n"
            "\ttargetDegrees:\t%4.2f\n"
            "\tget_position():\t%4.2f\n")
               .c_str(),
           targetDegrees, float(this->rotation.get_position()) / 100);
  }

  // Check if the index was set correctly
  if (this->getIndex() != newIndex) {
    // print some debug info
    printf((ERR + "AutonSelector::setIndex(): failed to set index\n"
                  "\tnewIndex:\t%i\n"
                  "\tcurrIndex:\t%i\n"
                  "\tgetIndex():\t%i\n"
                  "\tcurrentDegrees:\t%4.2f\n"
                  "\tminTargetDegrees:\t%4.2f\n"
                  "\ttargetDegrees:\t%4.2f\n")
               .c_str(),
           newIndex, currIndex, this->getIndex(), currentDegrees,
           minTargetDegrees, targetDegrees);
    return;
  }

  // update the display to show the new auton
  this->updateDisplay();
}

void AutonSelector::changeIndex(int indexChange) {
  unsigned int newIndex = this->getIndex() + indexChange;

  // ensure the index is within the bounds of (-autons.size(), autons.size())
  newIndex = newIndex % autons.size();

  // if the index is negative, add the size of the autons to it
  // this will limit the index to the range of [0, autons.size())
  if (newIndex < 0) newIndex = newIndex + autons.size();

  this->setIndex(newIndex);
}

void AutonSelector::initScreen() { pros::lcd::initialize(); }

void AutonSelector::init() {
  this->enable();
  this->initScreen();
  this->updateDisplay();
}

void AutonSelector::clearDisplay() { pros::lcd::clear_line(0); }

void AutonSelector::updateDisplay() {
  pros::lcd::set_text(0, this->getCurrentAuton());
}

bool AutonSelector::addAuton(Auton* auton) {
  if (this->autons.size() >= this->gearTeeth) return false;
  this->autons.push_back(auton);
  return true;
}

void AutonSelector::enable() {
  AutonSelector::state = true;
  AutonSelector::updateDisplay();
}

void AutonSelector::disable() {
  AutonSelector::state = false;
  AutonSelector::clearDisplay();
}

bool AutonSelector::isEnabled() { return AutonSelector::state; }

void AutonSelector::runAuton() { autons[this->getIndex()]->run(); }

char* AutonSelector::getCurrentAuton() {
  return autons[this->getIndex()]->label;
}

enum class BUTTON { LEFT, CENTER, RIGHT, NONE };

unsigned int buttonToMask(BUTTON button) {
  switch (button) {
    case BUTTON::LEFT: return 0b100;
    case BUTTON::CENTER: return 0b010;
    case BUTTON::RIGHT: return 0b001;
    default: return 0;
  }
}

bool isButtonOnRisingEdge(unsigned int prevState, unsigned int currState,
                          BUTTON button) {
  const bool prevButton = prevState & buttonToMask(button);
  const bool currButton = currState & buttonToMask(button);
  return !prevButton && currButton;
}

BUTTON getNewButtonPress(unsigned int prevState, unsigned int currState) {
  if (isButtonOnRisingEdge(prevState, currState, BUTTON::LEFT))
    return BUTTON::LEFT;
  if (isButtonOnRisingEdge(prevState, currState, BUTTON::CENTER))
    return BUTTON::CENTER;
  if (isButtonOnRisingEdge(prevState, currState, BUTTON::RIGHT))
    return BUTTON::RIGHT;
  return BUTTON::NONE;
}

void AutonSelector::update() {
  // if the auton selector is disabled, don't run
  if (!this->isEnabled()) return;

  static unsigned int prevButtons = pros::lcd::read_buttons();
  static unsigned int prevIndex = 0;
  const unsigned int currButtons = pros::lcd::read_buttons();

  // if left or right lcd buttons are pressed, change the index
  switch (getNewButtonPress(prevButtons, currButtons)) {
    // if the left button is pressed, decrement the index
    case BUTTON::LEFT: this->changeIndex(-1); break;
    // if the right button is pressed, increment the index
    case BUTTON::RIGHT: this->changeIndex(1); break;
    // if the center button is pressed, or if no button is pressed, do nothing
    default: break;
  }

  const unsigned int currIndex = this->getIndex();
  // if the index has changed, update the display
  if (currIndex != prevIndex) this->updateDisplay();
  prevIndex = currIndex;
}

AutonSelector::AutonSelector(pros::Rotation rotation, unsigned int gearTeeth)
  : rotation(rotation), gearTeeth(gearTeeth), autons(gearTeeth) {}