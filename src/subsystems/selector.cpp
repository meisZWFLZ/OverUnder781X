#include "selector.h"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "robot.h"
#include <cfloat>
#include <cmath>

using namespace auton;

const std::string ERR = "\x1b[31m[ERR]\x1b[0m";

bool AutonSelector::isRotationConnected() {
  return this->rotation.get_position() != PROS_ERR;
}

float AutonSelector::getDegreesPerAuton() const {
  const int teethPerAuton = floor(float(this->gearTeeth) / this->autons.size());
  float degreesPerAuton = 360.0 * (float(teethPerAuton) / this->gearTeeth);

  // if there more autons than teeth (should be impossible), set the degrees per
  // auton to 360 / autons.size()
  if (teethPerAuton < 1) degreesPerAuton = 360.0 / this->autons.size();

  return degreesPerAuton;
}

float AutonSelector::getRotationAngle() {
  const float degrees =
      this->isRotationConnected()
          ? float(this->rotation.get_position()) / 100.0 + this->offsetDegrees
          : this->offsetDegrees;

  /* has a range of (-180, 180) (not sure about inclusive/exclusive)*/
  const float possiblyNegativeAngle = std::fmod(degrees, 360.0);
  return possiblyNegativeAngle < 0 ? 360.0 + possiblyNegativeAngle
                                   : possiblyNegativeAngle;
}

float AutonSelector::setRotationPosition(float newPositionDegrees) {
  const bool shouldSetSensor =
      !pros::competition::is_disabled() && this->isRotationConnected();
  const float oldOffsetDegrees = this->offsetDegrees;
  const float oldPositionDegrees = this->getRotationAngle();
  if (shouldSetSensor) {
    this->offsetDegrees = 0;
    this->rotation.set_position(newPositionDegrees * 100);
    printf("AutonSelector::setRotationPosition(): setting position to: %4.2f\n",
           newPositionDegrees * 100);
  } else {
    this->offsetDegrees += newPositionDegrees - getRotationAngle();
  }

  if (std::fabs(this->getRotationAngle() - newPositionDegrees) > 0.01) {
    printf((ERR +
            "AutonSelector::setRotationPosition(): failed to set position\n"
            "\tnewPositionDegrees:\t%4.2f\n"
            "\tgetRotationAngle():\t%4.2f\n"
            "\tshouldSetSensor:\t%i\n"
            "\toldOffsetDegrees:\t%4.2f\n"
            "\toldPositionDegrees:\t%4.2f\n")
               .c_str(),
           newPositionDegrees, this->getRotationAngle(), shouldSetSensor,
           oldOffsetDegrees, oldPositionDegrees);
  }

  return this->getRotationAngle();
}

unsigned int AutonSelector::getIndex() {
  const float degreesPerAuton = this->getDegreesPerAuton();
  // printf("degreesPerAuton: %4.2f\n", degreesPerAuton);
  const float sensedDegrees = this->getRotationAngle();
  // printf("sensedDegrees: %4.2f\n", sensedDegrees);
  int index = floor(sensedDegrees / degreesPerAuton);

  // if index is out of bounds, return the last auton
  if (index >= autons.size()) index = autons.size() - 1;
  if (index < 0 || index >= autons.size()) {
    printf(
        (ERR + "AutonSelector::getIndex(): index out of bounds: %i\n").c_str(),
        index);
    return autons.size() - 1;
  }
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

  const float degreesPerAuton = this->getDegreesPerAuton();

  const float minTargetDegrees = newIndex * degreesPerAuton;
  const float currentDegrees = this->getRotationAngle();

  // The minimum degrees to be at the current index
  const float currIndexMinDegrees = currIndex * degreesPerAuton;
  // The degrees past currIndexMinDegrees
  const float additionalDegrees = currentDegrees - currIndexMinDegrees;

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
  const float bigTargetDegrees = minTargetDegrees + additionalDegrees;

  // limit targetDegrees to the range of [0, 360)
  const float targetDegrees = std::fmod(bigTargetDegrees, 360.0) < 0
                                  ? 360.0 + std::fmod(bigTargetDegrees, 360.0)
                                  : std::fmod(bigTargetDegrees, 360.0);

  // Set the position of the rotation sensor
  // im not sure if set_position takes centidegrees
  this->setRotationPosition(targetDegrees);
  if (fabs(this->getRotationAngle() - targetDegrees) > 0.1) {
    // print some debug info
    printf((ERR +
            "AutonSelector::setIndex(): failed to correctly set position\n"
            "\ttargetDegrees:\t%4.2f\n"
            "\tgetRotationAngle():\t%4.2f\n")
               .c_str(),
           targetDegrees, this->getRotationAngle());
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
                  "\ttargetDegrees:\t%4.2f\n"
                  "\tdegreesPerAuton:\t%4.2f\n"
                  "\tadditionalDegrees:\t%4.2f\n"
                  "\tgetRotationAngle():\t%4.2f\n")
               .c_str(),
           newIndex, currIndex, this->getIndex(), currentDegrees,
           minTargetDegrees, targetDegrees, degreesPerAuton, additionalDegrees,
           this->getRotationAngle());
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
  Auton* auton = this->autons[this->getIndex()];
  const std::string brainStr(auton->label);

  pros::lcd::clear_line(0);
  pros::lcd::print(0, (brainStr + ": %d/%d").c_str(), this->getIndex() + 1,
                   this->autons.size());

  std::string controllerStr(auton->labelForController.has_value()
                                ? auton->labelForController.value()
                                : brainStr);
  // clear previous entry
  while (controllerStr.length() < 20) controllerStr += " ";

  printf("AutonSelector::updateDisplay(): controllerStr: %s$END$\n",
         controllerStr.c_str());
  Robot::control.print(2, 0, controllerStr.c_str());
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
  // const size_t index = this->getIndex();
  // printf("AutonSelector::getCurrentAuton(): this->autons.size(): %i\n",
  // this->autons.size()); printf("AutonSelector::getCurrentAuton(): index:
  // %i\n", index); pros::delay(100); printf("AutonSelector::getCurrentAuton():
  // label: %s\n", autons.at(index)->label); pros::delay(100);
  return autons.at(this->getIndex())->label;
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
  const bool prevButton = (prevState & buttonToMask(button)) != 0;
  const bool currButton = (currState & buttonToMask(button)) != 0;
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
  static bool hasBeenEnabled = false;
  // if the auton selector is disabled, don't run
  if (!this->isEnabled()) return;

  // if competition is enabled, update rotation's set position and maybe freeze
  // the auton selector
  if (!pros::competition::is_disabled()) {
    if (!hasBeenEnabled)
      printf("AutonSelector::update(): competition enabled\n");
    hasBeenEnabled=true;
    // if the rotation sensor is connected and the offset is not 0, set the new
    // position (saves the auton selection)
    if (this->isRotationConnected() && this->offsetDegrees != 0) {
      printf("AutonSelector::update(): saving position\n");
      this->setRotationPosition(this->getRotationAngle());
    }

    // if the auton selector should be frozen when competition is enabled, do
    // not check for button presses
    if (freezeAutonWhenCompEnabled) return;
  }

  static unsigned int prevButtons = pros::lcd::read_buttons();
  static unsigned int prevIndex = 0;
  const unsigned int currButtons = pros::lcd::read_buttons();

  // if left or right lcd buttons are pressed, change the index
  switch (getNewButtonPress(prevButtons, currButtons)) {
    // if the left button is pressed, decrement the index
    case BUTTON::LEFT:
      printf("prevButtons: %i, currButtons: %i\n", prevButtons, currButtons);
      this->changeIndex(-1);
      break;
    // if the right button is pressed, increment the index
    case BUTTON::RIGHT:
      printf("prevButtons: %i, currButtons: %i\n", prevButtons, currButtons);
      this->changeIndex(1);
      break;
    // if the center button is pressed, or if no button is pressed, do nothing
    default: break;
  }

  const unsigned int currIndex = this->getIndex();
  // if the index has changed, update the display
  if (currIndex != prevIndex) this->updateDisplay();
  prevIndex = currIndex;
  prevButtons = currButtons;
}

AutonSelector::AutonSelector(pros::Rotation rotation, unsigned int gearTeeth)
  : rotation(rotation), gearTeeth(gearTeeth), autons() {}