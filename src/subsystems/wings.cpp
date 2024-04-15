#include "wings.h"
#include "robot.h"
#include <memory>
#include <vector>

ADIPortConfig::ADIPortConfig(std::uint8_t adi_port) : adi_port(adi_port) {}

ADIPortConfig::ADIPortConfig(pros::ext_adi_port_pair_t port_pair)
  : port_pair(port_pair) {}

std::unique_ptr<pros::ADIDigitalOut>
ADIPortConfig::makeDigitalOut(bool state) const {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIDigitalOut>(this->adi_port.value(), state);
  else
    return std::make_unique<pros::ADIDigitalOut>(this->port_pair.value(),
                                                 state);
}

std::unique_ptr<pros::ADIDigitalIn> ADIPortConfig::makeDigitalIn() const {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIDigitalIn>(this->adi_port.value());
  else return std::make_unique<pros::ADIDigitalIn>(this->port_pair.value());
}

std::unique_ptr<pros::ADIAnalogOut> ADIPortConfig::makeAnalogOut() const {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIAnalogOut>(this->adi_port.value());
  else return std::make_unique<pros::ADIAnalogOut>(this->port_pair.value());
}

std::unique_ptr<pros::ADIAnalogIn> ADIPortConfig::makeAnalogIn() const {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIAnalogIn>(this->adi_port.value());
  else return std::make_unique<pros::ADIAnalogIn>(this->port_pair.value());
}

std::unique_ptr<ADIPortConfig>
ADIPortConfig::makeUnique(std::uint8_t adi_port) {
  return std::make_unique<ADIPortConfig>(adi_port);
}

Solenoid::Solenoid(const ADIPortConfig& config, bool state)
  : AbstractSolenoid(), digitalOut(config.makeDigitalOut(state)), state(state) {
}

void Solenoid::setState(bool newState) {
  this->state = newState;
  this->digitalOut->set_value(newState);
}

void Solenoid::enable() { this->setState(true); }

void Solenoid::disable() { this->setState(false); }

bool Solenoid::getState() const { return this->state; }

void Solenoid::toggle() { this->setState(!this->getState()); }

SolenoidGroup::SolenoidGroup(std::vector<ADIPortConfig> ports,
                             bool initialState)
  : AbstractSolenoid(), state(initialState) {
  for (auto& port : ports) {
    this->solenoids.push_back(std::make_unique<Solenoid>(port, initialState));
  }
}

void SolenoidGroup::enable() { this->setState(true); }

void SolenoidGroup::disable() { this->setState(false); }

void SolenoidGroup::toggle() {
  this->state = !this->state;
  for (auto& solenoid : this->solenoids) { solenoid->setState(this->state); }
}

void SolenoidGroup::setState(bool newState) {
  this->state = newState;
  for (auto& solenoid : this->solenoids) { solenoid->setState(newState); }
}

bool SolenoidGroup::getState() const { return this->state; }

FourWingSubsystem::FourWingSubsystem(WingPair* front, WingPair* back,
                                     const int joystickThreshold)
  : front(front), back(back), joystickThreshold(joystickThreshold) {
  printf("constructing four wing subsytem\n");
  printf("front size: %d\n", front->size());
  printf("back size: %d\n", back->size());
  printf("constructing four wing subsytem\n");
}

FourWingSubsystem*
FourWingSubsystem::makeFromPortConfig(const PortConfig& portConfig,
                                      const int joystickThreshold) {
  printf("making four wing subsystem\n");
  AbstractSolenoid* frontLeftSolenoid = new Solenoid(portConfig.front.first);
  AbstractSolenoid* frontRightSolenoid = new Solenoid(portConfig.front.second);
  AbstractSolenoid* backLeftSolenoid = new Solenoid(portConfig.back.first);
  AbstractSolenoid* backRightSolenoid = new Solenoid(portConfig.back.second);

  std::vector<AbstractSolenoid*> frontVector {};
  frontVector.push_back(frontLeftSolenoid);
  frontVector.push_back(frontRightSolenoid);
  std::vector<AbstractSolenoid*> backVector;
  backVector.push_back(backLeftSolenoid);
  backVector.push_back(backRightSolenoid);

  WingPair* front = new WingPair {frontVector};
  WingPair* back = new WingPair(backVector);

  return new FourWingSubsystem(front, back, joystickThreshold);
}

void FourWingSubsystem::retractAll() {
  printf("retract all\n");
  this->front->disable();
  this->back->disable();
}

void FourWingSubsystem::driverUpdate() {
  // printf("driver update\n");

  enum SELECTED_WING_PAIR {
    /** joystick is being pushed outward more than this->joystickThreshold */
    LEFT,
    /** joystick is being pushed inward more than this->joystickThreshold */
    RIGHT,
    /** joystick is being within this->joystickThreshold */
    NONE,
  };

  static bool prevR1 = false;
  const bool r1 = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

  // if on rising edge
  if (r1 && !prevR1) {
    printf("driver rising edge\n");
    // get left and right joystick x values
    const float leftX =
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    const float rightX =
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // determine which wing the driver is trying to toggle
    const SELECTED_WING_PAIR front = std::abs(leftX) > this->joystickThreshold
                                         ? (leftX < 0 ? LEFT : RIGHT)
                                         : NONE;
    const SELECTED_WING_PAIR back = std::abs(rightX) > this->joystickThreshold
                                        ? (rightX > 0 ? LEFT : RIGHT)
                                        : NONE;
    const bool anySelected = front != NONE || back != NONE;
    if (!anySelected) {
      printf("nothing selected\n");
      // if any wing is expanded, then retract all
      if (this->back->summarizeStates() != WingPair::DISABLED ||
          this->front->summarizeStates() != WingPair::DISABLED) {
        this->retractAll();
      }
      // otherwise toggle the front wing
      else
        this->front->toggleToSame();
    } else {
      printf("something selected\n");
      if (front == LEFT)
        this->front->toggleIthSolenoid(int(WING_PAIR_INDEX::LEFT));
      else if (front == RIGHT)
        this->front->toggleIthSolenoid(int(WING_PAIR_INDEX::RIGHT));

      if (back == LEFT)
        this->back->toggleIthSolenoid(int(WING_PAIR_INDEX::LEFT));
      else if (back == RIGHT)
        this->back->toggleIthSolenoid(int(WING_PAIR_INDEX::RIGHT));
    }
  }

  prevR1 = r1;
}
