#include "wings.h"
#include "robot.h"

ADIPortConfig::ADIPortConfig(std::uint8_t adi_port) : adi_port(adi_port) {}

ADIPortConfig::ADIPortConfig(pros::ext_adi_port_pair_t port_pair)
  : port_pair(port_pair) {}

std::unique_ptr<pros::ADIDigitalOut> ADIPortConfig::makeDigitalOut(bool state) {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIDigitalOut>(this->adi_port.value(), state);
  else
    return std::make_unique<pros::ADIDigitalOut>(this->port_pair.value(),
                                                 state);
}

std::unique_ptr<pros::ADIDigitalIn> ADIPortConfig::makeDigitalIn() {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIDigitalIn>(this->adi_port.value());
  else
    return std::make_unique<pros::ADIDigitalIn>(this->port_pair.value());
}

std::unique_ptr<pros::ADIAnalogOut> ADIPortConfig::makeAnalogOut() {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIAnalogOut>(this->adi_port.value());
  else
    return std::make_unique<pros::ADIAnalogOut>(this->port_pair.value());
}

std::unique_ptr<pros::ADIAnalogIn> ADIPortConfig::makeAnalogIn() {
  if (this->adi_port.has_value())
    return std::make_unique<pros::ADIAnalogIn>(this->adi_port.value());
  else
    return std::make_unique<pros::ADIAnalogIn>(this->port_pair.value());
}

std::unique_ptr<ADIPortConfig> ADIPortConfig::makeUnique(std::uint8_t adi_port) {
  return std::make_unique<ADIPortConfig>(adi_port);
}


Solenoid::Solenoid(std::unique_ptr<ADIPortConfig> config, bool state)
  : AbstractSolenoid(), digitalOut(config->makeDigitalOut(state)),
    state(state) {}

void Solenoid::setState(bool newState) {
  this->state = newState;
  this->digitalOut->set_value(newState);
}

void Solenoid::enable() { this->setState(true); }

void Solenoid::disable() { this->setState(false); }

bool Solenoid::getState() const { return this->state; }

void Solenoid::toggle() { this->setState(!this->getState()); }

SolenoidGroup::SolenoidGroup(std::vector<std::unique_ptr<ADIPortConfig>> ports,
                             bool initialState)
  : AbstractSolenoid(), state(initialState) {
  for (auto& port : ports) {
    this->solenoids.push_back(
        std::make_unique<Solenoid>(std::move(port), initialState));
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

FourWingSubsystem::FourWingSubsystem(std::unique_ptr<WingPair> front,
                                     std::unique_ptr<WingPair> back,
                                     const int joystickThreshold)
  : front(std::move(front)), back(std::move(back)),
    joystickThreshold(joystickThreshold) {}

std::unique_ptr<FourWingSubsystem>
FourWingSubsystem::makeFromPortConfig(std::unique_ptr<PortConfig> portConfig,
                                      const int joystickThreshold) {
  std::unique_ptr<AbstractSolenoid> frontLeftSolenoid =
      std::make_unique<Solenoid>(std::move(portConfig->front.first));
  std::unique_ptr<AbstractSolenoid> frontRightSolenoid =
      std::make_unique<Solenoid>(std::move(portConfig->front.second));
  std::unique_ptr<AbstractSolenoid> backLeftSolenoid =
      std::make_unique<Solenoid>(std::move(portConfig->back.first));
  std::unique_ptr<AbstractSolenoid> backRightSolenoid =
      std::make_unique<Solenoid>(std::move(portConfig->back.second));

  auto front = std::make_unique<WingPair>(
      std::array {std::move(frontLeftSolenoid), std::move(frontRightSolenoid)});
  auto back = std::make_unique<WingPair>(
      std::array {std::move(backLeftSolenoid), std::move(backRightSolenoid)});

  return std::make_unique<FourWingSubsystem>(std::move(front), std::move(back),
                                             joystickThreshold);
}

void FourWingSubsystem::retractAll() {
  this->front->disable();
  this->back->disable();
}

void FourWingSubsystem::driverUpdate() {
  enum SELECTED_WING_PAIR {
    /** joystick is being pushed outward more than this->joystickThreshold */
    FRONT,
    /** joystick is being pushed inward more than this->joystickThreshold */
    BACK,
    /** joystick is being within this->joystickThreshold */
    NONE,
  };

  static bool prevR1 = false;
  const bool r1 = Robot::control.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

  // if on rising edge
  if (r1 && !prevR1) {
    // get left and right joystick x values
    const float leftX =
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    const float rightX =
        Robot::control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // determine which wing the driver is trying to toggle
    const SELECTED_WING_PAIR left = std::abs(leftX) > this->joystickThreshold
                                        ? (leftX < 0 ? FRONT : BACK)
                                        : NONE;
    const SELECTED_WING_PAIR right = std::abs(rightX) > this->joystickThreshold
                                         ? (rightX > 0 ? FRONT : BACK)
                                         : NONE;
    const bool anySelected = left != NONE || right != NONE;
    if (!anySelected) {
      // if any back wing is expanded, retract them
      if (this->back->summarizeStates() != WingPair::DISABLED)
        this->back->disable();
      // otherwise toggle the front wing
      else this->front->toggleToSame();
    } else {
      if (left == FRONT)
        this->front->toggleIthSolenoid(int(WING_PAIR_INDEX::LEFT));
      else if (left == BACK)
        this->back->toggleIthSolenoid(int(WING_PAIR_INDEX::LEFT));

      if (right == FRONT)
        this->front->toggleIthSolenoid(int(WING_PAIR_INDEX::RIGHT));
      else if (right == BACK)
        this->back->toggleIthSolenoid(int(WING_PAIR_INDEX::RIGHT));
    }
  }

  prevR1 = r1;
}