#pragma once

#include "pros/adi.hpp"
#include <memory>
#include <optional>
#include <vector>

class ADIPortConfig {
  public:
    ADIPortConfig(std::uint8_t adi_port);
    ADIPortConfig(pros::ext_adi_port_pair_t port_pair);

    std::unique_ptr<pros::ADIDigitalOut> makeDigitalOut(bool state = false);
    std::unique_ptr<pros::ADIDigitalIn> makeDigitalIn();
    std::unique_ptr<pros::ADIAnalogOut> makeAnalogOut();
    std::unique_ptr<pros::ADIAnalogIn> makeAnalogIn();

    static std::unique_ptr<ADIPortConfig> makeUnique(std::uint8_t adi_port);
    static std::unique_ptr<ADIPortConfig>
    makeUnique(pros::ext_adi_port_pair_t port_pair);
  private:
    std::optional<std::uint8_t> adi_port;
    std::optional<pros::ext_adi_port_pair_t> port_pair;
};

class AbstractSolenoid {
  public:
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void toggle() = 0;
    virtual void setState(bool newState) = 0;
    virtual bool getState() const = 0;
};

class Solenoid : public AbstractSolenoid {
  public:
    Solenoid(std::unique_ptr<ADIPortConfig> config, bool initialState = false);

    /**
     * @brief sets the state of the solenoid to true
     */
    void enable() override;
    /**
     * @brief sets the state of the solenoid to false
     */
    void disable() override;
    /**
     * @brief toggles the state of the solenoid
     */
    void toggle() override;
    /**
     * @brief sets the state of the solenoid
     */
    void setState(bool newState) override;
    /**
     * @brief gets the state of the solenoid
     * @return the state of the solenoid
     */
    bool getState() const override;
  private:
    bool state = false;
    std::unique_ptr<pros::ADIDigitalOut> digitalOut;
};

/**
 * @brief ensures that all solenoids have the same value
 */
class SolenoidGroup : public AbstractSolenoid {
  public:
    SolenoidGroup(std::vector<std::unique_ptr<ADIPortConfig>> ports,
                  bool initialState = false);
    void enable() override;
    void disable() override;
    void toggle() override;
    void setState(bool newState) override;
    bool getState() const override;
  private:
    std::vector<std::unique_ptr<Solenoid>> solenoids;
    bool state = false;
};

template <size_t N> class SolenoidSet {
  public:
    SolenoidSet(std::array<std::unique_ptr<AbstractSolenoid>, N> solenoids) {
      solenoids.swap(this->solenoids);
    };

    /**
     * @brief enables all solenoids
     */
    void enable() { this->setAllSolenoids(true); };

    /**
     * @brief disables all solenoids
     */
    void disable() { this->setAllSolenoids(false); };

    /**
     * @brief toggle each solenoid independently
     */
    void toggleEach() {
      for (auto& solenoid : this->solenoids) solenoid->toggle();
    }

    /**
     * @brief toggle all solenoids to the same state
     * @param ifDiffState if solenoids are in different states, which state
     * should the solenoids be set to (defaults to disabled)
     *
     * @return true if the solenoids were in different states, false otherwise
     */
    bool toggleToSame(bool ifDiffState = false) {
      // could be optimized, but this is the most readable
      switch (this->summarizeStates()) {
        case DIFFERENT: this->setAllSolenoids(ifDiffState); return true;
        case ENABLED: this->disable(); break;
        case DISABLED: this->enable(); break;
      }
      return false;
    };

    /**
     * @brief sets the state of all solenoids to newState
     * @param newState the new state of the solenoids
     */
    void setAllSolenoids(bool newState) {
      for (auto& solenoid : this->solenoids) solenoid->setState(newState);
    };

    /**
     * @brief Set the ith state
     *
     * @param i the index of the solenoid
     * @param newState new state
     */
    void setIthState(size_t i, bool newState) {
      this->solenoids.at(i)->setState(newState);
    };

    enum STATE_SUMMARY {
      /** all solenoids are disabled */
      DISABLED = false,
      /** all solenoids are enabled */
      ENABLED = true,
      /** some solenoids are enabled and some are disabled */
      DIFFERENT = -1,
    };

    /**
     * @brief summarize the states of the solenoids
     */
    STATE_SUMMARY summarizeStates() const {
      bool firstState = this->getIthState(0);
      for (auto state : this->getStates())
        if (state != firstState) return DIFFERENT;
      if (firstState) return ENABLED;
      else return DISABLED;
    };

    /**
     * @brief get the states of the solenoids
     * @return an array of the states of the solenoids
     */
    std::array<bool, N> getStates() const {
      std::array<bool, N> states;
      for (int i = 0; i < N; i++) states[i] = this->getIthState(i);

      return states;
    };

    /**
     * @brief get the state of the ith solenoid
     * @param I the index of the solenoid
     * @return the state of the ith solenoid
     */
    bool getIthState(size_t i) const {
      return solenoids.at(i)->getState();
    }

    /**
     * @brief toggle the ith solenoid
     * @param i the index of the solenoid
     */
    void toggleIthSolenoid(size_t i) { solenoids.at(i)->toggle(); }
  private:
    std::array<std::unique_ptr<AbstractSolenoid>, N> solenoids;
};

enum class WING_PAIR_INDEX {
  LEFT = 0,
  RIGHT = 1,
};

using WingPair = SolenoidSet<2>;

class FourWingSubsystem {
  public:
    FourWingSubsystem(std::unique_ptr<WingPair> front,
                      std::unique_ptr<WingPair> back,
                      const int joystickThreshold);
    std::unique_ptr<WingPair> front;
    std::unique_ptr<WingPair> back;

    struct PortConfig {
        std::pair<std::unique_ptr<ADIPortConfig>,
                  std::unique_ptr<ADIPortConfig>>
            front;
        std::pair<std::unique_ptr<ADIPortConfig>,
                  std::unique_ptr<ADIPortConfig>>
            back;
    };

    static std::unique_ptr<FourWingSubsystem>
    makeFromPortConfig(std::unique_ptr<PortConfig> portConfig,
                       const int joystickThreshold);

    void retractAll();

    const int joystickThreshold;
    void driverUpdate();
};