#pragma once

#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <memory>
#include <vector>

class ADIPortConfig {
  public:
    ADIPortConfig(std::uint8_t adi_port);
    ADIPortConfig(pros::ext_adi_port_pair_t port_pair);

    std::unique_ptr<pros::ADIDigitalOut>
    makeDigitalOut(bool state = false) const;
    std::unique_ptr<pros::ADIDigitalIn> makeDigitalIn() const;
    std::unique_ptr<pros::ADIAnalogOut> makeAnalogOut() const;
    std::unique_ptr<pros::ADIAnalogIn> makeAnalogIn() const;

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
    Solenoid(const ADIPortConfig& config, bool initialState = false);

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
    SolenoidGroup(std::vector<ADIPortConfig> ports, bool initialState = false);
    void enable() override;
    void disable() override;
    void toggle() override;
    void setState(bool newState) override;
    bool getState() const override;
  private:
    std::vector<std::unique_ptr<Solenoid>> solenoids;
    bool state = false;
};

/**
 * @brief Groups N solenoids together, but allows for independent control
 *
 * @param N number of solenoids
 */
class SolenoidSet {
  public:
    SolenoidSet(std::vector<AbstractSolenoid*> solenoids) {
      printf("constructing solenoid set\n");
      printf("size: %d\n", solenoids.size());
      solenoids.swap(this->solenoids);
    };

    /**
     * @brief enables all solenoids
     */
    void enable() {
      printf("enabling\n");
      this->setAllSolenoids(true);
    };

    /**
     * @brief disables all solenoids
     */
    void disable() {
      printf("disabling\n");
      this->setAllSolenoids(false);
    };

    /**
     * @brief toggle each solenoid independently
     */
    void toggleEach() {
      printf("toggle each -ing\n");
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
      printf("toggle same -ing\n");
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
      printf("setting all to %d\n", newState);
      for (auto& solenoid : this->solenoids) solenoid->setState(newState);
    };

    /**
     * @brief Set the ith state
     *
     * @param i the index of the solenoid
     * @param newState new state
     */
    void setIthState(size_t i, bool newState) {
      printf("i: %d\n", i);
      printf("setting to %d\n", newState);
      printf("size: %d\n", this->solenoids.size());
      printf("not null?: %d\n", (bool)this->solenoids.at(i));

      pros::delay(200);

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
      printf("summarizing states\n");
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
    std::vector<bool> getStates() const {
      printf("getting states\n");
      std::vector<bool> states;
      for (int i = 0; i < this->solenoids.size(); i++)
        states.push_back(this->solenoids[i]->getState());
      return states;
    };

    /**
     * @brief get the state of the ith solenoid
     * @param I the index of the solenoid
     * @return the state of the ith solenoid
     */
    bool getIthState(size_t i) const {
      printf("getting %dth solenoid state\n", i);
      return solenoids.at(i)->getState();
    }

    /**
     * @brief toggle the ith solenoid
     * @param i the index of the solenoid
     */
    void toggleIthSolenoid(size_t i) {
      printf("toggling %dth solenoid\n", i);
      solenoids.at(i)->toggle();
    }

    size_t size() const { return this->solenoids.size(); }
  private:
    std::vector<AbstractSolenoid*> solenoids;
};

enum class WING_PAIR_INDEX {
  LEFT = 0,
  RIGHT = 1,
};

using WingPair = SolenoidSet;

class FourWingSubsystem {
  public:
    FourWingSubsystem(WingPair* front, WingPair* back,
                      const int joystickThreshold);
    WingPair* front;
    WingPair* back;

    struct PortConfig {
        std::pair<const ADIPortConfig&, const ADIPortConfig&> front;
        std::pair<const ADIPortConfig&, const ADIPortConfig&> back;
    };

    static FourWingSubsystem*
    makeFromPortConfig(const PortConfig& portConfig,
                       const int joystickThreshold);

    void retractAll();

    const int joystickThreshold;
    void driverUpdate();
};