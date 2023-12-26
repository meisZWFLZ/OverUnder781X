#pragma once

#include "lemlib/timer.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <cstdint>

class CatapultStateMachine {
  public:
    enum STATE {
      IDLE, // retracted
      LOADING, // waiting to receive matchload
      FIRING, // shooting triball
      RETRACTING // retracting catapult
    };

    void fire();

    /**
     * @brief Matchload the catapult
     *
     * @param millis max amount of time the catapult will spend matchloading
     * @param triballs how many triballs to fire
     */
    void matchload(int millis = INT32_MAX, int triballs = -1);
    void stopMatchloading();

    STATE getState() const;

    /**
     * @return How many triballs haven't been fired from current matchloading
     * session
     */
    int getTriballsLeft() const;
    /**
     * @return How many triballs been fired from catapult this match
     */
    int getTriballsFired() const;
    /**
     * @return Whether the catapult is matchloading or not
     */
    bool getIsMatchloading() const;

    CatapultStateMachine(pros::Motor_Group* cataMotors,
                         pros::ADILineSensor* triballSensor);

    void update();
  private:
    bool isTriballLoaded() const;

    lemlib::Timer timer = {0};

    int triballsLeftToBeFired = 0;
    int triballsFired = 0;

    pros::Motor_Group* motors;
    pros::ADILineSensor* sensor;

    STATE state = IDLE;
    bool matchloading = false;
};