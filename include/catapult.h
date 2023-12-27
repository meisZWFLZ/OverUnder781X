#pragma once

#include "lemlib/timer.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cstdint>

class CatapultStateMachine {
  public:
    CatapultStateMachine(pros::Motor_Group* cataMotors,
                         pros::ADILineSensor* triballSensor,
                         pros::Rotation* cataRotation);

    enum STATE {
      IDLE, // retracted
      LOADING, // waiting to receive matchload
      FIRING, // shooting triball
      RETRACTING // retracting catapult
    };

    /**
     * @brief Attempt to fire the catapult
     *
     * @return Whether the catapult was able to fire
     */
    bool fire();

    /**
     * @brief Matchload the catapult
     *
     * @param millis max amount of time the catapult will spend matchloading
     * @param triballs how many triballs to fire
     *
     * @return Whether the catapult was able to matchload
     */
    bool matchload(int millis = INT32_MAX, int triballs = -1);
    void stop();

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
    /**
     * @return whether the rotation sensor indicates the catapult is retracted
     */
    bool isCataLoadable() const;

    void update();
  private:
    bool isTriballLoaded() const;

    lemlib::Timer timer = {0};

    int triballsLeftToBeFired = 0;
    int triballsFired = 0;

    pros::Motor_Group* motors;
    pros::ADILineSensor* triballSensor;
    pros::Rotation* rotation;

    STATE state = IDLE;
    bool matchloading = false;
    bool stopping = false;
};