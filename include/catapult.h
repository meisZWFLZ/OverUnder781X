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
      READY, // retracted and ready to fire
      FIRING, // shooting triball
      RETRACTING, // retracting catapult
      EMERGENCY_STOPPED // emergency stop
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
    /**
     * @brief Forces the catapult to stop immediately
     */
    void emergencyStop();
    /**
     * @brief Resumes the catapult's functions after an emergency stop
     */
    void cancelEmergencyStop();

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
    /**
     * @return whether the rotation sensor indicates the catapult is not retracted
     */
    bool isCataNotLoadable() const;

    /**
     * @brief Blocks current thread until match loading is finished.
     */
    void waitUntilDoneMatchloading();

    void update();
  private:
    /**
     * @brief Modify triballsLeftToBeFired and triballsFired to indicate a
     * triball has been fired
     */
    void indicateTriballFired();
    void retractCataMotor();
    void stopCataMotor();
    
    bool isElevationBarSensed() const;

    bool constantFiring = false;

    lemlib::Timer timer = {0};

    int triballsLeftToBeFired = 0;
    int triballsFired = 0;

    pros::Motor_Group* motors;
    pros::ADILineSensor* elevationBarSensor;
    pros::Rotation* rotation;

    STATE state = READY;
    bool matchloading = false;
};