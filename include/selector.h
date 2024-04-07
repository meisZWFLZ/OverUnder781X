#pragma once
#include "pros/rotation.hpp"
#include <vector>
#include <functional>

namespace auton {

struct Auton {
    char* label;
    std::function<void(void)> run;
    std::optional<char*> labelForController;
};

class AutonSelector {
  private:
    std::vector<Auton*> autons;
    /** Whether the AutonSelector is enabled */
    bool state = false;

    // represents the offset of the rotation sensor
    float offsetDegrees = 0;

    const unsigned int gearTeeth;

    void clearDisplay();
    void updateDisplay();

    // remove once LVGL is done
    void initScreen();

    /**
     * @brief Returns the rotation's angle in degrees in the range of [0, 360).
     * Uses rotation.getPosition() to get the angle
     *
     * @return The rotation's angle in degrees in the range of [0, 360)
     */
    float getRotationAngle();

    float setRotationPosition(float newPositionDegrees);

    pros::Rotation rotation;
  protected:
    float getDegreesPerAuton() const;
  public:
    AutonSelector(pros::Rotation rotation, unsigned int gearTeeth);

    bool isRotationConnected();

    unsigned int getIndex();
    void setIndex(unsigned int newIndex);
    void changeIndex(int indexChange);

    /**
     * @return true when auton is added
     * @return false when auton failed to be added due to too many autons
     */
    bool addAuton(Auton* auton);
    void runAuton();

    void enable();
    /**
     * @brief prevents the changing of the currently selected auton and disables printing to the screen
     */
    void disable();
    bool isEnabled();

    char* getCurrentAuton();

    void init();

    /**
     * @brief changes auton based on rotation sensor
     */
    void update();

    /**
     * @brief when the competition status is enabled, the auton selector will
     * not change selected autons
     */
    bool freezeAutonWhenCompEnabled = false;
};

} // namespace auton